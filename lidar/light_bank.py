"""LIGHT BANK -- all-day synchronized camera + LiDAR sampler.

Every --interval seconds grabs one camera frame (RTSP) and a short LiDAR
burst (UDP Compact), saves both, and appends light/edge statistics to a CSV
so the day's full lighting range can be judged for camera-based diameter
work (is the zone-4 log edge visible at dawn / full sun / dusk / night?).

    python lidar/light_bank.py --interval 300           # all day
    python lidar/light_bank.py --once                   # single sample test

Output: lidar_recordings/lightbank_<YYYYMMDD>/
    HHMMSS_camera.jpg     full-res frame (JPEG q90)
    HHMMSS_lidar.pkl      {"frames": {fn: [segs]}, "t": epoch}
    lightbank.csv         one row per sample (see CSV_FIELDS)

The UDP socket is opened only during each burst so other LiDAR tools can
use port 2115 between samples; if the port is busy the LiDAR part is
skipped for that sample and noted in the CSV.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import pickle
import socket
import sys
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

ROOT = Path(__file__).resolve().parent.parent
from lidar.camera_url import cam_url  # noqa: E402
CAM_URL = cam_url()                    # from the CAM_URL environment variable
ZONES_JSON = ROOT / "lidar" / "camera_zones.json"
CSV_FIELDS = ["iso_time", "epoch", "cam_ok", "mode", "gray_mean", "gray_p5",
              "gray_p95", "chroma", "z4_edge", "z4_gray", "sat_frac",
              "lidar_ok", "lidar_frames", "lidar_segs", "z4_pts", "note"]


# ---------------------------------------------------------------- camera
def grab_camera(url: str, flush: int = 6, timeout_s: float = 8.0):
    import cv2
    os.environ.setdefault("OPENCV_FFMPEG_CAPTURE_OPTIONS",
                          "rtsp_transport;tcp|stimeout;5000000")
    cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
    t0 = time.time()
    img = None
    try:
        n = 0
        while time.time() - t0 < timeout_s:
            ok, fr = cap.read()
            if not ok:
                time.sleep(0.05)
                continue
            n += 1
            img = fr
            if n > flush:          # skip the stale decoder backlog
                break
    finally:
        cap.release()
    return img


def _z4_mask(shape):
    """Boolean mask of the zone-4 fan band from camera_zones.json."""
    h, w = shape[:2]
    try:
        cz = json.loads(ZONES_JSON.read_text())
        ax, ay = cz["apex"]
        band = next(b for b in cz["bands"] if b.get("zone") == 4)
    except Exception:
        return None
    ys, xs = np.mgrid[0:h, 0:w]
    ang = np.degrees(np.arctan2(ys - ay, xs - ax))
    return (ang >= band["lo"]) & (ang < band["hi"])


_Z4 = None


def camera_stats(img):
    import cv2
    global _Z4
    b, g, r = [img[..., i].astype(np.float32) for i in range(3)]
    chroma = float(np.mean(np.abs(b - g)) + np.mean(np.abs(g - r)))
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if _Z4 is None or _Z4.shape != gray.shape:
        _Z4 = _z4_mask(gray.shape)
    gx = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    gy = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    mag = np.sqrt(gx * gx + gy * gy)
    if _Z4 is not None:
        z4_edge = float(np.mean(mag[_Z4]))
        z4_gray = float(np.mean(gray[_Z4]))
    else:
        z4_edge = float(np.mean(mag))
        z4_gray = float(np.mean(gray))
    return {
        "mode": "mono" if chroma < 1.0 else "color",
        "gray_mean": round(float(gray.mean()), 1),
        "gray_p5": float(np.percentile(gray, 5)),
        "gray_p95": float(np.percentile(gray, 95)),
        "chroma": round(chroma, 2),
        "z4_edge": round(z4_edge, 1),
        "z4_gray": round(z4_gray, 1),
        "sat_frac": round(float(np.mean(gray >= 250)), 4),
    }


# ----------------------------------------------------------------- lidar
def lidar_burst(port: int, seconds: float):
    """Collect complete frames for `seconds`; returns ({fn: segs}, nsegs)."""
    from scansegmentapi.compact import parse_payload
    from scansegmentapi.udp_handler import UDPHandler
    tr = UDPHandler("0.0.0.0", port, 1 << 16)
    frames = defaultdict(list)
    nseg = 0
    try:
        tr.client.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
        tr.client.settimeout(1.0)
        t0 = time.time()
        while time.time() - t0 < seconds:
            try:
                payload = tr.receive_new_scan_segment()
            except socket.timeout:
                continue
            if isinstance(payload, tuple):
                payload = payload[0]
            try:
                seg = parse_payload(payload)
            except Exception:
                continue
            mods = seg.get("Modules", [])
            if not mods:
                continue
            frames[mods[0].get("FrameNumber", 0)].append(seg)
            nseg += 1
    finally:
        try:
            tr.client.close()
        except Exception:
            pass
    # keep complete frames only (drop first/last partials)
    keys = sorted(frames)
    out = {k: frames[k] for k in keys if len(frames[k]) >= 11}
    return out, nseg


def z4_point_count(frames):
    try:
        from lidar.probe_zones import PRESENCE_WINDOWS
        from lidar.pointcloud import segment_points
    except Exception:
        return -1
    w = PRESENCE_WINDOWS["4_TopOfScrew"]
    n = 0
    for segs in frames.values():
        for seg in segs:
            p, _ = segment_points(seg)
            if len(p):
                sel = ((p[:, 0] >= w["x"][0]) & (p[:, 0] <= w["x"][1]) &
                       (p[:, 1] >= w["y"][0]) & (p[:, 1] <= w["y"][1]))
                n += int(sel.sum())
    return n


# ---------------------------------------------------------------- sample
def take_sample(out: Path, port: int, burst_s: float, cam_url: str):
    import cv2
    t = time.time()
    stamp = time.strftime("%H%M%S", time.localtime(t))
    row = {k: "" for k in CSV_FIELDS}
    row["iso_time"] = datetime.fromtimestamp(t).isoformat(timespec="seconds")
    row["epoch"] = round(t, 3)
    notes = []

    img = None
    try:
        img = grab_camera(cam_url)
    except Exception as e:          # noqa: BLE001
        notes.append(f"cam:{type(e).__name__}")
    if img is not None:
        cv2.imwrite(str(out / f"{stamp}_camera.jpg"), img,
                    [cv2.IMWRITE_JPEG_QUALITY, 90])
        row.update(camera_stats(img))
        row["cam_ok"] = 1
    else:
        row["cam_ok"] = 0
        if not notes:
            notes.append("cam:no_frame")

    frames = {}
    try:
        if burst_s > 0:
            frames, nseg = lidar_burst(port, burst_s)
            row["lidar_segs"] = nseg
        else:
            notes.append("lidar:disabled")
    except OSError as e:
        notes.append(f"lidar:port_busy({e.errno})")
    except Exception as e:          # noqa: BLE001
        notes.append(f"lidar:{type(e).__name__}")
    if frames:
        with open(out / f"{stamp}_lidar.pkl", "wb") as f:
            pickle.dump({"frames": frames, "t": t}, f)
        row["lidar_ok"] = 1
        row["lidar_frames"] = len(frames)
        row["z4_pts"] = z4_point_count(frames)
    else:
        row["lidar_ok"] = 0
        if not any(n.startswith("lidar") for n in notes):
            notes.append("lidar:no_frames")
    row["note"] = ";".join(notes)
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--interval", type=float, default=300.0)
    ap.add_argument("--burst", type=float, default=1.2)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--cam", default=CAM_URL)
    ap.add_argument("--out", default="")
    ap.add_argument("--once", action="store_true")
    ap.add_argument("--no-lidar", action="store_true",
                    help="camera + light stats only; never open the LiDAR UDP port "
                         "(use while interlace_capture.py owns it)")
    args = ap.parse_args()
    if args.no_lidar:
        args.burst = 0.0

    out = Path(args.out) if args.out else (
        ROOT / "lidar_recordings" / f"lightbank_{time.strftime('%Y%m%d')}")
    out.mkdir(parents=True, exist_ok=True)
    csv_path = out / "lightbank.csv"
    new = not csv_path.exists()
    print(f"light bank -> {out}  every {args.interval:.0f}s "
          f"(burst {args.burst}s, UDP {args.port})", flush=True)

    with open(csv_path, "a", newline="") as f:
        wr = csv.DictWriter(f, fieldnames=CSV_FIELDS)
        if new:
            wr.writeheader()
            f.flush()
        while True:
            t_next = time.time() + args.interval
            row = take_sample(out, args.port, args.burst, args.cam)
            wr.writerow(row)
            f.flush()
            print(f"[{row['iso_time'][11:]}] cam={row['cam_ok']} {row['mode']:5s} "
                  f"gray={row['gray_mean']} p5/p95={row['gray_p5']}/{row['gray_p95']} "
                  f"z4edge={row['z4_edge']} sat={row['sat_frac']} | "
                  f"lidar={row['lidar_ok']} frames={row['lidar_frames']} "
                  f"z4pts={row['z4_pts']} {row['note']}", flush=True)
            if args.once:
                break
            wait = t_next - time.time()
            if wait > 0:
                time.sleep(wait)


if __name__ == "__main__":
    main()
