"""STAGED DIAMETER MEASUREMENT -- the stop-and-measure half of the diameter
certification protocol.

During a ~10 s stop with a clear staged scene (ideally one log per zone),
this grabs a short STATIC burst of LiDAR (many frames -> dense, un-occluded
diameter) plus one camera frame, measures each occupied zone's log, and
prints them ORDERED BY ZONE -- which is the order they will pass the mill
optimizer (lowest occupied zone exits first). So with logs at zones 2,3,4
the printout's 1st/2nd/3rd rows are the 1st/2nd/3rd logs the optimizer will
report -- a deterministic pairing, no travel-lag guessing.

Flow:
    1. user stops (~10 s)
    2. python lidar/stage_measure.py          # this: measure + save json
    3. user releases; start truth_logger.py   # optimizer side
    4. pair row N  <->  optimizer log N        # stage_pair.py

Per zone we aggregate every arc (cross-deck x, diameter) over the whole
burst and fit diameter-vs-x -> small-end SED, large-end LED, AVG, plus the
plain median the live bridge sends. detect_logs already drops the screw
(deviant axis) and oversized machinery, so a staged log dominates its zone.
"""
from __future__ import annotations

import argparse
import json
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.live_bridge2 import FrameBuffer
from lidar.segment import detect_logs
from lidar.zones import DEFAULT_ZONES

ZONE_ORDER = list(DEFAULT_ZONES)          # 6..1 as declared
# optimizer order = ascending process-zone number (outfeed = 1 exits first)
def _znum(z):
    return int(z.split("_")[0])


def taper_from_arcs(xs, ds):
    """Aggregated per-arc (x, dia) -> SED/LED/AVG/median (mm)."""
    xs = np.asarray(xs, float)
    ds = np.asarray(ds, float)
    med = float(np.median(ds))
    lo, hi = np.percentile(xs, 5), np.percentile(xs, 95)
    edges = np.linspace(lo, hi, 9)
    bx, bd = [], []
    for i in range(len(edges) - 1):
        m = (xs >= edges[i]) & (xs < edges[i + 1])
        if m.sum() >= 3:
            bx.append((edges[i] + edges[i + 1]) / 2)
            bd.append(float(np.median(ds[m])))
    if len(bx) >= 3:
        b, a = np.polyfit(bx, bd, 1)
        d_lo, d_hi = a + b * lo, a + b * hi
        sed, led, avg = min(d_lo, d_hi), max(d_lo, d_hi), a + b * (lo + hi) / 2
        slope = b
    else:
        sed = led = avg = med
        slope = 0.0
    return {"avg": round(avg), "sed": round(sed), "led": round(led),
            "med": round(med), "span_m": round(float(hi - lo), 2),
            "slope_mm_per_m": round(float(slope), 1), "n_arcs": int(len(xs))}


def grab_camera():
    import os
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
        "rtsp_transport;tcp|stimeout;5000000")
    import cv2
    from lidar.camera_url import cam_url
    cap = cv2.VideoCapture(cam_url(), cv2.CAP_FFMPEG)   # CAM_URL env var
    frame = None
    for _ in range(15):
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
    cap.release()
    return frame


def capture_static(seconds, port):
    """Collect complete frames for `seconds`; return {fn: segs}."""
    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(port,), daemon=True).start()
    t0 = time.time()
    frames = {}
    while time.time() - t0 < seconds:
        fn, segs = buf.latest_complete()
        if segs and fn not in frames:
            frames[fn] = segs
        time.sleep(0.02)
    buf.stop = True
    return frames


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=6.0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--no-camera", action="store_true")
    args = ap.parse_args()

    print(f"capturing {args.seconds:.0f}s static burst on UDP {args.port} ...",
          flush=True)
    frames = capture_static(args.seconds, args.port)
    if not frames:
        print("NO LIDAR FRAMES -- is the scanner streaming / port free?")
        return

    zone_x = defaultdict(list)
    zone_d = defaultdict(list)
    zone_y = defaultdict(list)
    zone_frames = defaultdict(set)
    for fn, segs in frames.items():
        for lg in detect_logs(segs):
            z = lg["zone"]
            if z is None:
                continue
            zone_x[z].extend(lg.get("arc_x", []))
            zone_d[z].extend(lg.get("arc_dia", []))
            zone_y[z].append(lg["center"][1])
            zone_frames[z].add(fn)

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(__file__).resolve().parent.parent / "lidar_recordings" / \
        "snapshots" / f"{ts}_staged"
    out.mkdir(parents=True, exist_ok=True)

    frame = None if args.no_camera else grab_camera()
    if frame is not None:
        import cv2
        cv2.imwrite(str(out / "camera.png"), frame)

    occupied = sorted([z for z in zone_x if len(zone_x[z]) >= 20], key=_znum)
    rows = []
    print(f"\nsnapshot {ts}  ({len(frames)} frames, "
          f"camera={'ok' if frame is not None else 'none'})\n")
    print("optimizer order -> the Nth log through the scanner:")
    print(f"{'#':>2} {'zone':<16}{'AVG':>5}{'SED':>5}{'LED':>5}{'med':>5}"
          f"{'span':>6}{'slope':>7}{'arcs':>6}{'frames':>7}")
    for i, z in enumerate(occupied, 1):
        tp = taper_from_arcs(zone_x[z], zone_d[z])
        tp["zone"] = z
        tp["order"] = i
        tp["y_med"] = round(float(np.median(zone_y[z])), 2)
        tp["n_frames"] = len(zone_frames[z])
        rows.append(tp)
        print(f"{i:>2} {z:<16}{tp['avg']:>5}{tp['sed']:>5}{tp['led']:>5}"
              f"{tp['med']:>5}{tp['span_m']:>6}{tp['slope_mm_per_m']:>7}"
              f"{tp['n_arcs']:>6}{tp['n_frames']:>7}")
    if not occupied:
        print("  (no zones with enough arcs -- empty deck?)")

    meta = {"ts": ts, "n_frames": len(frames), "rows": rows}
    with open(out / "staged.json", "w") as f:
        json.dump(meta, f, indent=2)
    print(f"\nsaved -> {out}")
    print("NOW: release the logs and start the optimizer capture:")
    print("  python scripts/truth_logger.py --duration 120")
    print(f"then pair:  python lidar/stage_pair.py {ts}_staged")


if __name__ == "__main__":
    main()
