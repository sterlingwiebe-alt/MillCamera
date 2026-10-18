"""Always-warm snapshot trigger. Keeps the LiDAR UDP buffer AND the
camera stream running continuously, so pressing ENTER freezes and saves
the current camera+LiDAR pair instantly (no per-capture startup delay).

Run it in a terminal and leave it open. The moment the deck pauses, press
ENTER -- it saves a frozen snapshot to lidar_recordings/snapshots/<ts>/
and prints the detector read. Press q then ENTER to quit.

    python lidar/capture_watch.py
"""
from __future__ import annotations

import json
import os
import pickle
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.arcfit import MachineryMask
from lidar.background import BeamBackground
from lidar.live_bridge import DIA_ZONES, ZONE_ORDER, FrameBuffer
from lidar.segment import detect_logs, flag_skew, zone_presence
from lidar.zones import DEFAULT_ZONES
from lidar.camera_url import cam_url

CAM_URL = cam_url()                    # from the CAM_URL environment variable
COLORS = {1: (255, 60, 60), 2: (60, 220, 60), 3: (40, 220, 220),
          4: (40, 140, 255), 5: (230, 60, 230), 6: (230, 220, 40)}


class CameraReader:
    """Holds the RTSP stream open and keeps the latest frame ready."""

    def __init__(self, url):
        os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
            "rtsp_transport;tcp|stimeout;5000000")
        import cv2
        self.cv2 = cv2
        self.url = url
        self.frame = None
        self.lock = threading.Lock()
        self.stop = False
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        cap = self.cv2.VideoCapture(self.url, self.cv2.CAP_FFMPEG)
        try:
            cap.set(self.cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        while not self.stop:
            ok, f = cap.read()
            if ok and f is not None:
                with self.lock:
                    self.frame = f
            else:
                cap.release()
                time.sleep(0.5)
                cap = self.cv2.VideoCapture(self.url, self.cv2.CAP_FFMPEG)
        cap.release()

    def latest(self):
        with self.lock:
            return None if self.frame is None else self.frame.copy()


def overlay(clean, spec, cv2):
    Hc, Wc = clean.shape[:2]
    apex = np.array(spec["apex"])
    ys, xs = np.mgrid[0:Hc, 0:Wc]
    ang = np.degrees(np.arctan2(ys - apex[1], xs - apex[0]))
    tint = clean.copy()
    for band in spec["bands"]:
        if band["zone"] is None:
            continue
        sel = (ang >= band["lo"]) & (ang < band["hi"]) & (ys > apex[1])
        tint[sel] = COLORS[band["zone"]]
    over = cv2.addWeighted(tint, 0.35, clean, 0.65, 0)
    for band in spec["bands"]:
        for a in (band["lo"], band["hi"]):
            r = np.radians(a)
            end = apex + 2.8 * max(Wc, Hc) * np.array([np.cos(r), np.sin(r)])
            cv2.line(over, tuple(apex.astype(int)), tuple(end.astype(int)),
                     (255, 255, 255), 2, cv2.LINE_AA)
        mid = np.radians((band["lo"] + band["hi"]) / 2)
        pos = apex + 0.68 * Hc * np.array([np.cos(mid), np.sin(mid)])
        pos[0] = np.clip(pos[0], 40, Wc - 70)
        t = str(band["zone"]) if band["zone"] else "X"
        for col, th in [((0, 0, 0), 9), ((255, 255, 255), 4)]:
            cv2.putText(over, t, tuple(pos.astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.9, col, th, cv2.LINE_AA)
    return over


def main():
    root = Path(__file__).resolve().parent.parent
    d = Path(__file__).resolve().parent
    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(2115,), daemon=True).start()
    cam = CameraReader(CAM_URL)
    mask = MachineryMask(d / "machinery_mask.npz") if (
        d / "machinery_mask.npz").exists() else None
    bg = BeamBackground(d / "background.npz") if (
        d / "background.npz").exists() else None
    spec = None
    zp = d / "camera_zones.json"
    if zp.exists():
        spec = json.load(open(zp))
    import cv2

    print("warming up (lidar + camera)...", flush=True)
    t0 = time.time()
    while time.time() - t0 < 10:
        _, segs = buf.latest_complete(2)
        if segs and cam.latest() is not None:
            break
        time.sleep(0.2)
    if not segs or cam.latest() is None:
        print("WARN: not fully warm (lidar segs or camera missing) - "
              "captures may be incomplete")
    print("READY. Press ENTER to capture a frozen snapshot, q+ENTER to quit.",
          flush=True)

    n = 0
    while True:
        line = sys.stdin.readline()
        if line.strip().lower() == "q":
            break
        fn, segs = buf.latest_complete(2)
        frame = cam.latest()
        if not segs or frame is None:
            print("  skip: lidar/camera not ready", flush=True)
            continue
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = root / "lidar_recordings" / "snapshots" / ts
        out.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(out / "camera.png"), frame)
        with open(out / "lidar_segments.pkl", "wb") as f:
            pickle.dump(segs, f)
        if spec is not None:
            cv2.imwrite(str(out / "camera_zones.png"),
                        overlay(frame, spec, cv2))
        # corrected detector: per-log segmentation + zone offset. NOTE this
        # is single-frame (no motion tracking), so stationary machinery
        # (screw flights) can't be rejected here -- zones 5/6 may over-read;
        # zones 2/3/4 match the live bridge.
        logs = flag_skew(detect_logs(segs))
        pres = zone_presence(logs)
        zdia = {z: -1 for z in ZONE_ORDER}
        zarcs = {z: 0 for z in ZONE_ORDER}
        for lg in logs:
            z = lg["zone"]
            if z in zdia:
                zarcs[z] = max(zarcs[z], lg["n_arcs"])
                if z in DIA_ZONES and lg.get("dia_arc_mm", -1) > zdia[z]:
                    zdia[z] = lg["dia_arc_mm"]
        rows = [{"zone": z, "present": bool(pres[z]),
                 "signal": f"{zarcs[z]}arcs",
                 "dia_mm": zdia[z] if pres[z] else -1} for z in ZONE_ORDER]
        json.dump({"ts": ts, "rows": rows}, open(out / "detector.json", "w"),
                  indent=2)
        n += 1
        pres = " ".join(f"{r['zone'][0]}:{'Y' if r['present'] else '-'}"
                        for r in rows)
        print(f"  [{n}] saved {ts}  present {pres}", flush=True)
    cam.stop = True
    buf.stop = True
    print("stopped.")


if __name__ == "__main__":
    main()
