"""Correlated log of cam servo position + LiDAR crown height in the cam
region, to build the presence height-model (bare cam vs log-on-cam vs
log-in-trough vs empty, as a function of CamKicker_Servo.ActualPosition).

    python lidar/cam_height_log.py --seconds 120
"""
from __future__ import annotations

import argparse
import csv
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC
from lidar.arcfit import round_runs, zone_line_points
from lidar.live_bridge2 import FrameBuffer

CAM_TAG = "CamKicker_Servo.ActualPosition"
# cam / zone-3 region (raw sensor frame); crown = highest point here
REGION = {"x": (-6.3, -1.3), "y": (-1.70, -0.90)}


def crown(segs):
    best_z, bx, by, narc = -9.0, 0.0, 0.0, 0
    for seg in segs:
        for lp in zone_line_points(seg, REGION):
            zi = int(lp[:, 2].argmax())
            if lp[zi, 2] > best_z:
                best_z, bx, by = float(lp[zi, 2]), float(lp[zi, 0]), float(lp[zi, 1])
            narc += sum(1 for _ in round_runs(lp))
    return best_z, bx, by, narc


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=120)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--out", default="lidar_recordings/cam_height_log.csv")
    args = ap.parse_args()

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    f = open(args.out, "w", newline="")
    wr = csv.writer(f)
    wr.writerow(["t", "cam_pos", "crown_z", "crown_x", "crown_y", "narcs"])
    t0 = time.time()
    last_fn = None
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"logging cam_pos + crown height for {args.seconds:.0f}s ...",
              flush=True)
        while time.time() - t0 < args.seconds:
            fn, segs = buf.latest_complete()
            if segs and fn != last_fn:
                last_fn = fn
                r = plc.Read(CAM_TAG)
                cam = float(r.Value) if r.Status == "Success" else -1.0
                z, x, y, narc = crown(segs)
                wr.writerow([round(time.time() - t0, 2), round(cam, 1),
                             round(z, 3), round(x, 2), round(y, 2), narc])
                n += 1
                if n % 40 == 0:
                    f.flush()
                    print(f"  {n} rows  cam={cam:.0f} crown_z={z:.2f} "
                          f"@({x:.1f},{y:.1f}) narcs={narc}", flush=True)
            time.sleep(0.03)
    buf.stop = True
    f.close()
    print(f"done: {n} rows -> {args.out}", flush=True)


if __name__ == "__main__":
    main()
