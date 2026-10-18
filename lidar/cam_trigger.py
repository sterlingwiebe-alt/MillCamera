"""PLC-triggered cam-zone height probe. On a rising edge of the trigger tag
(operator flips it at the bare-cam instant, pos ~137 before a log drops on)
it reads the cam servo position and measures the crown height + object at
the cam footprint -- so we can tell whether the ~0.47 crown is the log or
fixed structure.

    python lidar/cam_trigger.py --tag Vision_Capture --ack
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC
from lidar.arcfit import round_runs, zone_line_points
from lidar.live_bridge2 import FrameBuffer
from lidar.segment import detect_logs

CAM_TAG = "CamKicker_Servo.ActualPosition"
REGION = {"x": (-4.5, -2.5), "y": (-1.70, -1.00)}   # cam footprint


def probe(segs):
    zs = []
    for seg in segs:
        for lp in zone_line_points(seg, REGION):
            zs.extend(lp[:, 2].tolist())
    zs = np.array(zs) if zs else np.array([np.nan])
    objs = []
    for lg in detect_logs(segs):
        x, y = lg["center"][0], lg["center"][1]
        if REGION["x"][0] <= x <= REGION["x"][1] and REGION["y"][0] <= y <= REGION["y"][1]:
            objs.append(lg)
    return zs, objs


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--duration", type=float, default=3600)
    args = ap.parse_args()

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    t0 = time.time()
    last = False
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"watching '{args.tag}' -- flip at BARE CAM (pos ~137, no log) ...",
              flush=True)
        while time.time() - t0 < args.duration:
            r = plc.Read(args.tag)
            if r.Status != "Success":
                print("read failed:", r.Status, flush=True); time.sleep(2); continue
            v = bool(r.Value)
            if v and not last:
                n += 1
                cam = plc.Read(CAM_TAG)
                campos = round(float(cam.Value), 1) if cam.Status == "Success" else -1
                fn, segs = buf.latest_complete()
                if not segs:
                    time.sleep(0.3); fn, segs = buf.latest_complete()
                zs, objs = probe(segs)
                zmax = float(np.nanmax(zs))
                print(f"\n[{time.strftime('%H:%M:%S')}] PROBE #{n}  cam_pos={campos}",
                      flush=True)
                print(f"  crown_z(max)={zmax:+.2f}  z p[10,50,90]="
                      f"{np.nanpercentile(zs,10):+.2f}/{np.nanpercentile(zs,50):+.2f}/"
                      f"{np.nanpercentile(zs,90):+.2f}  npts={np.isfinite(zs).sum()}",
                      flush=True)
                if objs:
                    for o in objs:
                        print(f"  object: dia_arc={o['dia_arc_mm']} "
                              f"height={o['height_mm']} narcs={o['n_arcs']} "
                              f"y={o['center'][1]:+.2f}", flush=True)
                else:
                    print("  no detected object in cam footprint", flush=True)
                if args.ack:
                    w = plc.Write(args.tag, 0)
                    print(f"  tag cleared ({w.Status})", flush=True)
            last = v
            time.sleep(0.2)


if __name__ == "__main__":
    main()
