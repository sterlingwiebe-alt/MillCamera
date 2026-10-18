"""PLC-triggered LiDAR capture for diameter staging.

Watches a BOOL tag in the PLC; on a rising edge (operator sets it true when
the deck is staged) it grabs a short static LiDAR burst and measures the
diameter zones (3, 4, 5), printing AVG/SED/LED per zone. Optionally clears
the tag afterward as an acknowledge.

    python lidar/capture_trigger.py --tag Vision_Capture
    python lidar/capture_trigger.py --tag Vision_Capture --ack   # clear tag after

The operator then reads the first values through the optimizer; we pair
value #2 -> zone 3, #3 -> zone 4, #4 -> zone 5 (zone 2 = value #1, ignored).
"""
from __future__ import annotations

import argparse
import sys
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC

from lidar.segment import detect_logs
from lidar.stage_measure import capture_static, taper_from_arcs, _znum

DIA_ZONES = {"3_CamStaging", "4_TopOfScrew"}   # zone 5 is presence-only


def measure(frames):
    zx, zd = defaultdict(list), defaultdict(list)
    zw, zh, zc = defaultdict(list), defaultdict(list), defaultdict(list)
    for segs in frames.values():
        for lg in detect_logs(segs):
            z = lg["zone"]
            if z in DIA_ZONES:
                zx[z].extend(lg.get("arc_x", []))
                zd[z].extend(lg.get("arc_dia", []))
                zw[z].append(lg.get("width_mm", 0))
                zh[z].append(lg.get("height_mm", 0))
                zc[z].append(lg.get("dia_crown_mm", 0))
    rows = {}
    for z in zx:
        if len(zx[z]) >= 20:
            tp = taper_from_arcs(zx[z], zd[z])
            tp["width"] = int(np.median(zw[z])) if zw[z] else 0
            tp["crown"] = int(np.median(zc[z])) if zc[z] else 0
            tp["height"] = int(np.median(zh[z])) if zh[z] else 0
            rows[z] = tp
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--ack", action="store_true",
                    help="write the tag FALSE after capturing (acknowledge)")
    ap.add_argument("--duration", type=float, default=3600)
    args = ap.parse_args()

    t0 = time.time()
    last = False
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as comm:
        print(f"watching PLC tag '{args.tag}' on {args.ip}/{args.slot} "
              f"(capture {args.seconds:.0f}s on rising edge) ...", flush=True)
        while time.time() - t0 < args.duration:
            r = comm.Read(args.tag)
            if r.Status != "Success":
                print(f"read failed: {r.Status} -- check tag name", flush=True)
                time.sleep(2.0)
                continue
            v = bool(r.Value)
            if v and not last:                      # rising edge
                n += 1
                print(f"\n[{time.strftime('%H:%M:%S')}] CAPTURE #{n} "
                      f"triggered -- grabbing {args.seconds:.0f}s burst ...",
                      flush=True)
                frames = capture_static(args.seconds, args.port)
                rows = measure(frames)
                print(f"  {len(frames)} frames captured", flush=True)
                if not rows:
                    print("  (no diameter zones occupied)", flush=True)
                for z in sorted(rows, key=_znum):
                    tp = rows[z]
                    print(f"  {z:<14} circ={tp['avg']:>3} "
                          f"width={tp['width']:>3} crown={tp['crown']:>3} "
                          f"(SED={tp['sed']:>3} LED={tp['led']:>3}) "
                          f"arcs={tp['n_arcs']}", flush=True)
                if args.ack:
                    w = comm.Write(args.tag, 0)
                    print(f"  tag cleared ({w.Status})", flush=True)
                print("  --> now read me the first values through the optimizer",
                      flush=True)
            last = v
            time.sleep(0.2)
    print("done", flush=True)


if __name__ == "__main__":
    main()
