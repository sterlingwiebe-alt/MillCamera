"""Measure the STATIC window from a pre-armed continuous recording.

Companion to the pre-armed staged protocol: a continuous capture.py runs in
the background before the stop, so the operator's ~6 s stop is recorded
regardless of chat latency. Afterwards this pulls the still frames (default:
the last `--last` seconds of the recording, i.e. the stop) and measures each
occupied zone's log, ORDERED BY ZONE = the order they pass the optimizer.

    python lidar/extract_static.py <recording_dir> [--last 3] [--fps 20]

Prints per-zone AVG/SED/LED + a y-stability column (sd_y) so you can confirm
the window really was static, and writes staged.json for lidar/stage_pair.py.
"""
from __future__ import annotations

import argparse
import json
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

import sys
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.pointcloud import load_segments
from lidar.segment import detect_logs
from lidar.stage_measure import taper_from_arcs, _znum


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("recording")
    ap.add_argument("--last", type=float, default=3.0,
                    help="measure the last N seconds (the stop)")
    ap.add_argument("--fps", type=float, default=20.0)
    ap.add_argument("--window", help="explicit frame range A:B instead of --last")
    args = ap.parse_args()

    frames = defaultdict(list)
    order = []
    for seg in load_segments(args.recording):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        if fn not in frames:
            order.append(fn)
        frames[fn].append(seg)

    if args.window:
        a, b = (int(v) for v in args.window.split(":"))
        sel = order[a:b]
    else:
        nlast = int(args.last * args.fps)
        sel = order[-nlast:]
    print(f"recording {Path(args.recording).name}: {len(order)} frames, "
          f"measuring {len(sel)} ({'window ' + args.window if args.window else f'last {args.last}s'})")

    zx, zd, zy = defaultdict(list), defaultdict(list), defaultdict(list)
    zf = defaultdict(set)
    for fn in sel:
        for lg in detect_logs(frames[fn]):
            z = lg["zone"]
            if z is None:
                continue
            zx[z].extend(lg.get("arc_x", []))
            zd[z].extend(lg.get("arc_dia", []))
            zy[z].append(lg["center"][1])
            zf[z].add(fn)

    occupied = sorted([z for z in zx if len(zx[z]) >= 20], key=_znum)
    rows = []
    print(f"\noptimizer order -> the Nth log through the scanner:")
    print(f"{'#':>2} {'zone':<16}{'AVG':>5}{'SED':>5}{'LED':>5}{'med':>5}"
          f"{'span':>6}{'slope':>7}{'sd_y':>6}{'arcs':>6}{'frames':>7}")
    for i, z in enumerate(occupied, 1):
        tp = taper_from_arcs(zx[z], zd[z])
        tp.update(zone=z, order=i, y_med=round(float(np.median(zy[z])), 2),
                  sd_y=round(float(np.std(zy[z])), 3), n_frames=len(zf[z]))
        rows.append(tp)
        print(f"{i:>2} {z:<16}{tp['avg']:>5}{tp['sed']:>5}{tp['led']:>5}"
              f"{tp['med']:>5}{tp['span_m']:>6}{tp['slope_mm_per_m']:>7}"
              f"{tp['sd_y']:>6}{tp['n_arcs']:>6}{tp['n_frames']:>7}")
    if not occupied:
        print("  (no zones with enough arcs)")

    ts = datetime.now().strftime("%Y%m%d_%H%M%S")
    out = Path(args.recording)
    # release time ~= end of recording (we stop the recorder just after the
    # stop window) = recording-start + nframes/fps, in wall-clock
    from datetime import timedelta
    try:
        rec_start = datetime.strptime(out.name[:15], "%Y%m%d_%H%M%S")
        t0_iso = (rec_start + timedelta(seconds=len(order) / args.fps)).isoformat()
    except ValueError:
        t0_iso = None
    with open(out / "staged.json", "w") as f:
        json.dump({"ts": ts, "t0_iso": t0_iso, "source": str(out),
                   "rows": rows}, f, indent=2)
    if t0_iso:
        print(f"est. release time (t0 for pairing): {t0_iso[11:19]}")
    print(f"\nsaved -> {out / 'staged.json'}")
    print(f"pair:  python lidar/stage_pair.py --from {out / 'staged.json'}")


if __name__ == "__main__":
    main()
