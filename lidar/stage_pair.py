"""Pair a staged snapshot (lidar/stage_measure.py) against the optimizer.

The staged rows are ordered by zone = the order the logs pass the optimizer.
After release, the first N DISTINCT optimizer logs (N = number of staged
rows) are those same logs in that order. Pair row i <-> optimizer log i and
report AVG/SED/LED error -- an instrument-vs-instrument number on KNOWN,
deterministically matched logs.

    python lidar/stage_pair.py <ts>_staged [truth_csv]

Shows both sequences so any boundary offset (a stray log already past the
lowest staged zone at stop time) is visible -- shift with --skip N if so.
"""
from __future__ import annotations

import argparse
import csv
import json
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
SNAP = ROOT / "lidar_recordings" / "snapshots"
IN_TO_MM = 25.4


def load_truth_after(csv_path, t0, n_want, horizon_s=180):
    """First `n_want` DISTINCT optimizer logs at/after t0."""
    rows = []
    last = None
    t_hi = t0 + timedelta(seconds=horizon_s)
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            key = (row["data3"], row["data4"], row["data6"])
            if key == last:
                continue
            last = key
            t = datetime.fromisoformat(row["iso_time"])
            if t < t0 or t > t_hi:
                continue
            rows.append({
                "t": t,
                "sed_mm": round(int(row["data3"]) / 100 * IN_TO_MM),
                "led_mm": round(int(row["data4"]) / 100 * IN_TO_MM),
                "len_m": round(int(row["data6"]) * IN_TO_MM / 1000, 2)})
    return rows[:n_want]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("staged", nargs="?",
                    help="<ts>_staged dir under snapshots/")
    ap.add_argument("--from", dest="from_json",
                    help="explicit path to a staged.json (e.g. from extract_static)")
    ap.add_argument("truth_csv", nargs="?",
                    default=str(ROOT / "lidar_recordings" / "truth_log.csv"))
    ap.add_argument("--skip", type=int, default=0,
                    help="drop this many leading optimizer logs (boundary)")
    args = ap.parse_args()

    jpath = Path(args.from_json) if args.from_json else \
        SNAP / args.staged / "staged.json"
    meta = json.load(open(jpath))
    rows = meta["rows"]
    # t0 = release time: prefer the extractor's estimate, else the snapshot ts
    t0 = (datetime.fromisoformat(meta["t0_iso"]) if meta.get("t0_iso")
          else datetime.strptime(meta["ts"], "%Y%m%d_%H%M%S"))
    truth = load_truth_after(args.truth_csv, t0, args.skip + len(rows))
    truth = truth[args.skip:]

    print(f"staged {meta['ts']}: {len(rows)} logs; "
          f"optimizer logs matched: {len(truth)}\n")
    print(f"{'#':>2} {'zone':<14}"
          f"{'myAVG':>6}{'trMID':>6}{'dAVG':>6} | "
          f"{'mySED':>6}{'trSED':>6}{'dSED':>6} | "
          f"{'myLED':>6}{'trLED':>6}{'dLED':>6}   {'opt.len':>7}")
    pa, ps, pl = [], [], []
    for i, r in enumerate(rows):
        if i >= len(truth):
            print(f"{r['order']:>2} {r['zone']:<14}{r['avg']:>6}"
                  "   (no optimizer log)")
            continue
        t = truth[i]
        mid = (t["sed_mm"] + t["led_mm"]) / 2
        da, ds, dl = r["avg"] - mid, r["sed"] - t["sed_mm"], r["led"] - t["led_mm"]
        pa.append(da); ps.append(ds); pl.append(dl)
        print(f"{r['order']:>2} {r['zone']:<14}"
              f"{r['avg']:>6}{mid:>6.0f}{da:>+6.0f} | "
              f"{r['sed']:>6}{t['sed_mm']:>6}{ds:>+6.0f} | "
              f"{r['led']:>6}{t['led_mm']:>6}{dl:>+6.0f}   {t['len_m']:>6}m")

    if pa:
        def stat(p):
            p = np.array(p, float)
            return (f"bias {p.mean():+.1f}  MAE {np.abs(p).mean():.1f}  "
                    f"RMSE {np.sqrt((p**2).mean()):.1f}  (n={len(p)})")
        print("\nAVG vs truth-mid :", stat(pa), f" -> calib +{-np.mean(pa):.1f}")
        print("SED vs truth-SED :", stat(ps))
        print("LED vs truth-LED :", stat(pl))


if __name__ == "__main__":
    main()
