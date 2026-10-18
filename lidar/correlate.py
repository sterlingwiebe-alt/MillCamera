"""Correlate our LiDAR per-log diameter against the mill optimizer truth.

Pipeline:
  1. measure_band_series: per-frame diameter in the top-of-screw reference
     band over the flowing pilot recording.
  2. detect_transits: group occupied frames into individual log passes,
     one diameter per transit.
  3. load_truth: optimizer SED/LED/length, de-duplicated to one row/log.
  4. lag_correlate: find the travel-time lag that best aligns our transit
     sequence to the truth sequence, then report matched-pair diameter
     error -- instrument vs instrument.

The optimizer sits DOWNSTREAM, so a log we measure at time t reaches it
at t+lag (lag > 0).
"""
from __future__ import annotations

import csv
import json
from datetime import datetime, timedelta
from pathlib import Path

import numpy as np

from .arcfit import round_runs, zone_line_points
from .pointcloud import load_segments

# pilot-era top-of-screw reference band (sensor pos of 2026-10-06 pilot)
REF_BAND = {"x": (-6.3, -1.3), "y": (-1.05, -0.42)}
FPS = 20.0
MIN_ARCS = 6
MIN_TRANSIT_FRAMES = 4      # >=0.2 s
GAP_FRAMES = 3              # bridge short dropouts within a transit
IN_TO_MM = 25.4


def _frame_diameter(arcs_pts, arcs_centers):
    if len(arcs_centers) < MIN_ARCS:
        return None
    C = np.vstack(arcs_centers)
    P = np.vstack(arcs_pts)
    keep = np.ones(len(C), bool)
    c = axis = None
    for _ in range(3):
        c = C[keep].mean(0)
        q = C[keep] - c
        axis = np.linalg.svd(q, full_matrices=False)[2][0]
        dall = np.linalg.norm((C - c) - np.outer((C - c) @ axis, axis), axis=1)
        keep = dall < max(3 * np.median(dall[keep]), 0.05)
    radial = (P - c) - np.outer((P - c) @ axis, axis)
    return 2 * float(np.median(np.linalg.norm(radial, axis=1)))


def measure_band_series(recording, band=REF_BAND):
    """Per frame: (frame_idx, n_arcs, diameter_mm or None)."""
    frames = {}
    order = []
    for seg in load_segments(recording):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        if fn not in frames:
            frames[fn] = {"pts": [], "c": []}
            order.append(fn)
        f = frames[fn]
        for lp in zone_line_points(seg, band):
            for run, r, c3 in round_runs(lp):
                f["pts"].append(run)
                f["c"].append(c3)
    series = []
    for i, fn in enumerate(order):
        f = frames[fn]
        dia = _frame_diameter(f["pts"], f["c"]) if f["c"] else None
        series.append((i, len(f["c"]), dia))
    return series


def detect_transits(series, t0):
    """Group occupied frames into transits -> (t_mid, diameter_mm, n)."""
    transits = []
    run = []
    gap = 0
    for i, n, dia in series:
        if n >= MIN_ARCS and dia:
            run.append((i, dia))
            gap = 0
        else:
            gap += 1
            if run and gap > GAP_FRAMES:
                if len(run) >= MIN_TRANSIT_FRAMES:
                    transits.append(_finish(run, t0))
                run = []
    if len(run) >= MIN_TRANSIT_FRAMES:
        transits.append(_finish(run, t0))
    return transits


def _finish(run, t0):
    idxs = [i for i, _ in run]
    dias = [d for _, d in run]
    t_mid = t0 + timedelta(seconds=(idxs[0] + idxs[-1]) / 2 / FPS)
    return {"t": t_mid, "dia_mm": round(float(np.median(dias)) * 1000),
            "n_frames": len(run)}


def load_truth(csv_path, t_lo, t_hi):
    """De-duplicated optimizer logs in window -> (t, sed_mm, led_mm, len_m)."""
    rows = []
    last = None
    with open(csv_path) as f:
        for row in csv.DictReader(f):
            key = (row["data3"], row["data4"], row["data6"])
            if key == last:
                continue
            last = key
            t = datetime.fromisoformat(row["iso_time"])
            if not (t_lo <= t <= t_hi):
                continue
            sed = int(row["data3"]) / 100 * IN_TO_MM
            led = int(row["data4"]) / 100 * IN_TO_MM
            length = int(row["data6"]) * IN_TO_MM / 1000
            rows.append({"t": t, "sed_mm": round(sed), "led_mm": round(led),
                         "len_m": round(length, 2)})
    return rows


def lag_correlate(transits, truth, lag_lo=0, lag_hi=180, step=1):
    """Find travel lag (s) best aligning transits->truth; report error vs
    SED / LED / mid. Returns best result dict."""
    best = None
    for lag in range(lag_lo, lag_hi + 1, step):
        pairs = []
        for tr in transits:
            tgt = tr["t"] + timedelta(seconds=lag)
            j = min(truth, key=lambda r: abs((r["t"] - tgt).total_seconds()))
            dt = abs((j["t"] - tgt).total_seconds())
            if dt <= 2.0:
                mid = (j["sed_mm"] + j["led_mm"]) / 2
                pairs.append((tr["dia_mm"], j["sed_mm"], j["led_mm"], mid))
        if len(pairs) < 5:
            continue
        arr = np.array(pairs)
        err_mid = float(np.median(np.abs(arr[:, 0] - arr[:, 3])))
        cand = {"lag_s": lag, "n_pairs": len(pairs),
                "err_vs_sed": float(np.median(np.abs(arr[:, 0] - arr[:, 1]))),
                "err_vs_led": float(np.median(np.abs(arr[:, 0] - arr[:, 2]))),
                "err_vs_mid": err_mid,
                "corr_mid": float(np.corrcoef(arr[:, 0], arr[:, 3])[0, 1])
                if len(pairs) > 2 else 0.0}
        if best is None or cand["err_vs_mid"] < best["err_vs_mid"]:
            best = cand
    return best


def run(recording, truth_csv):
    meta = json.load(open(Path(recording) / "meta.json"))
    # start time from meta label timestamp or dir name
    name = Path(recording).name          # 20261006_124051_...
    t0 = datetime.strptime(name[:15], "%Y%m%d_%H%M%S")
    series = measure_band_series(recording)
    occ = sum(1 for _, n, d in series if n >= MIN_ARCS and d)
    transits = detect_transits(series, t0)
    t_lo = t0 - timedelta(seconds=10)
    t_hi = t0 + timedelta(seconds=len(series) / FPS + 200)
    truth = load_truth(truth_csv, t_lo, t_hi)
    print(f"frames {len(series)} ({occ} occupied), "
          f"transits {len(transits)}, truth logs in window {len(truth)}")
    print("our transit diameters (mm):",
          [tr["dia_mm"] for tr in transits])
    print("truth SED (mm):", [r["sed_mm"] for r in truth])
    best = lag_correlate(transits, truth)
    print("\nBEST ALIGNMENT:", best)
    return series, transits, truth, best


if __name__ == "__main__":
    import sys
    run(sys.argv[1], sys.argv[2])
