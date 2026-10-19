"""Evaluate zone-2 presence rule variants offline over burst captures.
Per frame: qualifying arcs = log radius, centre in band, points-median in band
(the current rule). Then simulate the 5-frame history for several rules and
print the fraction of frames called present per capture."""
import glob, pickle, sys
from pathlib import Path
from collections import deque
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.flow_track import FlowTracker, PRES_R_LO, PRES_R_HI   # noqa: E402
from lidar.arcfit import zone_round_runs                            # noqa: E402

zone = "2_CamWaiting"
win, wide = {z: (w, wd) for z, w, wd, _ in FlowTracker._presence_bands()}[zone]
y0, y1 = win["y"]

def frame_arcs(segs):
    out = []
    for run, r, c3 in zone_round_runs(segs, wide):
        if not (PRES_R_LO <= r <= PRES_R_HI and y0 <= c3[1] <= y1):
            continue
        if not (y0 <= float(np.median(run[:, 1])) <= y1):
            continue
        out.append((float(c3[1]), r * 2000, len(run)))
    return out

RULES = {
    "current: >=1 arc in 3 of 5":      lambda h: sum(1 for a in h if len(a) >= 1) >= 3,
    ">=2 arcs in 3 of 5":              lambda h: sum(1 for a in h if len(a) >= 2) >= 3,
    ">=1 arc, n>=7 pts, 3 of 5":       lambda h: sum(1 for a in h if any(n >= 7 for _, _, n in a)) >= 3,
    ">=1 arc 3 of 5, y spread<=0.10":  lambda h: (lambda ys: len(ys) >= 3 and max(ys) - min(ys) <= 0.10)(
                                            [np.median([y for y, _, _ in a]) for a in h if a]),
    ">=1 arc 3 of 5, y spread<=0.06":  lambda h: (lambda ys: len(ys) >= 3 and max(ys) - min(ys) <= 0.06)(
                                            [np.median([y for y, _, _ in a]) for a in h if a]),
}
pkls = sys.argv[1:] or sorted(glob.glob(str(Path(__file__).resolve().parent / "z4captures" / "interlaced" / "cap_*.pkl")))
print(f"{'capture':28s} {'cam':>6s} {'fr':>3s} {'arcs/fr':>8s}  " + "  ".join(f"{k[:30]:>30s}" for k in RULES))
for pkl in pkls:
    d = pickle.load(open(pkl, "rb"))
    frames = [segs for _, segs in sorted(d["frames"].items())]
    per = [frame_arcs(s) for s in frames]
    hist = deque(maxlen=5); calls = {k: 0 for k in RULES}
    for a in per:
        hist.append(a)
        for k, f in RULES.items():
            calls[k] += bool(f(list(hist)))
    n = len(per)
    arcs_mean = np.mean([len(a) for a in per])
    ys = sorted(round(y, 2) for a in per for y, _, _ in a)
    print(f"{pkl.split(chr(92))[-1]:28s} {d.get('cam'):6} {n:3d} {arcs_mean:8.1f}  "
          + "  ".join(f"{calls[k]/n:30.0%}" for k in RULES)
          + f"   arc y: {ys[:3]}..{ys[-3:]}" if ys else "")
