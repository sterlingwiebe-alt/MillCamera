"""Zone-4 fixed-background model for diameter work.

Builds, from an EMPTY-running recording, (1) the bare-deck plane and (2) a
per-(x,y)-cell structure mask (cells where the empty machinery rises above
the deck -- rail, flat member, far-side band). Loaded captures then measure
the log as height-above-deck in cells the empty background leaves clean.

Build once:
    python lidar/z4_bg.py build lidar_recordings/20261007_144627_empty_running

Then load z4captures/z4_bg.npz in the measurement code.
"""
from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.pointcloud import load_segments
from lidar.arcfit import zone_line_points
from lidar.probe_zones import PRESENCE_WINDOWS, DIA_ZONE

# wide capture window; the far-side band is excluded by the structure mask
WIN = dict(PRESENCE_WINDOWS[DIA_ZONE])
WIN["y"] = (-1.05, -0.10)
WIN["x"] = (-4.3, -2.2)
BIN = 0.05
OUT = Path(__file__).resolve().parent / "z4captures" / "z4_bg.npz"

_rng = np.random.default_rng(0)


def window_points(src):
    P = []
    segiter = (load_segments(src) if isinstance(src, (str, Path))
               else (s for fr in src.values() for s in fr))
    for seg in segiter:
        for lp in zone_line_points(seg, WIN):
            P.append(lp)
    return np.vstack(P) if P else np.zeros((0, 3))


def _plane_eig(P):
    c = P.mean(0)
    w, v = np.linalg.eigh(np.cov((P - c).T))
    nrm = v[:, 0]
    return nrm, -nrm @ c


def fit_deck_plane(P, tol=0.04, it=600):
    """RANSAC plane; keep the LOW (deck) surface, not tilted structure."""
    best = None
    n = len(P)
    for _ in range(it):
        i = _rng.choice(n, 3, replace=False)
        p = P[i]
        nrm = np.cross(p[1] - p[0], p[2] - p[0])
        nn = np.linalg.norm(nrm)
        if nn < 1e-6:
            continue
        nrm = nrm / nn
        d = -nrm @ p[0]
        # prefer near-horizontal planes (deck); penalize tilt
        if abs(nrm[2]) < 0.6:
            continue
        inl = (np.abs(P @ nrm + d) < tol).sum()
        if best is None or inl > best[0]:
            best = (inl, nrm, d)
    if best is None:                      # fallback: no horizontal plane found
        nrm, d = _plane_eig(P)
    else:
        _, nrm, d = best
    for _ in range(3):
        m = np.abs(P @ nrm + d) < tol
        nrm, d = _plane_eig(P[m])
    if nrm[2] < 0:
        nrm, d = -nrm, -d
    return nrm, d


def cell_struct_height(P, nrm, d, q=95):
    """per-cell height-above-plane (q-th percentile) of the empty background."""
    r = P @ nrm + d
    g = defaultdict(list)
    for (x, y), rr in zip(P[:, :2], r):
        g[(round(x / BIN), round(y / BIN))].append(rr)
    return {k: float(np.percentile(v, q)) for k, v in g.items() if len(v) >= 3}


def band_heights(src, nrm, d):
    """per-scan-FRAME max height-above-deck for each band (module,layer),
    inside the window. Returns {(mi,li): [per-frame max, ...]}."""
    framelist = ([list(v) if not isinstance(v, list) else v
                  for v in src.values()] if isinstance(src, dict)
                 else [[seg] for seg in load_segments(src)])
    perband = defaultdict(list)
    for segs in framelist:
        fb = {}
        for s in segs:
            for mi, mod in enumerate(s.get("Modules", [])):
                scale = mod.get("DistanceScalingFactor", 1.0)
                phis = mod["Phi"]
                for li, line in enumerate(mod["SegmentData"]):
                    th = np.asarray(line["ChannelTheta"])
                    dist = np.asarray(line["Distance"][0]) * scale / 1000.
                    ok = dist > 0.05
                    if not ok.any():
                        continue
                    phi = float(phis[li])
                    cp, sp = np.cos(phi), np.sin(phi)
                    x = dist[ok] * cp * np.cos(th[ok])
                    y = dist[ok] * cp * np.sin(th[ok])
                    z = dist[ok] * sp
                    inw = ((x >= WIN["x"][0]) & (x <= WIN["x"][1]) &
                           (y >= WIN["y"][0]) & (y <= WIN["y"][1]))
                    if not inw.any():
                        continue
                    h = (x[inw] * nrm[0] + y[inw] * nrm[1] +
                         z[inw] * nrm[2] + d)
                    g = (mi, li)
                    hm = float(h.max())
                    if g not in fb or hm > fb[g]:
                        fb[g] = hm
        for g, hm in fb.items():
            perband[g].append(hm)
    return perband


def build(rec):
    E = window_points(rec)
    nrm, d = fit_deck_plane(E)
    struct = cell_struct_height(E, nrm, d)
    keys = np.array(list(struct.keys()), dtype=np.int32)
    vals = np.array([struct[tuple(k)] for k in keys], dtype=np.float32)
    # per-band empty background (p90 of per-frame max heights)
    eb = band_heights(rec, nrm, d)
    bkeys = np.array(list(eb.keys()), dtype=np.int32)
    bp90 = np.array([np.percentile(eb[tuple(k)], 90) for k in bkeys],
                    dtype=np.float32)
    OUT.parent.mkdir(parents=True, exist_ok=True)
    np.savez(OUT, normal=nrm, offset=d, bin=BIN,
             win_x=WIN["x"], win_y=WIN["y"],
             struct_keys=keys, struct_vals=vals,
             band_keys=bkeys, band_p90=bp90)
    print(f"deck plane normal={np.round(nrm,3)} offset={d:.3f}")
    print(f"structure cells={len(keys)}  bands={len(bkeys)}")
    print("per-band empty bg (p90 mm): " +
          ", ".join(f"{tuple(int(x) for x in k)}={v*1000:.0f}"
                    for k, v in zip(bkeys, bp90)))
    print(f"saved {OUT}")


if __name__ == "__main__":
    if len(sys.argv) >= 3 and sys.argv[1] == "build":
        build(sys.argv[2])
    else:
        print(__doc__)
