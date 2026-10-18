"""Beam-space background model for presence detection.

Learned from an EMPTY-deck capture: for every beam (layer phi x azimuth
theta bin) store the near envelope of its empty-scene range. Afterwards,
presence is pure physics: an object in a zone SHORTENS beams -- a hit
arriving >MARGIN closer than the empty envelope is foreground. Zone
membership uses the hit's own xyz, so this works regardless of mount
tilt (no vertical assumption, no plane fits).

Build:   python lidar/background.py lidar_recordings/<empty_recording>
Output:  lidar/background.npz
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

THETA_RES = np.deg2rad(0.125)      # finest interlaced azimuth pitch
N_THETA = int(round(2 * np.pi / THETA_RES))
ENVELOPE_PCT = 5                   # near envelope: 5th percentile of range
MARGIN = 0.10                      # beam must come in >10 cm closer [m]
MIN_SAMPLES = 5                    # beam needs this many empty samples


def _phi_key(phi: float) -> int:
    return int(round(phi * 1e4))


class BeamBackground:
    """Lookup: beam (phi, theta) -> empty-scene near-range envelope."""

    def __init__(self, npz_path):
        m = np.load(npz_path, allow_pickle=True)
        self.grid = m["grid"]                  # [n_phi, N_THETA] range or nan
        self.phi_keys = {int(k): i for i, k in enumerate(m["phi_keys"])}
        self.margin = float(m["margin"])

    def foreground_points(self, seg, echo: int = 0):
        """Yield xyz arrays of hits closer than the empty envelope."""
        for mod in seg.get("Modules", []):
            scale = mod.get("DistanceScalingFactor", 1.0)
            phis = mod["Phi"]
            for li, line in enumerate(mod["SegmentData"]):
                row = self.phi_keys.get(_phi_key(float(phis[li])))
                if row is None:
                    continue
                theta = np.asarray(line["ChannelTheta"], dtype=np.float64)
                d = np.asarray(line["Distance"][echo],
                               dtype=np.float64) * scale / 1000.0
                ti = (np.round(theta / THETA_RES).astype(int)) % N_THETA
                bg = self.grid[row, ti]
                ok = (d > 0.05) & ~np.isnan(bg) & (d < bg - self.margin)
                if not np.any(ok):
                    continue
                phi = float(phis[li])
                cp, sp = np.cos(phi), np.sin(phi)
                yield np.column_stack((d[ok] * cp * np.cos(theta[ok]),
                                       d[ok] * cp * np.sin(theta[ok]),
                                       d[ok] * sp))

    def zone_occupancy(self, segs, zone, echo: int = 0) -> int:
        """Count of foreground hits inside the zone band."""
        (x0, x1), (y0, y1) = zone["x"], zone["y"]
        n = 0
        for seg in segs:
            for xyz in self.foreground_points(seg, echo):
                n += int(np.sum((xyz[:, 0] >= x0) & (xyz[:, 0] <= x1) &
                                (xyz[:, 1] >= y0) & (xyz[:, 1] <= y1)))
        return n


def build(rec_dir, out_path=None, echo: int = 0):
    from lidar.pointcloud import load_segments
    samples = {}                   # phi_key -> {theta_idx: [ranges]}
    nseg = 0
    for seg in load_segments(rec_dir):
        for mod in seg.get("Modules", []):
            scale = mod.get("DistanceScalingFactor", 1.0)
            phis = mod["Phi"]
            for li, line in enumerate(mod["SegmentData"]):
                key = _phi_key(float(phis[li]))
                per = samples.setdefault(key, {})
                theta = np.asarray(line["ChannelTheta"], dtype=np.float64)
                d = np.asarray(line["Distance"][echo],
                               dtype=np.float64) * scale / 1000.0
                ti = (np.round(theta / THETA_RES).astype(int)) % N_THETA
                for t, r in zip(ti, d):
                    if r > 0.05:
                        per.setdefault(int(t), []).append(float(r))
        nseg += 1
    phi_keys = sorted(samples)
    grid = np.full((len(phi_keys), N_THETA), np.nan, dtype=np.float32)
    kept = 0
    for i, k in enumerate(phi_keys):
        for t, rs in samples[k].items():
            if len(rs) >= MIN_SAMPLES:
                grid[i, t] = np.percentile(rs, ENVELOPE_PCT)
                kept += 1
    out = Path(out_path or Path(__file__).parent / "background.npz")
    np.savez(out, grid=grid, phi_keys=np.array(phi_keys), margin=MARGIN)
    print(f"{nseg} segments, {len(phi_keys)} layers, "
          f"{kept} beams with background -> {out}")
    return out


if __name__ == "__main__":
    build(sys.argv[1])
