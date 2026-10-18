"""Learn a machinery mask from a recording: arc centers that sit in the
same place across most frames are machine parts (screw flights, curved
steel), not logs -- logs translate or come and go.

Output: lidar/machinery_mask.npz (grid + params), consumed by
live_bridge/measure. Rebuild whenever the sensor is remounted.

    python lidar/build_mask.py lidar_recordings/20261006_124051_correlation_pilot
"""

from __future__ import annotations

import sys
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.arcfit import round_runs, zone_line_points
from lidar.pointcloud import load_segments
from lidar.zones import DEFAULT_ZONES

CELL = 0.05           # m, dwell grid pitch
DWELL_FRAC = 0.45     # center in cell in > this fraction of frames = machine
BAND_X = (-6.3, -1.3)
BAND_Y = (-2.72, 3.17)   # all six zones


def main() -> None:
    rec = Path(sys.argv[1] if len(sys.argv) > 1 else
               "lidar_recordings/20261006_124051_correlation_pilot")
    band = {"x": BAND_X, "y": BAND_Y}
    nx = int((BAND_X[1] - BAND_X[0]) / CELL) + 1
    ny = int((BAND_Y[1] - BAND_Y[0]) / CELL) + 1

    # per-cell set of frames in which an arc center landed there
    cell_frames = defaultdict(set)
    frames_seen = set()
    for seg in load_segments(rec):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        frames_seen.add(fn)
        for pts in zone_line_points(seg, band):
            for _run, _r, c3 in round_runs(pts):
                ix = int((c3[0] - BAND_X[0]) / CELL)
                iy = int((c3[1] - BAND_Y[0]) / CELL)
                if 0 <= ix < nx and 0 <= iy < ny:
                    cell_frames[(ix, iy)].add(fn)

    nframes = len(frames_seen)
    frac = np.zeros((ny, nx), dtype=np.float32)
    for (ix, iy), fns in cell_frames.items():
        frac[iy, ix] = len(fns) / nframes
    mask = frac > DWELL_FRAC
    # dilate by one cell: centers jitter a little around machine parts
    d = mask.copy()
    d[1:, :] |= mask[:-1, :]; d[:-1, :] |= mask[1:, :]
    d[:, 1:] |= mask[:, :-1]; d[:, :-1] |= mask[:, 1:]

    out = Path(__file__).parent / "machinery_mask.npz"
    np.savez(out, mask=d, x0=BAND_X[0], y0=BAND_Y[0], cell=CELL,
             dwell_frac=DWELL_FRAC, nframes=nframes, src=str(rec))
    print(f"frames: {nframes}, machine cells: {int(d.sum())} "
          f"(raw {int(mask.sum())}) of {nx * ny}")
    print("saved", out)

    # per-zone report: share of arc observations that the mask removes
    for zname, zone in sorted(DEFAULT_ZONES.items()):
        y0, y1 = zone["y"]
        tot = kill = 0
        for (ix, iy), fns in cell_frames.items():
            cy = BAND_Y[0] + (iy + 0.5) * CELL
            if y0 <= cy <= y1:
                tot += len(fns)
                if d[iy, ix]:
                    kill += len(fns)
        pct = 100.0 * kill / tot if tot else 0.0
        print(f"  {zname:<18} arc-obs {tot:>6}  masked {pct:5.1f}%")


if __name__ == "__main__":
    main()
