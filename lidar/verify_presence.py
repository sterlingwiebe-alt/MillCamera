"""Static-scene presence check: read a few live LiDAR frames and report
per-zone detector output (no PLC writes). Compare against camera/user
ground truth for tuning without an empty run."""
from __future__ import annotations

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.arcfit import MachineryMask
from lidar.background import BeamBackground
from lidar.live_bridge import (BG_ZONES, DIA_ZONES, OCC_MIN, ZONE_ORDER,
                               FrameBuffer, measure)
from lidar.zones import DEFAULT_ZONES


def main() -> None:
    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(2115,), daemon=True).start()
    t0 = time.time()
    while time.time() - t0 < 5.0:
        fn, segs = buf.latest_complete(2)
        if segs:
            break
        time.sleep(0.2)
    if not segs:
        print("NO LIDAR FRAMES - sensor not streaming to this PC?")
        return

    d = Path(__file__).parent
    mask = MachineryMask(d / "machinery_mask.npz") if (
        d / "machinery_mask.npz").exists() else None
    bg = BeamBackground(d / "background.npz") if (
        d / "background.npz").exists() else None

    print(f"frame {fn}, {len(segs)} segments pooled\n")
    print(f"{'zone':<18}{'present':>8}{'arcs/occ':>9}{'dia_mm':>8}  method")
    for z in ZONE_ORDER:
        m = measure(segs, DEFAULT_ZONES[z], mask, want_dia=z in DIA_ZONES)
        method = "arc-detect"
        if bg is not None and z in BG_ZONES:
            occ = bg.zone_occupancy(segs, DEFAULT_ZONES[z])
            m["present"] = occ >= OCC_MIN
            m["arcs"] = occ
            method = "background"
        print(f"{z:<18}{str(m['present']):>8}{m['arcs']:>9}"
              f"{m['dia_mm']:>8}  {method}")


if __name__ == "__main__":
    main()
