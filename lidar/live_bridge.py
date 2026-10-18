"""Live LiDAR -> PLC bridge (v0): measure the top-of-ScrewDeck band every
interval and write a result record to Vision_Transport continuously.

Zones 1-4: strict per-frame log detector (arcfit.detect_log -- collinear
arc centers sharing one radius) + machinery mask + 2-tick hysteresis.
Zones 5-6: presence-only (arc-count activity), diameter = -1 sentinel.

    python lidar/live_bridge.py --plc-ip 192.168.1.20 --duration 1200
"""

from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import defaultdict, deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from app.inspection import Match, RoiResult
from app.plc import RockwellPlc
from lidar.arcfit import (MachineryMask, detect_log, round_runs,
                          zone_line_points)
from lidar.background import BeamBackground
from lidar.zones import DEFAULT_ZONES
from scansegmentapi.compact import parse_payload
from scansegmentapi.udp_handler import UDPHandler

# PLC slot order 1..6 = process zone number (slot = zone)
ZONE_ORDER = ["1_OutfeedConveyor", "2_CamWaiting", "3_CamStaging",
              "4_TopOfScrew", "5_MidScrew", "6_ScrewDeck", "7_InfeedDeck"]
# zones where diameter matters; the rest are presence/activity-only
# (5 ScrewDeck is multi-piece -- pooled diameter is meaningless; 6 Infeed
# is inventory presence; per customer 2026-10-06)
DIA_ZONES = {"1_OutfeedConveyor", "2_CamWaiting", "3_CamStaging",
             "4_TopOfScrew"}
# zones where lidar/background.npz is trusted for presence. Current model
# was learned 2026-10-06 while ONLY zones 1-2 were empty (customer
# confirmed) -- extend to all six after a full empty-deck capture.
BG_ZONES = {"1_OutfeedConveyor", "2_CamWaiting"}
OCC_MIN = 15          # foreground hits (2 frames pooled) = occupied


class FrameBuffer:
    def __init__(self) -> None:
        self.lock = threading.Lock()
        self.frames = defaultdict(list)
        self.order = deque(maxlen=8)
        self.stop = False

    def run(self, port: int) -> None:
        import socket as _s
        tr = UDPHandler("0.0.0.0", port, 1 << 16)
        try:
            tr.client.setsockopt(_s.SOL_SOCKET, _s.SO_RCVBUF, 8 * 1024 * 1024)
        except Exception:
            pass
        while not self.stop:
            try:
                payload = tr.receive_new_scan_segment()
                if isinstance(payload, tuple):
                    payload = payload[0]
                seg = parse_payload(payload)
            except Exception:
                continue
            mods = seg.get("Modules", [])
            if not mods:
                continue
            fn = mods[0].get("FrameNumber", 0)
            with self.lock:
                if fn not in self.frames and len(self.order) == self.order.maxlen:
                    old = self.order[0]
                    self.frames.pop(old, None)
                if fn not in self.frames:
                    self.order.append(fn)
                self.frames[fn].append(seg)

    def latest_complete(self, n: int = 2):
        """Segments of the latest n complete frames pooled (a static log is
        only caught on ~60% of single frames; two frames ~doubles arcs)."""
        got, segs = [], []
        with self.lock:
            for fn in list(self.order)[-2::-1]:   # newest-but-one first
                f = self.frames.get(fn, [])
                if len(f) >= 11:
                    got.append(fn)
                    segs.extend(f)
                    if len(got) == n:
                        break
        return (got[0] if got else None), segs


def measure(segs, band, mask, want_dia: bool = True):
    radii, pts, centers = [], [], []
    for seg in segs:
        for line_pts in zone_line_points(seg, band):
            for run, r, c3 in round_runs(line_pts):
                radii.append(r)
                pts.append(run)
                centers.append(c3)
    if not want_dia:
        # presence-only zones (5/6): activity = arc count, no log-ness
        return {"present": len(centers) >= 6, "arcs": len(radii),
                "dia_mm": -1}
    # diameter zones: strict log detector (collinear centers + one radius)
    return detect_log(radii, pts, centers, mask=mask)


ON_TICKS = 2      # consecutive hits to turn presence on  (~1 s)
OFF_TICKS = 5     # consecutive misses to turn it off     (~2.5 s)


class ZoneState:
    """Asymmetric hysteresis on presence + diameter median over recent
    hits. Fast on, slow off: detection of a parked log is stochastic
    frame-to-frame, absence of a real log lasts."""

    def __init__(self) -> None:
        self.raw = deque(maxlen=OFF_TICKS)
        self.dias = deque(maxlen=8)
        self.present = False

    def update(self, m):
        self.raw.append(m["present"])
        if m["present"] and m["dia_mm"] > 0:
            self.dias.append(m["dia_mm"])
        r = list(self.raw)
        if len(r) >= ON_TICKS and all(r[-ON_TICKS:]):
            self.present = True
        elif len(r) == OFF_TICKS and not any(r):
            self.present = False
            self.dias.clear()
        dia = int(np.median(self.dias)) if (self.present and self.dias) \
            else -1
        return {"present": self.present, "arcs": m["arcs"], "dia_mm": dia}


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--plc-ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=1200)
    args = ap.parse_args()

    buf = FrameBuffer()
    rx = threading.Thread(target=buf.run, args=(args.port,), daemon=True)
    rx.start()

    plc = RockwellPlc(args.plc_ip, args.slot)
    plc.connect()
    mask_path = Path(__file__).parent / "machinery_mask.npz"
    mask = MachineryMask(mask_path) if mask_path.exists() else None
    bg_path = Path(__file__).parent / "background.npz"
    bg = BeamBackground(bg_path) if bg_path.exists() else None
    states = {z: ZoneState() for z in ZONE_ORDER}
    print("bridge up: sensor UDP", args.port, "-> Vision_Transport @",
          args.plc_ip, "| mask:", "loaded" if mask else "NONE",
          "| background:", f"loaded for {sorted(BG_ZONES)}" if bg else "NONE")

    rid = 100            # distinct from earlier tests
    writes = 0
    t0 = time.time()
    try:
        while time.time() - t0 < args.duration:
            tick = time.time()
            fn, segs = buf.latest_complete()
            if segs:
                rois = []
                summary = []
                for zname in ZONE_ORDER:
                    m = measure(segs, DEFAULT_ZONES[zname], mask,
                                want_dia=zname in DIA_ZONES)
                    if bg is not None and zname in BG_ZONES:
                        # presence from empty-scene background (physics:
                        # an object SHORTENS beams); arcfit only sizes it
                        occ = bg.zone_occupancy(segs, DEFAULT_ZONES[zname])
                        m["present"] = occ >= OCC_MIN
                        m["arcs"] = occ
                        if not m["present"]:
                            m["dia_mm"] = -1
                    m = states[zname].update(m)
                    rois.append(RoiResult(
                        name=zname, kind="template", passed=m["present"],
                        matches=[Match(x=max(m["dia_mm"], -1), y=m["arcs"],
                                       score=min(m["arcs"] / 50.0, 1.0))]
                        if m["present"] else []))
                    summary.append(f"{zname[0]}:{m['dia_mm'] if m['present'] else '-'}")
                rid += 1
                try:
                    plc.write_result(rid, any(r.passed for r in rois),
                                     rois, 0.0)
                    writes += 1
                except Exception as exc:
                    print("write failed, reconnecting:", exc, flush=True)
                    try:
                        plc.connect()
                    except Exception:
                        time.sleep(1.0)
                if writes % 40 == 0:
                    print(f"  {writes} writes, zones(dia mm) "
                          f"{' '.join(summary)} "
                          f"(write {plc.last_write_ms:.1f} ms)", flush=True)
            time.sleep(max(0.0, args.interval - (time.time() - tick)))
    except KeyboardInterrupt:
        pass
    finally:
        buf.stop = True
        plc.close()
    print(f"done: {writes} result writes in {time.time() - t0:.0f}s")


if __name__ == "__main__":
    main()
