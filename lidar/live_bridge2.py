"""Live LiDAR -> PLC bridge v2: segmentation + motion tracking.

Supersedes the arc-detect + beam-background bridge (jitter-broken, cam
false-positives). Here:
  - every frame is segmented into individual logs (lidar.segment);
  - logs are tracked across frames (lidar.track); MOTION rejects the cams
    and screw flights (stationary) without any background model, so it is
    jitter-tolerant;
  - a zone is PRESENT when a MOVING log's center is in it;
  - diameter is dia_arc_mm (arc-radius estimate, validated ~13 mm vs the
    mill optimizer, essentially unbiased).

PLC map (Vision_Transport DINT[40]) layout is unchanged from v1 so the
ladder is undisturbed; only the VALUES improve (tracking presence +
arc-radius diameter).

    python lidar/live_bridge2.py --plc-ip 192.168.1.20 --duration 1800
    python lidar/live_bridge2.py --replay <recording>     # offline, no PLC
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import defaultdict, deque
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from app.inspection import Match, RoiResult
from app.plc import RockwellPlc
from lidar.pointcloud import load_segments
from lidar.segment import detect_logs
from lidar.track import LogTracker
from lidar.zones import DEFAULT_ZONES

# slot 1..6 = process zone (slot = zone)
ZONE_ORDER = ["1_OutfeedConveyor", "2_CamWaiting", "3_CamStaging",
              "4_TopOfScrew", "5_MidScrew", "6_ScrewDeck", "7_InfeedDeck"]
# zones where diameter is reported. Measurement zones are 3, 4, 5 (5 =
# MidScrew, the cleaner upstream vantage). 6/7 presence-only (far-field,
# multi-piece); 1/2 dropped (zone 2 muddy with zone 1).
DIA_ZONES = {"3_CamStaging", "4_TopOfScrew"}   # zone 5 presence-only


def zone_state(tracker):
    """Per-zone (present, diameter_mm). Machinery rejection is done upstream
    by SHAPE in detect_logs (the screw/auger by deviant axis, oversized
    parts by diameter), NOT by motion -- so this reports every tracked log,
    moving or still. The only machine that still passes is the cam (a round
    lobe); that is handled by the PLC cam-position gate. The tracker is used
    purely for smoothing (steady diameter + jitter-proof zone)."""
    present = {z: False for z in ZONE_ORDER}
    dia = {z: -1 for z in ZONE_ORDER}
    arcs = {z: 0 for z in ZONE_ORDER}
    for t in tracker.active:
        if t.n < tracker.min_len:
            continue
        z = t.zone()
        if z not in present:
            continue
        present[z] = True
        arcs[z] = max(arcs[z], t.last.get("n_arcs", 0))
        if z in DIA_ZONES:
            d = t.diameter()                 # arc-radius estimate
            if d > dia[z]:
                dia[z] = d
    return present, dia, arcs


def build_rois(present, dia, arcs):
    rois = []
    for z in ZONE_ORDER:
        p = present[z]
        rois.append(RoiResult(
            name=z, kind="template", passed=p,
            matches=[Match(x=max(dia[z], -1), y=arcs[z],
                           score=min(arcs[z] / 50.0, 1.0))] if p else []))
    return rois


class FrameBuffer:
    def __init__(self):
        self.lock = threading.Lock()
        self.frames = defaultdict(list)
        self.order = deque(maxlen=8)
        self.stop = False

    def run(self, port):
        import socket as _s
        from scansegmentapi.compact import parse_payload
        from scansegmentapi.udp_handler import UDPHandler
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
                    self.frames.pop(self.order[0], None)
                if fn not in self.frames:
                    self.order.append(fn)
                self.frames[fn].append(seg)

    def latest_complete(self):
        with self.lock:
            for fn in list(self.order)[-2::-1]:
                segs = self.frames.get(fn, [])
                if len(segs) >= 11:
                    return fn, list(segs)
        return None, []


def replay(recording):
    """Offline: run the full pipeline over a recording, print per-zone
    state periodically. No PLC."""
    frames = defaultdict(list)
    order = []
    for seg in load_segments(recording):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        if fn not in frames:
            order.append(fn)
        frames[fn].append(seg)
    tk = LogTracker()
    print(f"replay {Path(recording).name}: {len(order)} frames")
    for i, fn in enumerate(order):
        tk.update(detect_logs(frames[fn]))
        if i % 100 == 0 or i == len(order) - 1:
            present, dia, _ = zone_state(tk)
            pres = " ".join(f"{z[0]}:{'Y' if present[z] else '-'}"
                            for z in ZONE_ORDER)
            dstr = " ".join(f"{z[0]}={dia[z]}" for z in ZONE_ORDER
                            if present[z] and dia[z] > 0)
            print(f"  f{i:4d}  {pres}   dia[{dstr}]")
    movers = sum(1 for t in tk.all_tracks() if t.is_moving())
    print(f"done: {movers} moving logs, "
          f"{len(tk.all_tracks()) - movers} stationary tracked")


def live(args):
    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    plc = RockwellPlc(args.plc_ip, args.slot)
    plc.connect()
    tk = LogTracker()
    print("bridge2 up: UDP", args.port, "-> Vision_Transport @", args.plc_ip)
    rid, writes = 200, 0
    last_fn = None
    t0 = time.time()
    last_write = 0.0
    try:
        while time.time() - t0 < args.duration:
            fn, segs = buf.latest_complete()
            if segs and fn != last_fn:
                last_fn = fn
                tk.update(detect_logs(segs))
            if time.time() - last_write >= args.interval:
                last_write = time.time()
                present, dia, arcs = zone_state(tk)
                rois = build_rois(present, dia, arcs)
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
                if writes % 20 == 0:
                    pres = " ".join(f"{z[0]}:{'Y' if present[z] else '-'}"
                                    for z in ZONE_ORDER)
                    print(f"  {writes} writes  {pres}  "
                          f"z4dia={dia['4_TopOfScrew']}", flush=True)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        buf.stop = True
        plc.close()
    print(f"done: {writes} writes in {time.time()-t0:.0f}s")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--replay")
    ap.add_argument("--plc-ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--interval", type=float, default=0.5)
    ap.add_argument("--duration", type=float, default=1800)
    args = ap.parse_args()
    if args.replay:
        replay(args.replay)
    else:
        live(args)


if __name__ == "__main__":
    main()
