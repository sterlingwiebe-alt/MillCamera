"""STEP 1 -- stopped-deck zone-4 capture: single-frame vs interlaced-pooled.

Arms on the PLC BOOL `Vision_Capture` (rising edge, then --ack clears it).
On each trigger: grabs a ~1.5 s LiDAR burst (>= 7 interlace cycles), saves
the raw frames, and measures the zone-4 log two ways:

    single  -- detect_logs per 20 Hz frame (yesterday's method)
    pooled  -- detect_logs per 4-frame interlaced cycle (0.125 deg)

and prints both with their spread across frames/cycles, plus TestPtr and
the tail of TestLog for pairing. Deck must be STOPPED for the pooled
numbers to mean anything.

    python lidar/interlace_capture.py --ack --dump lidar/z4captures/interlaced
    python lidar/interlace_capture.py --replay <pkl | recording dir>
"""
from __future__ import annotations

import argparse
import csv
import pickle
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar.segment import detect_logs
from lidar.probe_zones import (PRESENCE_WINDOWS, DIA_ZONE, frame_zone_arcs,
                               MIN_ARCS_FRAME)
from lidar.interlace import cycles, scaled_gates, CYCLE

CAM_TAG = "CamKicker_Servo.ActualPosition"
NSLOT = 60                      # TestPtr caps at 59 (user, 2026-10-15); array itself is 70 long
PAIR_STALL_S = 45.0             # TestPtr unchanged this long = line stopped, align now
PAIR_MAX_OFFSET = 3             # user (2026-10-15): never more than 3 logs ahead of the deck
PAIR_MIN_MARGIN = 4.0           # best offset must beat the runner-up by this [mm, band metric]
MOVING_MM = 20.0                # zone-4 median-y drift over the burst above this = deck moving
CSV_FIELDS = ["time", "n", "cam", "testptr", "testlog_tail",
              "single_n", "single_med", "single_iqr", "single_arcs",
              "pooled_n", "pooled_med", "pooled_iqr", "pooled_arcs",
              "pooled_beams", "pkl"]


# -------------------------------------------------------------- measuring
def z4_from_logs(logs):
    for lg in logs:
        if lg["zone"] == DIA_ZONE:
            return lg
    return None


def _stats(vals):
    v = np.asarray([x for x in vals if x is not None], float)
    if len(v) == 0:
        return (0, -1, -1)
    return (len(v), int(round(np.median(v))),
            int(round(np.percentile(v, 75) - np.percentile(v, 25))))


def measure_single(frames):
    dias, arcs = [], []
    for segs in frames.values():
        lg = z4_from_logs(detect_logs(segs))
        if lg:
            dias.append(lg["dia_arc_mm"]); arcs.append(lg["n_arcs"])
    n, med, iqr = _stats(dias)
    return {"n": n, "med": med, "iqr": iqr, "frames": len(frames),
            "arcs": int(np.median(arcs)) if arcs else 0, "dias": dias}


def measure_pooled(frames):
    dias, arcs, beams = [], [], []
    ncyc = 0
    with scaled_gates(CYCLE):
        for fn0, pooled in cycles(frames):
            ncyc += 1
            lg = z4_from_logs(detect_logs(pooled))
            if lg:
                dias.append(lg["dia_arc_mm"]); arcs.append(lg["n_arcs"])
            # beams per zone-4 arc on the pooled frame
            from lidar.arcfit import round_runs, zone_line_points
            for seg in pooled:
                for lp in zone_line_points(seg, PRESENCE_WINDOWS[DIA_ZONE]):
                    for run, r, c3 in round_runs(lp):
                        beams.append(len(run))
    n, med, iqr = _stats(dias)
    return {"n": n, "med": med, "iqr": iqr, "cycles": ncyc,
            "arcs": int(np.median(arcs)) if arcs else 0,
            "beams": int(np.median(beams)) if beams else 0, "dias": dias}


def z4_present(frames):
    hits = sum(len(frame_zone_arcs(s, PRESENCE_WINDOWS[DIA_ZONE])) >= MIN_ARCS_FRAME
               for s in frames.values())
    return hits, len(frames)


DECK_REGION = {"x": (-6.3, -1.3), "y": (-2.0, 1.0)}   # zones 1..6


def deck_inventory(frames, gap=0.1):
    # gap 0.1 m: the zone-3 cam lobe parks ~0.15 m from the staged log and
    # must not be merged into it (cap_01: log 383 vs cam 148 at y -1.23/-1.08)
    """Every log on the deck, single-frame detect per frame, grouped by
    y position. Returns rows sorted by y (most downstream first):
    (y, x, zone, n_frames, dia_arc_med, dia_arc_halfiqr, width_med)."""
    dets = []
    for segs in frames.values():
        for lg in detect_logs(segs, region=DECK_REGION):
            dets.append((lg["center"][1], lg["center"][0], lg["zone"],
                         lg["dia_arc_mm"], lg["width_mm"]))
    dets.sort()
    rows, cur = [], []
    for d in dets:
        if cur and d[0] - cur[-1][0] > gap:
            rows.append(cur); cur = []
        cur.append(d)
    if cur:
        rows.append(cur)
    out = []
    for grp in rows:
        a = np.array([[g[0], g[1], g[3], g[4]] for g in grp], float)
        zones = [g[2] for g in grp if g[2]]
        zone = max(set(zones), key=zones.count) if zones else "-"
        out.append((float(np.median(a[:, 0])), float(np.median(a[:, 1])), zone,
                    len(grp), int(np.median(a[:, 2])),
                    int((np.percentile(a[:, 2], 75) - np.percentile(a[:, 2], 25)) / 2),
                    int(np.median(a[:, 3]))))
    return out


def print_inventory(frames, tag=""):
    print(f"{tag} DECK INVENTORY (single-frame detect, downstream first; "
          f"flow order = TestLog order after the outfeed clears):")
    for y, x, zone, n, dia, hiqr, width in deck_inventory(frames):
        print(f"{tag}   y {y:+.2f}  {zone:18s} dia_arc {dia:4d} ±{hiqr:<3d} "
              f"width {width:4d}  (seen {n}/{len(frames)} frames)")


def pairable_logs(inv, min_seen_frac=0.2, nframes=1):
    """Deck inventory rows that can be paired with optimizer records:
    drop zone 1 (outfeed trough contaminates the fit) and cam lobes
    (dia_arc < 170; the zone-3 cam reads ~148-156)."""
    # Zone 5 (mid-screw, multi-piece) and zone 3 (log on the cams) stay IN
    # the sequence -- their logs still produce optimizer records -- but get
    # low weight in the score (see STATION_WEIGHT); zone 4 and 6 decide.
    return [r for r in inv
            if r[2] != "1_OutfeedConveyor" and r[4] >= 170
            and r[3] >= min_seen_frac * nframes]


STATION_WEIGHT = {"2_CamWaiting": 0.5, "3_CamStaging": 0.5, "4_TopOfScrew": 1.0,
                  "5_MidScrew": 0.0, "6_ScrewDeck": 1.0, "7_InfeedDeck": 0.5}
WALL_DIA = 360          # zone 1/2 readings above this are deck-wall arcs, not logs


def station_weight(row):
    zone, dia = row[2], row[4]
    if zone in ("1_OutfeedConveyor", "2_CamWaiting") and dia > WALL_DIA:
        return 0.0
    return STATION_WEIGHT.get(zone, 0.5)


def interval_err(d, sed, led):
    """LiDAR sees ~2.4 m of the log's MIDDLE, so a correct reading lies
    between the optimizer's SED and MID (never below SED, never above
    MID on every clear pair so far). Error = distance to that interval."""
    mid = (sed + led) / 2.0
    lo, hi = min(sed, mid), max(sed, mid)
    return 0.0 if lo <= d <= hi else (lo - d if d < lo else d - hi)


def align(deck_dias, slot_rows, max_offset=3, weights=None):
    """Sequence-align deck diameters (downstream first) to optimizer
    records (sed, led, len) in slot order, scoring each pair by
    interval_err. Returns (best_offset, score, pairs, scores). Offset k
    means k records were already past the deck (chain logs ahead of the
    scanner); the user states k <= 3 for this deck."""
    w = list(weights) if weights is not None else [1.0] * len(deck_dias)
    scores = {}
    for off in range(0, max_offset + 1):
        if off + len(deck_dias) > len(slot_rows):
            continue
        errs = [interval_err(d, slot_rows[off + i][0], slot_rows[off + i][1])
                for i, d in enumerate(deck_dias)]
        scores[off] = float(np.average(errs, weights=w)) if sum(w) > 0 else float(np.mean(errs))
    if not scores:
        return None, None, [], {}
    best = min(scores, key=scores.get)
    pairs = [(i, d, best + i, (slot_rows[best + i][0] + slot_rows[best + i][1]) / 2.0)
             for i, d in enumerate(deck_dias)]
    return best, scores[best], pairs, scores


def read_slots(plc, lo, hi):
    out = []
    for i in range(lo, hi + 1):
        tags = [f"TestLog[{i}].SED", f"TestLog[{i}].LED", f"TestLog[{i}].Length"]
        try:
            rs = plc.Read(tags)
            out.append(tuple(int(r.Value) if r.Status == "Success" else -1 for r in rs))
        except Exception:
            out.append((-1, -1, -1))
    return out


class OptimizerTap:
    """Continuous copy of every optimizer record as it appears in TestLog.

    Polls TestPtr a few times a second on its own PLC connection; each new
    slot is read IMMEDIATELY and appended (with wall time) to an in-memory
    list and to <dump>/optimizer_records.csv. A line-restart reset (TestPtr
    dropping) therefore cannot erase anything -- the old race between
    "user resets before the next stop" and "pairing reads the slots" is
    gone. Pairing aligns against the records that arrived AFTER a capture.
    """

    def __init__(self, ip, slot, dump):
        self.ip, self.slot = ip, slot
        self.path = Path(dump) / "optimizer_records.csv"
        self.records = []           # (t, slot, sed, led, len)
        self.lock = threading.Lock()
        self.last_change = time.time()
        self.stop = False
        self.cam_pos = None         # CamKicker servo angle, refreshed each poll

    def run(self):
        from pylogix import PLC
        new = not self.path.exists()
        with open(self.path, "a", newline="") as f:
            w = csv.writer(f)
            if new:
                w.writerow(["iso_time", "epoch", "slot", "sed", "led", "len"])
                f.flush()
            last = None
            while not self.stop:
                try:
                    with PLC(ip_address=self.ip, slot=self.slot) as plc:
                        while not self.stop:
                            r = plc.Read("TestPtr")
                            if r.Status != "Success":
                                time.sleep(1.0); continue
                            ptr = int(r.Value)
                            cr = plc.Read(CAM_TAG)
                            if cr.Status == "Success":
                                self.cam_pos = round(float(cr.Value), 1)
                            if last is None:
                                last = ptr
                            elif ptr != last:
                                now = time.time()
                                if ptr >= last:
                                    todo = range(last, min(ptr, NSLOT))
                                elif last >= NSLOT - 1:
                                    # ring wrap 59 -> 0: slot 59 was written, then 0..ptr-1
                                    todo = list(range(last, NSLOT)) + list(range(0, ptr))
                                else:
                                    todo = range(0, ptr)           # manual reset
                                for k in todo:
                                    sed, led, ln = read_slots(plc, k, k)[0]
                                    rec = (now, k, sed, led, ln)
                                    with self.lock:
                                        self.records.append(rec)
                                        self.last_change = now
                                    w.writerow([datetime.fromtimestamp(now).isoformat(timespec="seconds"),
                                                round(now, 2), k, sed, led, ln])
                                f.flush()
                                last = ptr
                            time.sleep(0.4)
                except Exception as e:          # noqa: BLE001
                    print(f"optimizer tap: {type(e).__name__}: {e} -- reconnecting",
                          flush=True)
                    time.sleep(3.0)

    def since(self, t):
        with self.lock:
            return [r for r in self.records if r[0] > t]


def pair_later(tap, deck, n, stamp, dump, tb_capture, timeout_s=2400):
    """Background: wait for the deck's logs to reach the optimizer after
    the capture, then sequence-align the deck (downstream first) to the
    records that arrived after the capture, within the user's offset
    limit, and print per-station errors."""
    need = len(deck)
    t0 = time.time()
    try:
        while time.time() - t0 < timeout_s:
            time.sleep(2.0)
            recs = tap.since(tb_capture)
            stalled = time.time() - tap.last_change > PAIR_STALL_S
            # offset is bounded (<= PAIR_MAX_OFFSET logs ahead of the deck),
            # so once need + max_offset + 1 records are in, align at once;
            # a stall with at least `need` records also triggers it.
            if len(recs) >= need + PAIR_MAX_OFFSET + 1 or (len(recs) >= need and stalled):
                rows = [(sed, led, ln) for _, _, sed, led, ln in recs]
                dias = [d[4] for d in deck]
                wts = [station_weight(d) for d in deck]
                off, mae, pairs, scores = align(
                    dias, rows, max_offset=min(PAIR_MAX_OFFSET, max(0, len(rows) - need)),
                    weights=wts)
                others = sorted(v for k, v in scores.items() if k != off)
                margin = (others[0] - mae) if others else 99.0
                verdict = ("AMBIGUOUS -- deck logs too similar in size to anchor "
                           "the sequence; do not trust these pairs"
                           if margin < PAIR_MIN_MARGIN else
                           f"clear (next-best offset is {margin:.0f} mm worse)")
                print(f"\n[{time.strftime('%H:%M:%S')}] PAIRING for capture #{n} "
                      f"({stamp}): {len(recs)} records after capture, "
                      f"offset scores {{{', '.join(f'{k}:{v:.0f}' for k, v in scores.items())}}} mm"
                      f" -> OFFSET {off} (MAE {mae:.0f} mm) {verdict}", flush=True)
                with open(Path(dump) / "pairing.csv", "a", newline="") as f:
                    w = csv.writer(f)
                    for (i, d, s, m), row in zip(pairs, deck):
                        t_rec, slot_k, sed, led, ln = recs[s]
                        ie = interval_err(d, sed, led)
                        print(f"   {row[2]:18s} lidar {d:4d}  <->  record {s:2d} (slot {slot_k:2d}) "
                              f"SED {sed} LED {led} MID {m:.0f}  vsSED {d - sed:+.0f} vsMID {d - m:+.0f} "
                              f"band-err {ie:+.0f}  w{station_weight(row):.1f}"
                              f"   arrived +{t_rec - tb_capture:.0f}s", flush=True)
                        w.writerow([stamp, n, row[2], round(row[0], 2), d, row[6],
                                    slot_k, sed, led, ln, off, round(mae, 1),
                                    "ambiguous" if margin < PAIR_MIN_MARGIN else "clear",
                                    round(t_rec - tb_capture, 1), round(ie, 1)])
                tl = [(k, round(t - tb_capture)) for t, k, *_ in recs[:10]]
                print(f"   first records after capture (slot: +s): {tl}", flush=True)
                return
        print(f"\n[{time.strftime('%H:%M:%S')}] PAIRING for capture #{n}: timed out "
              f"(need {need} records after capture)", flush=True)
    except Exception as e:      # noqa: BLE001
        print(f"\nPAIRING thread for capture #{n} failed: {type(e).__name__}: {e}",
              flush=True)


def z4_drift_mm(frames):
    """Median-y drift of the zone-4 window points over the burst [mm];
    a stopped deck gives < 5 mm, a creeping one tens of mm."""
    from lidar.arcfit import zone_line_points
    ys = []
    for segs in frames.values():
        P = [lp for seg in segs for lp in zone_line_points(seg, PRESENCE_WINDOWS[DIA_ZONE])]
        if P:
            ys.append(float(np.median(np.vstack(P)[:, 1])))
    if len(ys) < 2:
        return 0.0
    return abs(ys[-1] - ys[0]) * 1000.0


def report(frames, tag=""):
    from lidar.interlace import is_interlaced
    s = measure_single(frames)
    interlaced = is_interlaced(frames)
    p = measure_pooled(frames) if interlaced else {
        "n": 0, "med": -1, "iqr": -1, "cycles": 0, "arcs": 0, "beams": 0, "dias": []}
    hits, nfr = z4_present(frames)
    print_inventory(frames, tag)
    drift = z4_drift_mm(frames)
    if drift > MOVING_MM:
        print(f"{tag} *** DECK MOVING during burst: zone-4 drift {drift:.0f} mm "
              f"-- single/pooled numbers unreliable, flip again once stopped ***")
    else:
        print(f"{tag} deck still (zone-4 drift {drift:.0f} mm)")
    print(f"{tag} z4 present {hits}/{nfr} frames")
    print(f"   single : dia_arc med {s['med']:4d} mm  IQR {s['iqr']:3d}  "
          f"(detected {s['n']}/{s['frames']} frames, ~{s['arcs']} arcs)")
    if interlaced:
        print(f"   pooled : dia_arc med {p['med']:4d} mm  IQR {p['iqr']:3d}  "
              f"(detected {p['n']}/{p['cycles']} cycles, ~{p['arcs']} arcs, "
              f"~{p['beams']} beams/arc)")
    else:
        print("   pooled : (sensor interlacing is OFF -- single frame only)")
    if s["dias"]:
        print("   single dias:", s["dias"][:16])
    if p["dias"]:
        print("   pooled dias:", p["dias"])
    return s, p


# ----------------------------------------------------------------- replay
def load_any(path):
    path = Path(path)
    if path.is_dir():
        from lidar.pointcloud import load_segments
        frames = defaultdict(list)
        for seg in load_segments(path):
            frames[seg["Modules"][0]["FrameNumber"]].append(seg)
        keys = sorted(frames)[1:-1]
        return {k: frames[k] for k in keys if len(frames[k]) >= 11}
    with open(path, "rb") as f:
        d = pickle.load(f)
    return d["frames"] if isinstance(d, dict) and "frames" in d else d


# --------------------------------------------------------------- plc bits
def read_testlog_tail(plc, ptr, n=4):
    """Read TestLog[ptr-n+1 .. ptr] SED/LED/Len in 3-tag batches (pylogix
    list-reads overflow above ~a dozen tags)."""
    out = []
    lo = max(0, ptr - n + 1)
    for i in range(lo, ptr + 1):
        tags = [f"TestLog[{i}].SED", f"TestLog[{i}].LED", f"TestLog[{i}].Length"]
        try:
            rs = plc.Read(tags)
            vals = [int(r.Value) if r.Status == "Success" else -1 for r in rs]
        except Exception:
            vals = [-1, -1, -1]
        out.append((i, *vals))
    return out


def live(args):
    from pylogix import PLC
    from lidar.live_bridge2 import FrameBuffer
    dump = Path(args.dump); dump.mkdir(parents=True, exist_ok=True)
    csv_path = dump / "interlaced.csv"
    new = not csv_path.exists()
    # SICK Compact parser: the vectorised _read_beam_data (lidar/fast_compact.py,
    # bit-identical output, ~13x cheaper) is patched into scansegmentapi before
    # the receiver thread starts. The first --raw-dump datagrams are recorded
    # verbatim for lidar/compact_check.py (overwritten on every start).
    from lidar import fast_compact
    raw_path = dump / "raw_payloads.bin" if args.raw_dump > 0 else None
    fast_compact.install(fast=not args.no_fast_parse, raw_dump=raw_path,
                         raw_count=args.raw_dump)
    print(f"SICK parser: {'ORIGINAL (--no-fast-parse)' if args.no_fast_parse else 'fast (lidar/fast_compact.py)'}"
          + (f"; recording the first {args.raw_dump} datagrams -> {raw_path}" if raw_path else ""),
          flush=True)
    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    tap = OptimizerTap(args.ip, args.slot, dump)
    threading.Thread(target=tap.run, daemon=True).start()
    if not args.no_flow:
        from lidar.flow_track import FlowTracker
        flow = FlowTracker(buf, tap, dump)
        threading.Thread(target=flow.run, daemon=True).start()
        print("flow mode ON: every zone-4 pass (moving or stopped) is measured and "
              "sequence-paired -> flow_passes.csv / flow_pairs.csv", flush=True)
        if args.plc_out:
            from lidar.plc_out import PlcOut
            PlcOut(flow, tap, args.ip, args.slot).start()
    n = 0; last = False; t0 = time.time()
    with PLC(ip_address=args.ip, slot=args.slot) as plc, \
            open(csv_path, "a", newline="") as cf:
        wr = csv.DictWriter(cf, fieldnames=CSV_FIELDS)
        if new:
            wr.writeheader(); cf.flush()
        print(f"ARMED on '{args.tag}' -- stop the deck with a log in zone 4, "
              f"flip the tag. burst {args.seconds}s -> {dump}", flush=True)
        while time.time() - t0 < args.duration:
            r = plc.Read(args.tag)
            if r.Status != "Success":
                print("trigger read failed:", r.Status, flush=True)
                time.sleep(2); continue
            v = bool(r.Value)
            if v and not last:
                n += 1
                cr = plc.Read(CAM_TAG)
                cam = round(float(cr.Value), 1) if cr.Status == "Success" else -1
                pr = plc.Read("TestPtr")
                ptr = int(pr.Value) if pr.Status == "Success" else -1
                tb = time.time(); frames = {}
                while time.time() - tb < args.seconds:
                    fn, segs = buf.latest_complete()
                    if segs and fn not in frames:
                        frames[fn] = segs
                    time.sleep(0.02)
                tail = read_testlog_tail(plc, ptr) if ptr >= 0 else []
                stamp = time.strftime("%H%M%S")
                pkl = f"cap_{n:02d}_{stamp}.pkl"
                # synchronous camera frame (which log was actually in zone 4)
                try:
                    import cv2
                    from lidar.light_bank import grab_camera, CAM_URL
                    img = grab_camera(CAM_URL)
                    if img is not None:
                        cv2.imwrite(str(dump / f"cap_{n:02d}_{stamp}_camera.jpg"),
                                    img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                except Exception as e:      # noqa: BLE001
                    print("   camera grab failed:", type(e).__name__, flush=True)
                with open(dump / pkl, "wb") as f:
                    pickle.dump({"frames": frames, "cam": cam, "testptr": ptr,
                                 "testlog_tail": tail, "t": tb}, f)
                print(f"\n[{time.strftime('%H:%M:%S')}] CAPTURE #{n} cam={cam} "
                      f"frames={len(frames)} TestPtr={ptr} -> {pkl}", flush=True)
                s, p = report(frames, "  ")
                print(f"   TestLog tail (slot: SED LED Len): {tail}", flush=True)
                deck = pairable_logs(deck_inventory(frames), nframes=len(frames))
                if deck:
                    print(f"   pairing armed: waiting for {len(deck)} deck logs to reach "
                          f"the optimizer after this capture", flush=True)
                    threading.Thread(target=pair_later,
                                     args=(tap, deck, n, stamp, dump, tb),
                                     daemon=True).start()
                wr.writerow({
                    "time": stamp, "n": n, "cam": cam, "testptr": ptr,
                    "testlog_tail": str(tail),
                    "single_n": s["n"], "single_med": s["med"],
                    "single_iqr": s["iqr"], "single_arcs": s["arcs"],
                    "pooled_n": p["n"], "pooled_med": p["med"],
                    "pooled_iqr": p["iqr"], "pooled_arcs": p["arcs"],
                    "pooled_beams": p["beams"], "pkl": pkl})
                cf.flush()
                if args.ack:
                    plc.Write(args.tag, 0)
            last = v
            time.sleep(0.2)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--seconds", type=float, default=1.5)
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--duration", type=float, default=6 * 3600)
    ap.add_argument("--dump", default="lidar/z4captures/interlaced")
    ap.add_argument("--replay", default="")
    ap.add_argument("--no-fast-parse", action="store_true",
                    help="use the scansegmentapi package's own beam parser instead of "
                         "lidar/fast_compact.py (A/B timing only; same output)")
    ap.add_argument("--raw-dump", type=int, default=240, metavar="N",
                    help="record the first N sensor datagrams verbatim to <dump>/raw_payloads.bin "
                         "at start-up, the verification set for lidar/compact_check.py (0 = off)")
    ap.add_argument("--no-flow", action="store_true",
                    help="disable continuous flow-mode pass tracking")
    ap.add_argument("--plc-out", action="store_true",
                    help="publish the live deck state to Vision_Transport DINT[40] "
                         "(layout v2, see lidar/plc_out.py) + Vision_Heartbeat")
    args = ap.parse_args()
    if args.replay:
        frames = load_any(args.replay)
        print(f"replay {args.replay}: {len(frames)} frames")
        report(frames, "  ")
        return
    live(args)


if __name__ == "__main__":
    main()
