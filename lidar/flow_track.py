"""FLOW MODE -- measure every log as it passes zone 4, stopped or moving,
and align the pass sequence against the optimizer record stream.

Runs as a thread inside interlace_capture.py (shares its FrameBuffer and
OptimizerTap). Per 20 Hz frame it runs detect_logs on the deck region and
follows the log whose center is in the zone-4 window:

  pass   = one log's stay in the window (entry -> exit, or an upstream jump
           of the tracked center = the next log took over)
  value  = median dia_arc over the pass's frames (same estimator as the
           stopped bursts), with IQR / n / duration / y travel

Every pass is appended to <dump>/flow_passes.csv. After each pass the last
ALIGN_WINDOW passes are aligned to the optimizer records that arrived after
them (monotonic dynamic-programming alignment, band error vs [SED, MID],
gaps allowed for missed detections / unseen logs, each record must follow
its pass by MIN_LAG..MAX_LAG seconds). Results -> flow_pairs.csv and a
console summary: how many passes paired, how many inside the band, MAE.

This is the "does the log have to stop?" experiment: compare the pass
statistics of moving logs (short duration, large y travel) with stopped
ones (long duration, ~0 travel).
"""
from __future__ import annotations

import csv
import json
import os
import time
from datetime import datetime
from pathlib import Path

import numpy as np

from lidar.arcfit import prefit_zones, zone_round_runs
from lidar.segment import detect_logs

Z4 = "4_TopOfScrew"
DECK_REGION = {"x": (-6.3, -1.3), "y": (-2.0, 1.0)}
Z4_REGION = {"x": (-6.3, -1.3), "y": (-1.05, 0.05)}   # zone-4 window + margin
DECK_EVERY = 4          # full-deck inventory (for the live view) every N frames
PRES_HIST = 5           # presence smoothing: last N inventory frames (~1 s)
PRES_MIN_HITS = 2       # present if >= this many of them had enough arcs
PRES_MIN_ARCS = {"7_InfeedDeck": 2,   # far field is sparse; default 3 arcs/frame
                 "2_CamWaiting": 2}   # cam-waiting station: few arcs of a log there (far
                                      # downstream edge of the view), so zone 2 relies on
                                      # PERSISTENCE more than count. Was 1: with the zone
                                      # empty, fixed structures at the band edges (y ~-1.73,
                                      # dia 400-560, and y ~-1.97) jitter single 5-6 point
                                      # arcs into the gates and lit the zone 42 % of the
                                      # time (burst cap_01_134722, 2026-10-19). 2 arcs: 0 %
                                      # there, and the 2026-10-15 at-rest captures with a
                                      # log in zone 2 (5-8 arcs/frame) are unchanged.
PRES_MIN_HITS_Z = {"2_CamWaiting": 3, # ...present if 3 of the last 5 frames had an arc
                   "4_TopOfScrew": 1} # zone 4: on with 1 frame, off after 2 empty (0.4 s)
PRES_HIST_Z = {"4_TopOfScrew": 2}     # zone 4 keeps only the last 2 inventory frames so
                                      # presence drops in the ~0.75 s between logs; with the
                                      # 2-of-5 rule the release (0.8 s) outlasted the gap and
                                      # zone 4 never read empty while the line ran
# zone-2 window override: probe_zones cut its x reach to 1.5 m to keep the deck
# wall out; wall arcs are now rejected by radius, so give it zone 3's reach
# (a log there yielded only 1-2 arcs/frame in the short window, 2026-10-15 07:28)
# zone 2 y: the log WAITING at the cam rests at y -1.85..-1.90 (today's captures:
# log-height objects 272-399 mm there in 5 of 9 stops, 125-148 mm high, which
# the segmenter files under zone 1); the old -1.68..-1.46 band only caught a
# log mid-deal, hence "zone 2 only triggers when a log has just been dealt".
# Outfeed-conveyor logs sit at y <= -2.0, so the band stops at -1.96.
WINDOW_OVERRIDE = {"2_CamWaiting": {"x": (-4.50, -2.20), "y": (-1.96, -1.46)},
                   # zone 4 PRESENCE band (diameter still uses Z4_REGION): 0.28 m
                   # instead of 0.46 m so consecutive logs (pitch ~0.56 m at
                   # 0.18 m/s) give a clear OFF between them; stopped logs sit
                   # at y -0.46..-0.54 (today's 9 captures), inside with margin
                   "4_TopOfScrew": {"x": (-4.50, -2.20), "y": (-0.64, -0.36)}}
Z2_DIAG_BAND = {"2_CamWaiting": (-2.10, -1.26)}   # y band of the zone-2 "near" diagnostic
PRES_R_LO, PRES_R_HI = 0.075, 0.19    # log radius for presence arcs (dia 150-380 mm)
PRES_PAD = 0.20                       # widen the point band by this [m]; arcs are then
                                      # attributed by their fitted centre, not their points
CAM_ZONES = ("2_CamWaiting", "3_CamStaging")
CAM_REST = (120.0, 150.0)   # CamKicker servo dwell (only parked position; 130-140 deg,
                            # ~1 s per cycle; rotates the rest of the cycle)
# per cam zone: servo angle range in which the lobes are CLEAR of that zone, so
# the LiDAR reading there is valid (user, 2026-10-15): zone 3 = 50..330,
# zone 2 = 80..150; outside those the zone shows amber "cam moving".
CAM_GOOD = {"3_CamStaging": (50.0, 330.0), "2_CamWaiting": (80.0, 150.0)}
CAM_LOG_H_MM = 85           # detected-object height: logs 86-149 mm, cam lobes 65-83
CAM_LOG_H_MOVING = 100      # stricter height while the lobes sweep (steeper lobe faces)
WALL_DIA = 360              # zone-2 objects above this are the outfeed deck wall
Z3_LOBE_Y = -1.15           # parked lobe sits at y ~ -1.08; staged logs at -1.18..-1.28
Z3_LOBE_DIA = 190           # lobe arcs read 148-175 mm; below this near the lobe = lobe
GAP_S = 0.35            # window empty this long -> pass SUSPENDED (ends only if
                        # the next log is elsewhere, or after RESUME_S of nothing)
RESUME_S = 8.0          # empty this long -> pass ended for good
SAME_LOG_Y = 0.06       # reappearing within this [m] of where it was = same log
JUMP_Y = 0.15           # tracked center jumps upstream by this -> new log
MIN_FRAMES = 10         # fewer frames than this -> fragment, not a pass (real
                        # passes run 30-55 frames; 5-7 frame fragments split off
                        # touching neighbours and only disturb the sequence)
LOWCONF_IQR = 15        # pass IQR above this = low confidence (touching logs,
                        # intermittent detection); still paired, but flagged
LOWCONF_N = 20          # fewer frames than this = thin track (partial view of
                        # the log, e.g. 12 frames reading 276 for a 307-414 log)


def low_confidence(p) -> bool:
    return p["iqr"] > LOWCONF_IQR or p["n"] < LOWCONF_N
MOVING_MM = 60          # y travel over the pass above this = moving log
ALIGN_WINDOW = 30       # passes per alignment
MIN_LAG, MAX_LAG = 8.0, 120.0   # record must arrive this long after pass end
                                 # (stopped captures: zone-4 log reached the scanner
                                 #  14-26 s after restart; chain logs arrive sooner)
GAP_PASS = 35.0         # cost of a pass with no record (false/duplicate detection)
GAP_REC = 25.0          # cost of a record with no pass (log missed by LiDAR / merged at a
                        # stop); lowered 35->25 on 2026-10-15 07:47 so the alignment skips
                        # a merged record instead of shifting a run of pairs by one
LAG_SUSPECT_S = 8.0     # pair lag this far from the window's median lag = suspect pairing
INSIDE_TOL = 15.0       # "inside band" tolerance for the summary [mm]
SUMMARY_EVERY = 10      # passes between flow_summary.csv rows


def anomalous_record(sed, led):
    """Optimizer records with SED > LED have shown up only on double feeds
    and split logs (238/208, 247/234): the scan did not see one normal log.
    Such a record is kept in the sequence (it still consumed a slot) but is
    scored as a wildcard and excluded from the accuracy statistics."""
    return sed > led


def band_err(d, sed, led):
    if anomalous_record(sed, led):
        return 0.0
    mid = (sed + led) / 2.0
    lo, hi = min(sed, mid), max(sed, mid)
    return 0.0 if lo <= d <= hi else (lo - d if d < lo else d - hi)


def align_passes(passes, records):
    """passes: list of dict(t_end, dia); records: list of (t, slot, sed, led, len).
    Returns list of (pass_idx, rec_idx or None) for every pass."""
    n, m = len(passes), len(records)
    if n == 0 or m == 0:
        return [(i, None) for i in range(n)]
    INF = 1e9
    cost = np.full((n, m), INF)
    for i, p in enumerate(passes):
        for j, r in enumerate(records):
            lag = r[0] - p["t_end"]
            if MIN_LAG <= lag <= MAX_LAG:
                cost[i, j] = band_err(p["dia"], r[2], r[3])
    dp = np.full((n + 1, m + 1), INF)
    dp[0, :] = 0.0                       # free leading records
    back = np.zeros((n + 1, m + 1), dtype=np.int8)
    for i in range(1, n + 1):
        for j in range(0, m + 1):
            best, arg = dp[i - 1, j] + GAP_PASS, 1          # pass unmatched
            if j > 0:
                c = dp[i - 1, j - 1] + cost[i - 1, j - 1]
                if c < best:
                    best, arg = c, 0                        # match
                c = dp[i, j - 1] + GAP_REC
                if c < best:
                    best, arg = c, 2                        # record skipped
            dp[i, j], back[i, j] = best, arg
    j = int(np.argmin(dp[n]))              # free trailing records
    i = n
    out = {}
    while i > 0:
        a = back[i, j]
        if a == 0:
            out[i - 1] = j - 1; i -= 1; j -= 1
        elif a == 1:
            out[i - 1] = None; i -= 1
        else:
            j -= 1
    return [(k, out.get(k)) for k in range(n)]


class FlowTracker:
    def __init__(self, buf, tap, dump):
        self.buf, self.tap = buf, tap
        self.dump = Path(dump)
        self.passes = []            # dicts
        self.final = {}             # t_start -> (pass, record|None, err) cumulative
        self.stop = False
        self._cur = None
        self._last_seen = 0.0
        self._last_fn = None
        self.pass_csv = self.dump / "flow_passes.csv"
        self.pair_csv = self.dump / "flow_pairs.csv"
        if not self.pass_csv.exists():
            with open(self.pass_csv, "w", newline="") as f:
                csv.writer(f).writerow(["iso_start", "t_start", "t_end", "dur_s", "n",
                                        "dia_med", "dia_iqr", "y_in", "y_out",
                                        "travel_mm", "moving"])

    # ---------------------------------------------------------- tracking
    def _z4_log(self, segs):
        """Zone-4 detection every frame on a narrow band (cheap); the
        full-deck inventory for the live display only every DECK_EVERY-th
        frame. detect_logs over the whole deck costs ~50 ms, which under CPU
        contention halved the tracker's frame rate (20 -> 10 fps, thin
        tracks); the zone-4 band alone is several times cheaper."""
        self._fi = getattr(self, "_fi", 0) + 1
        inventory = self._fi % DECK_EVERY == 0
        if inventory:
            # circle-fit every run of all nine zone passes of this frame in
            # ONE batch; detect_logs / _update_presence below then hit the
            # fit memo (same fits, far less per-call overhead)
            prefit_zones(segs, [Z4_REGION, DECK_REGION,
                                *(b for _, _, wide, diag in self._presence_bands()
                                  for b in ((wide, diag) if diag else (wide,)))])
        best = None
        for lg in detect_logs(segs, region=Z4_REGION):
            if lg["zone"] == Z4 and (best is None or lg["center"][1] < best["center"][1]):
                best = lg
        if inventory:
            logs = detect_logs(segs, region=DECK_REGION)
            self._update_presence(segs)
            self._publish(logs, best)
        return best

    @staticmethod
    def _presence_bands():
        """[(zone, window, wide_band, diag_band_or_None)]: the per-zone
        window (override applied) and the point bands _update_presence fits
        arcs in (window widened by PRES_PAD; zone 2 also its diagnostic
        band)."""
        from lidar.probe_zones import PRESENCE_WINDOWS
        out = []
        for z, win in PRESENCE_WINDOWS.items():
            win = WINDOW_OVERRIDE.get(z, win)
            y0, y1 = win["y"]
            wide = {"x": win["x"], "y": (y0 - PRES_PAD, y1 + PRES_PAD)}
            diag = {"x": win["x"], "y": Z2_DIAG_BAND[z]} if z in Z2_DIAG_BAND else None
            out.append((z, win, wide, diag))
        return out

    def _update_presence(self, segs):
        """Presence per zone = log-radius arc COUNT inside the tight per-zone
        window (probe_zones method, validated 2026-10-14 on zones 2/3 and
        small logs), smoothed over the last PRES_HIST frames. detect_logs is
        a DIAMETER tool (needs >= 6 arcs over >= 0.8 m) and misses small or
        partly visible logs, which is why zones 2, 3, 6, 7 read empty."""
        from collections import deque
        bands = self._presence_bands()
        if not hasattr(self, "_pres_hist"):
            self._pres_hist = {z: deque(maxlen=PRES_HIST_Z.get(z, PRES_HIST)) for z, _, _, _ in bands}
            self.presence = {}
        for z, win, wide, diag in bands:
            dias, ys = [], []
            # Fit arcs on a band widened by PRES_PAD so a log is seen whole,
            # then keep only arcs whose fitted CENTRE lies inside the zone's
            # own y band. Points of a neighbouring log (e.g. a zone-1 log at
            # y -1.8 whose top reaches -1.6) no longer light this zone.
            y0, y1 = win["y"]
            for run, r, c3 in zone_round_runs(segs, wide):   # batched circle fits
                # log radii only: screw flights fit 0.39-0.46 m dia and
                # must not count (the old 0.23 m limit let them through)
                if not (PRES_R_LO <= r <= PRES_R_HI and y0 <= c3[1] <= y1):
                    continue
                # ... and the run's own points must lie in the band as well.
                # A partial arc on the flank of a log in the NEXT zone fits a
                # larger circle whose centre is pushed away from its points,
                # across the zone border: on 2026-10-19 the lower flank of a
                # piece in zone 3 (points y -1.26..-1.40) gave 300-430 mm
                # "arcs" centred at -1.5, lighting zone 2 with nothing there.
                if not (y0 <= float(np.median(run[:, 1])) <= y1):
                    continue
                # zone 3: the parked cam lobe reads 148-175 mm at y -1.08;
                # arcs that close to it count only if clearly log-sized
                if z == "3_CamStaging" and c3[1] > Z3_LOBE_Y and r * 2000 < Z3_LOBE_DIA:
                    continue
                dias.append(r * 2000); ys.append(float(c3[1]))
            # zone-2 diagnostic: every round arc (any radius 50-600 mm) in the
            # whole zone-1/zone-2 area, so a log that rests outside the tight
            # window can be seen in live_state "raw" without a capture
            near = None
            if diag is not None:
                near = []
                for run, r, c3 in zone_round_runs(segs, diag):
                    if 0.025 <= r <= 0.30:
                        near.append((round(float(c3[1]), 2), int(r * 2000)))
            self._pres_hist[z].append(len(dias))
            need = PRES_MIN_ARCS.get(z, 3)
            hits = sum(1 for n in self._pres_hist[z] if n >= need)
            self.presence[z] = {"present": hits >= PRES_MIN_HITS_Z.get(z, PRES_MIN_HITS),
                                "arcs": int(np.median(self._pres_hist[z])),
                                "arc_dia": int(round(np.median(dias))) if dias else None,
                                "y": round(float(np.median(ys)), 2) if ys else None,
                                "near": near}

    # ---------------------------------------------------------- live state
    def _publish(self, logs, z4):
        """Write <dump>/live_state.json (atomic, <=10 Hz) for the live view:
        per-zone presence + diameter this frame, the zone-4 pass in progress,
        the last finished pass, the last pairing and the running score."""
        now = time.time()
        if now - getattr(self, "_pub_t", 0.0) < 0.1:
            return
        self._pub_t = now
        # Presence: zones 4-7 from the tight-window arc counts (diameter from
        # the segmenter when it has the log). Zones 2-3 are the CAM zones: a
        # rotating cam lobe is as round as a log and fools both the window
        # arcs and the segmenter, so there a log counts only when the
        # detected object has LOG height (>= CAM_LOG_H_MM; lobes read 65-83)
        # and the cam is at rest; while the cam is away from rest the zone is
        # reported as "cam moving" (not present, not empty).
        cam = getattr(self.tap, "cam_pos", None)
        if cam is not None:
            cam = cam % 360.0                 # servo reports past 360 while dealing

        def lobes_in(zone):
            """True while the cam lobes are sweeping through this zone."""
            if cam is None:
                return False
            lo, hi = CAM_GOOD[zone]
            return not (lo <= cam <= hi)
        cam_moving = {z: lobes_in(z) for z in CAM_ZONES}
        zones = {}
        for z, p in getattr(self, "presence", {}).items():
            # cam zones: window arcs count only while the lobes are clear of
            # the zone (zone 3 also filters the parked lobe's fingerprint)
            if z in CAM_ZONES and cam_moving[z]:
                continue
            # zone 2: tight window (tuned to exclude the deck wall), log-radius
            # arcs, cam at rest -- its logs' height is too soft to gate on
            if p["present"]:
                zones[z] = {"present": True, "dia": p["arc_dia"], "arcs": p["arcs"],
                            "y": p.get("y"), "src": "window"}
        for lg in logs:
            z = lg["zone"]
            if not z:
                continue
            if z in CAM_ZONES:
                if cam_moving[z]:
                    # lobes sweeping: only a clearly log-shaped object counts
                    # (cam dwells ~1 s at 130-140 deg, rotates the rest of the
                    # cycle, so without this the cam zones would be amber most
                    # of the time the line runs). Zone 3: never an object
                    # centred at the parked lobe's spot (y > Z3_LOBE_Y): with
                    # the servo reading 20.6 deg at rest (2026-10-19) the lobe
                    # at y -1.04..-1.11, height 97-106 mm, flickered zone 3
                    # present every few seconds. Staged logs sit at -1.18..-1.28.
                    lobe_spot = z == "3_CamStaging" and lg["center"][1] > Z3_LOBE_Y
                    # Zone 2: the window must agree, as the at-rest rule below
                    # demands. The segmenter builds a "log" from the same
                    # cross-border flank arcs the window now rejects (2026-10-19:
                    # 300 mm, 145 mm high, centred y -1.47, with zone 2 empty).
                    no_window = (z == "2_CamWaiting"
                                 and not getattr(self, "presence", {}).get(z, {}).get("present"))
                    if (not lobe_spot and not no_window
                            and lg["height_mm"] >= CAM_LOG_H_MOVING
                            and 190 <= lg["dia_arc_mm"] <= WALL_DIA):
                        zones[z] = {"present": True, "dia": lg["dia_arc_mm"], "arcs": lg["n_arcs"],
                                    "y": round(lg["center"][1], 2), "h": lg["height_mm"],
                                    "src": "detect+cam"}
                    else:
                        zones.setdefault(z, {"present": False, "cam_moving": True, "src": "cam"})
                    continue
                if z == "3_CamStaging" and (lg["height_mm"] < CAM_LOG_H_MM
                                            or lg["dia_arc_mm"] < 170):
                    continue                  # lobe fingerprint, not a log
                if z == "2_CamWaiting" and (z not in zones or lg["dia_arc_mm"] > WALL_DIA):
                    continue                  # window said empty, or wall arcs
                if z == "3_CamStaging" and z not in zones:
                    zones[z] = {"present": True, "arcs": 0}   # detect alone is enough here
                if z not in zones or lg["n_arcs"] > zones[z]["arcs"]:
                    zones[z] = {"present": True, "dia": lg["dia_arc_mm"], "arcs": lg["n_arcs"],
                                "y": round(lg["center"][1], 2), "h": lg["height_mm"], "src": "detect"}
                continue
            if z not in zones:
                continue
            if zones[z].get("src") == "window" or lg["n_arcs"] > zones[z]["arcs"]:
                zones[z].update({"dia": lg["dia_arc_mm"], "arcs": lg["n_arcs"],
                                 "y": round(lg["center"][1], 2), "src": "detect"})
        for z in CAM_ZONES:
            if cam_moving[z]:
                zones.setdefault(z, {"present": False, "cam_moving": True, "src": "cam"})
        cur = None
        if self._cur and self._cur["dias"]:
            d = np.asarray(self._cur["dias"], float)
            cur = {"dia": int(round(np.median(d))), "n": len(d),
                   "since": round(now - self._cur["t0"], 1),
                   "travel_mm": int(round((self._cur["ys"][0] - self._cur["ys"][-1]) * 1000))}
        raw = {z: {"arcs": p["arcs"], "y": p.get("y"), "dia": p.get("arc_dia"),
                   **({"near": p["near"]} if p.get("near") is not None else {})}
               for z, p in getattr(self, "presence", {}).items()}
        self.last_zones, self.last_cam_moving, self.last_cam = zones, cam_moving, cam   # for the PLC writer
        state = {"t": now, "iso": datetime.fromtimestamp(now).isoformat(timespec="seconds"),
                 "zones": zones, "raw": raw,
                 "z4_live": z4["dia_arc_mm"] if z4 else None,
                 "cam_pos": cam, "cam_moving": any(cam_moving.values()),
                 "pass_in_progress": cur,
                 "last_pass": self.passes[-1] if self.passes else None,
                 "last_pair": getattr(self, "_last_pair", None),
                 "score": getattr(self, "_score", None),
                 "passes_total": len(self.passes)}
        tmp = self.dump / "live_state.json.tmp"
        try:
            with open(tmp, "w") as f:
                json.dump(state, f)
            os.replace(tmp, self.dump / "live_state.json")
        except Exception:
            pass

    def _finish(self, now):
        c = self._cur
        self._cur = None
        if c is None or len(c["dias"]) < MIN_FRAMES:
            return
        d = np.asarray(c["dias"], float)
        travel = (c["ys"][0] - c["ys"][-1]) * 1000.0
        rec = {"t_start": c["t0"], "t_end": c["t1"], "n": len(d),
               "dia": int(round(np.median(d))),
               "iqr": int(round(np.percentile(d, 75) - np.percentile(d, 25))),
               "y_in": round(c["ys"][0], 2), "y_out": round(c["ys"][-1], 2),
               "travel": int(round(travel)), "moving": abs(travel) > MOVING_MM,
               "dur": round(c["t1"] - c["t0"], 1)}
        self.passes.append(rec)
        with open(self.pass_csv, "a", newline="") as f:
            csv.writer(f).writerow([
                datetime.fromtimestamp(rec["t_start"]).isoformat(timespec="seconds"),
                round(rec["t_start"], 2), round(rec["t_end"], 2), rec["dur"], rec["n"],
                rec["dia"], rec["iqr"], rec["y_in"], rec["y_out"], rec["travel"],
                int(rec["moving"])])
        kind = "MOVING " if rec["moving"] else "stopped"
        conf = ("  LOW-CONF (wide spread)" if rec["iqr"] > LOWCONF_IQR
                else "  LOW-CONF (thin track)" if rec["n"] < LOWCONF_N else "")
        print(f"[{time.strftime('%H:%M:%S')}] FLOW pass #{len(self.passes)} {kind} "
              f"dia {rec['dia']} ±{rec['iqr'] // 2} n={rec['n']} {rec['dur']}s "
              f"travel {rec['travel']:+d} mm{conf}", flush=True)
        self._align_and_report()

    def run(self):
        while not self.stop:
            fn, segs = self.buf.latest_complete()
            if not segs or fn == self._last_fn:
                time.sleep(0.01)
                continue
            self._last_fn = fn
            now = time.time()
            lg = self._z4_log(segs)
            if lg is None:
                # Detection flicker on a STOPPED log must not end the pass:
                # a 197 mm log parked 50 s was split into 7 passes that ate
                # 6 optimizer records. So a gap only SUSPENDS the pass; it
                # ends when the window stays empty for RESUME_S, or when the
                # next detection is at a different position (handled below).
                if self._cur is not None and now - self._last_seen > RESUME_S:
                    self._finish(now)
                continue
            y = lg["center"][1]
            if self._cur is not None:
                gap = now - self._last_seen
                moved = abs(y - self._cur["ys"][-1])
                if y - self._cur["ys"][-1] > JUMP_Y:
                    self._finish(now)               # next log took over (upstream jump)
                elif gap > GAP_S and moved > SAME_LOG_Y:
                    self._finish(now)               # window was empty, different log now
                # else: same log (stopped or crawling) -> keep accumulating
            if self._cur is None:
                self._cur = {"t0": now, "t1": now, "dias": [], "ys": []}
            self._cur["t1"] = now
            self._cur["dias"].append(lg["dia_arc_mm"])
            self._cur["ys"].append(y)
            self._last_seen = now

    # ---------------------------------------------------------- pairing
    def _align_and_report(self):
        passes = self.passes[-ALIGN_WINDOW:]
        if not passes:
            return
        recs = self.tap.since(passes[0]["t_start"] - 1.0)
        # only passes whose records could already have arrived
        now = time.time()
        ready = [p for p in passes if now - p["t_end"] > MIN_LAG]
        if not ready or not recs:
            return
        pairs = align_passes(ready, recs)
        rows, errs_mov, errs_stop, errs_low = [], [], [], []
        lags = [recs[j][0] - ready[i]["t_end"] for i, j in pairs if j is not None]
        lag_med = float(np.median(lags)) if lags else 0.0
        n_lag_suspect = 0
        for i, j in pairs:
            p = ready[i]
            if j is None:
                rows.append((p, None, None)); continue
            r = recs[j]
            e = band_err(p["dia"], r[2], r[3])
            rows.append((p, r, e))
            if anomalous_record(r[2], r[3]):
                continue                    # optimizer anomaly: not a LiDAR test
            if abs((r[0] - p["t_end"]) - lag_med) > LAG_SUSPECT_S:
                n_lag_suspect += 1          # record arrived off-rhythm (stop/slip): suspect pair
                continue
            if low_confidence(p):
                errs_low.append(e)          # wide-spread / thin pass: reported apart
            else:
                (errs_mov if p["moving"] else errs_stop).append(e)
        # cumulative table: passes that have left the alignment window keep
        # their last pairing; the window's entries are refreshed each time
        for p, r, e in rows:
            self.final[p["t_start"]] = (p, r, e)
        with open(self.pair_csv, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["iso_start", "moving", "n", "dur_s", "travel_mm", "dia", "iqr",
                        "slot", "sed", "led", "mid", "band_err", "lag_s"])
            for p, r, e in (self.final[k] for k in sorted(self.final)):
                if r is None:
                    w.writerow([datetime.fromtimestamp(p["t_start"]).isoformat(timespec="seconds"),
                                int(p["moving"]), p["n"], p["dur"], p["travel"], p["dia"],
                                p["iqr"], "", "", "", "", "", ""])
                else:
                    w.writerow([datetime.fromtimestamp(p["t_start"]).isoformat(timespec="seconds"),
                                int(p["moving"]), p["n"], p["dur"], p["travel"], p["dia"],
                                p["iqr"], r[1], r[2], r[3], round((r[2] + r[3]) / 2),
                                round(e, 1), round(r[0] - p["t_end"], 1)])
        def summ(v):
            if not v:
                return "n=0"
            a = np.asarray(v)
            return (f"n={len(a)} inside(<={INSIDE_TOL:.0f}mm) {int((a <= INSIDE_TOL).sum())}"
                    f" MAE {a.mean():.0f} max {a.max():.0f}")
        unp = sum(1 for _, r, _ in rows if r is None)
        print(f"   FLOW PAIRING (last {len(ready)} passes, {len(recs)} records): "
              f"unpaired {unp} | moving {summ(errs_mov)} | stopped {summ(errs_stop)}"
              f" | low-conf {summ(errs_low)} | lag-suspect {n_lag_suspect} (median lag {lag_med:.0f}s)",
              flush=True)
        # for the live view
        clean = errs_mov + errs_stop
        self._score = {"paired": len(clean), "inside": int(sum(1 for e in clean if e <= INSIDE_TOL)),
                       "mae": round(float(np.mean(clean)), 1) if clean else None,
                       "unpaired": unp, "window": len(ready)}
        for p, r, e in reversed(rows):
            if r is not None:
                self._last_pair = {"lidar": p["dia"], "sed": r[2], "led": r[3],
                                   "mid": round((r[2] + r[3]) / 2), "band_err": round(e, 1),
                                   "moving": p["moving"], "slot": r[1],
                                   "lag_s": round(r[0] - p["t_end"], 1)}
                break
        # compact summary row every SUMMARY_EVERY passes (chat monitor hooks this)
        if len(self.passes) % SUMMARY_EVERY == 0:
            sp = self.dump / "flow_summary.csv"
            new = not sp.exists()
            with open(sp, "a", newline="") as f:
                w = csv.writer(f)
                if new:
                    w.writerow(["iso_time", "passes_total", "window", "unpaired",
                                "moving_n", "moving_inside", "moving_mae", "moving_max",
                                "stopped_n", "stopped_inside", "stopped_mae", "stopped_max",
                                "lowconf_n", "lowconf_inside", "lowconf_mae", "lowconf_max"])
                def nums(v):
                    if not v:
                        return [0, 0, "", ""]
                    a = np.asarray(v)
                    return [len(a), int((a <= INSIDE_TOL).sum()), round(a.mean(), 1), round(a.max(), 1)]
                w.writerow([time.strftime("%H:%M:%S"), len(self.passes), len(ready), unp,
                            *nums(errs_mov), *nums(errs_stop), *nums(errs_low)])
        for p, r, e in rows[-3:]:
            if r is not None:
                print(f"      {'MOVING ' if p['moving'] else 'stopped'} dia {p['dia']:3d} "
                      f"<-> slot {r[1]:2d} SED {r[2]} LED {r[3]} band-err {e:+.0f} "
                      f"lag {r[0] - p['t_end']:.0f}s", flush=True)
