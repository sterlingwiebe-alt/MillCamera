"""Unified PLC-triggered probe.

PRESENCE (zones 2-7): count log-radius arcs inside a TIGHT per-zone window
that sees only the log station -- NOT the deck wall / structure around it
(so no filtering or consistency gate is needed; we just don't look at the
junk). Presence = enough log-radius arcs per frame, over a short burst.

DIAMETER (zone 4 only): aggregate detect_logs arcs -> circle-fit AVG, for
the optimizer comparison.

Also reports the cam servo position and, for cam-affected zone 3, the
detected-object dia/height/y (cam ~170/65/y-1.09 vs log).

    python lidar/probe_zones.py --tag Vision_Capture --ack
"""
from __future__ import annotations

import argparse
import sys
import threading
import time
from collections import defaultdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC
from lidar.arcfit import round_runs, zone_line_points
from lidar.segment import detect_logs
from lidar.stage_measure import taper_from_arcs
from lidar.live_bridge2 import FrameBuffer

CAM_TAG = "CamKicker_Servo.ActualPosition"
R_LO, R_HI = 0.075, 0.23          # log radius [m] (dia 150-460); wall > this
MIN_ARCS_FRAME = 3                 # log-radius arcs/frame to call a frame "present"
PRESENT_FRAC = 0.4                 # present if >= this fraction of frames are

# TIGHT presence windows in RAW sensor coords -- each sees only the log
# station, not the surrounding wall/structure. (2_CamWaiting tightened
# 2026-10-14 to exclude the outfeed deck wall that was poisoning detection.)
PRESENCE_WINDOWS = {
    "2_CamWaiting":      {"x": (-3.70, -2.20), "y": (-1.68, -1.46)},
    "3_CamStaging":      {"x": (-4.50, -2.20), "y": (-1.43, -1.00)},
    "4_TopOfScrew":      {"x": (-4.50, -2.20), "y": (-0.73, -0.27)},
    "5_MidScrew":        {"x": (-4.50, -2.20), "y": (-0.05, 0.30)},
    "6_ScrewDeck":       {"x": (-4.80, -2.20), "y": (0.50, 1.60)},
    "7_InfeedDeck":      {"x": (-4.80, -2.20), "y": (1.75, 3.20)},
}
DIA_ZONE = "4_TopOfScrew"


def frame_zone_arcs(segs, win):
    """log-radius arc diameters (mm) in one frame within the window."""
    out = []
    for seg in segs:
        for lp in zone_line_points(seg, win):
            for run, r, c3 in round_runs(lp):
                if R_LO <= r <= R_HI:
                    out.append(r * 2000)
    return out


def z4_window_arcs(frames):
    """All zone-4 window arcs as (xs, dias) numpy arrays across frames.
    xs = cross-deck position [m], dias = arc diameter [mm]."""
    win = PRESENCE_WINDOWS[DIA_ZONE]
    xs, dd = [], []
    for segs in frames.values():
        for seg in segs:
            for lp in zone_line_points(seg, win):
                for run, r, c3 in round_runs(lp):
                    if R_LO <= r <= R_HI:
                        xs.append(float(c3[0]))
                        dd.append(r * 2000)
    return np.asarray(xs), np.asarray(dd)


_BG = None


def _load_bg():
    global _BG
    if _BG is None:
        p = Path(__file__).resolve().parent / "z4captures" / "z4_bg.npz"
        if not p.exists():
            _BG = False
        else:
            d = np.load(p)
            struct = {tuple(int(x) for x in k): float(v)
                      for k, v in zip(d["struct_keys"], d["struct_vals"])}
            _BG = {"normal": d["normal"], "offset": float(d["offset"]),
                   "bin": float(d["bin"]),
                   "win": {"x": tuple(d["win_x"]), "y": tuple(d["win_y"])},
                   "struct": struct}
            if "band_p90" in d:
                _BG["band_p90"] = {tuple(int(x) for x in k): float(v)
                                   for k, v in zip(d["band_keys"],
                                                   d["band_p90"])}
    return _BG


def z4_height_features(frames):
    """Height-above-deck features for the zone-4 log, measured ONLY in cells
    the empty background leaves structure-free (far-side band + rail + flat
    member excluded). Needs z4_bg.npz (build with lidar/z4_bg.py). Returns
    dict or None."""
    bg = _load_bg()
    if not bg:
        return None
    nrm, off, BIN = bg["normal"], bg["offset"], bg["bin"]
    struct, win = bg["struct"], bg["win"]
    P = []
    for segs in frames.values():
        for s in segs:
            for lp in zone_line_points(s, win):
                P.append(lp)
    if not P:
        return None
    P = np.vstack(P)
    r = P @ nrm + off                      # height above deck plane [m]
    cells = defaultdict(list)
    for (x, y), rr in zip(P[:, :2], r):
        cells[(round(x / BIN), round(y / BIN))].append(rr)
    # log cells: rise >8cm above deck AND structure-free when empty
    log = {}
    for k, v in cells.items():
        hi = np.percentile(v, 95)
        if hi > 0.08 and struct.get(k, -9) < 0.08:
            log[k] = hi
    if len(log) < 3:
        return {"h_max": -1, "h_p90": -1, "h_med": -1, "ncell": len(log),
                "fx": 0.0, "fy": 0.0}
    hs = np.array(list(log.values()))
    ks = np.array(list(log.keys())) * BIN
    return {"h_max": int(round(hs.max() * 1000)),
            "h_p90": int(round(np.percentile(hs, 90) * 1000)),
            "h_med": int(round(np.median(hs) * 1000)),
            "ncell": len(log),
            "fx": round(float(np.ptp(ks[:, 0])), 2),
            "fy": round(float(np.ptp(ks[:, 1])), 2)}


def z4_band_crown(frames, margin=0.04):
    """Per-band crown: for each scan band, the max height-above-deck; a band
    is 'log' only where it exceeds the empty screw-ceiling (empty p90) by
    `margin`. Returns the highest such band height (= log crown above screw)
    + how many bands the log lifts. Works for BIG logs (rise above screw);
    small logs sit within the screw envelope (complementary to circle-fit)."""
    bg = _load_bg()
    if not bg or "band_p90" not in bg:
        return None
    from lidar.z4_bg import band_heights
    nrm, off = bg["normal"], bg["offset"]
    eb = bg["band_p90"]                       # {(mi,li): p90 height}
    lb = band_heights(frames, nrm, off)       # {(mi,li): [per-frame max]}
    reached = []
    for g, hs in lb.items():
        loaded = float(np.percentile(hs, 90))
        ceil = eb.get(g, -9)
        if loaded - ceil > margin:
            reached.append((loaded, g, loaded - ceil))
    if not reached:
        return {"crown": -1, "nband": 0, "excess": 0}
    reached.sort(reverse=True)
    return {"crown": int(round(reached[0][0] * 1000)),
            "nband": len(reached),
            "excess": int(round(reached[0][2] * 1000))}


def z4_estimators(frames, xbin=0.15, min_bin=2):
    """Candidate zone-4 diameter estimators from the window arcs. The log is
    scanned from above, so partial/grazing arcs UNDER-read; the upper envelope
    tracks true size better than the mean. Returns dict or None."""
    xs, dd = z4_window_arcs(frames)
    if len(dd) < 8:
        return None
    out = {"n": int(len(dd)),
           "mean": int(round(dd.mean())),
           "p50": int(round(np.percentile(dd, 50))),
           "p90": int(round(np.percentile(dd, 90))),
           "p95": int(round(np.percentile(dd, 95)))}
    bins = np.arange(xs.min(), xs.max() + xbin, xbin)
    binmax = []
    for b in range(1, len(bins)):
        m = (xs >= bins[b - 1]) & (xs < bins[b])
        if m.sum() >= min_bin:
            binmax.append(dd[m].max())
    out["binmax_med"] = int(round(np.median(binmax))) if binmax else -1
    return out


def measure(frames):
    nfr = max(len(frames), 1)
    pres = {}
    for z, win in PRESENCE_WINDOWS.items():
        hits = 0
        dias = []
        for segs in frames.values():
            d = frame_zone_arcs(segs, win)
            if len(d) >= MIN_ARCS_FRAME:
                hits += 1
                dias.extend(d)
        pres[z] = (hits / nfr >= PRESENT_FRAC, hits, nfr,
                   round(float(np.median(dias))) if dias else -1)
    # zone-4 diameter: PREFER detect_logs (accurate, optimizer-validated);
    # fall back to tight-window arcs (reliable but reads a bit low) only when
    # detect_logs drops the log. z3 object (cam vs log) also from detect_logs.
    ax, ad = [], []
    z3obj = []
    for segs in frames.values():
        for lg in detect_logs(segs):
            if lg["zone"] == DIA_ZONE:
                ax.extend(lg.get("arc_x", []))
                ad.extend(lg.get("arc_dia", []))
            if lg["zone"] == "3_CamStaging":
                z3obj.append((lg["dia_arc_mm"], lg["height_mm"],
                              round(lg["center"][1], 2)))
    method = "detect"
    if len(ax) < 20:
        method = "window"
        ax, ad = [], []
        for segs in frames.values():
            for seg in segs:
                for lp in zone_line_points(seg, PRESENCE_WINDOWS[DIA_ZONE]):
                    for run, r, c3 in round_runs(lp):
                        if R_LO <= r <= R_HI:
                            ax.append(round(float(c3[0]), 2))
                            ad.append(int(round(r * 2000)))
    dia4 = taper_from_arcs(ax, ad) if len(ax) >= 20 else None
    if dia4:
        dia4["method"] = method
    return pres, dia4, z3obj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--duration", type=float, default=3600)
    args = ap.parse_args()

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    t0 = time.time()
    last = False
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"watching '{args.tag}' -- presence 2-7 (tight windows) + "
              f"cam_pos + z4 dia ...", flush=True)
        while time.time() - t0 < args.duration:
            r = plc.Read(args.tag)
            if r.Status != "Success":
                print("read failed:", r.Status, flush=True); time.sleep(2); continue
            v = bool(r.Value)
            if v and not last:
                n += 1
                cr = plc.Read(CAM_TAG)
                cam = round(float(cr.Value), 1) if cr.Status == "Success" else -1
                tb = time.time()
                frames = {}
                while time.time() - tb < args.seconds:
                    fn, segs = buf.latest_complete()
                    if segs and fn not in frames:
                        frames[fn] = segs
                    time.sleep(0.02)
                pres, dia4, z3obj = measure(frames)
                print(f"\n[{time.strftime('%H:%M:%S')}] PROBE #{n}  cam_pos={cam} "
                      f"({len(frames)} frames)", flush=True)
                for z in PRESENCE_WINDOWS:
                    p, hits, nfr, d = pres[z]
                    print(f"  {z:<14} {'PRESENT' if p else '  -    '} "
                          f"({hits}/{nfr} frames, dia~{d})", flush=True)
                if z3obj:
                    a = np.array(z3obj, float)
                    print(f"  z3 object: dia={np.median(a[:,0]):.0f} "
                          f"height={np.median(a[:,1]):.0f} y={np.median(a[:,2]):+.2f} "
                          f"(cam~170/65/y-1.09)", flush=True)
                if dia4:
                    print(f"  z4 DIAMETER: circ={dia4['avg']} sed={dia4['sed']} "
                          f"led={dia4['led']} arcs={dia4['n_arcs']}", flush=True)
                if args.ack:
                    w = plc.Write(args.tag, 0)
                    print(f"  (tag cleared {w.Status})", flush=True)
            last = v
            time.sleep(0.2)


if __name__ == "__main__":
    main()
