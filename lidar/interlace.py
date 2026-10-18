"""Interlaced-frame pooling for the multiScan165 (x4 interlacing, on since
2026-10-15).

In interlaced mode each 20 Hz frame still carries 12 segments x 16 layers x
60 beams at 0.505 deg, but the azimuth grid steps through a 4-frame cycle
in 0.126 deg increments. Pooling 4 consecutive frames therefore gives one
"dense frame" at ~0.125 deg -- 4x the beams per arc -- PROVIDED the scene
did not move during those 200 ms (use on a STOPPED log only).

pool_frames() merges consecutive frames into synthetic segments with the
same structure the rest of the pipeline expects (Modules / SegmentData /
ChannelTheta / Distance / Rssi), so detect_logs, zone_line_points etc. run
unchanged on the result.

Gate scaling: arcfit.MIN_RUN (beams per run) is a per-frame constant;
pooled lines are k times denser, so use `scaled_gates(k)` around pooled
analysis to keep the same physical run length.
"""
from __future__ import annotations

import copy
from contextlib import contextmanager

import numpy as np

from . import arcfit

CYCLE = 4   # frames per interlace cycle (x4)


def cycle_phase(seg) -> int:
    """0..3 phase of this segment's azimuth grid (from layer-0 theta)."""
    th = float(np.degrees(seg["Modules"][0]["SegmentData"][0]["ChannelTheta"][0]))
    return int(round((th % 0.505) / 0.505 * CYCLE)) % CYCLE


def is_interlaced(frames: dict) -> bool:
    """True if consecutive frames show more than one azimuth phase (the
    sensor's interlaced mode is on). With it off, pooling is meaningless."""
    phases = set()
    for segs in list(frames.values())[:CYCLE + 1]:
        if segs:
            phases.add(cycle_phase(segs[0]))
    return len(phases) > 1


def pool_frames(frame_list):
    """Merge a list of frames (each a list of segs) into one synthetic
    frame: one seg per SegmentCounter, each line = all frames' beams for
    that layer, sorted by theta."""
    by_cnt = {}
    for segs in frame_list:
        for seg in segs:
            cnt = seg["Modules"][0]["SegmentCounter"]
            by_cnt.setdefault(cnt, []).append(seg)
    out = []
    for cnt in sorted(by_cnt):
        group = by_cnt[cnt]
        base = copy.copy(group[0])
        mods_out = []
        for mi, mod0 in enumerate(group[0]["Modules"]):
            mod = copy.copy(mod0)
            lines_out = []
            for li in range(len(mod0["SegmentData"])):
                parts = [g["Modules"][mi]["SegmentData"][li] for g in group
                         if mi < len(g["Modules"])
                         and li < len(g["Modules"][mi]["SegmentData"])]
                th = np.concatenate([np.asarray(p["ChannelTheta"], float)
                                     for p in parts])
                order = np.argsort(th)
                n_echo = len(parts[0]["Distance"])
                line = {
                    "ChannelTheta": th[order],
                    "Distance": [np.concatenate([np.asarray(p["Distance"][e], float)
                                                 for p in parts])[order]
                                 for e in range(n_echo)],
                    "Rssi": [np.concatenate([np.asarray(p["Rssi"][e], float)
                                             for p in parts])[order]
                             for e in range(n_echo)],
                }
                if "Properties" in parts[0]:
                    try:
                        line["Properties"] = np.concatenate(
                            [np.asarray(p["Properties"]) for p in parts])[order]
                    except Exception:
                        pass
                lines_out.append(line)
            mod["SegmentData"] = lines_out
            mod["NumberOfBeamsPerScan"] = int(mod0.get("NumberOfBeamsPerScan", 60)) * len(group)
            mods_out.append(mod)
        base["Modules"] = mods_out
        base["_pooled"] = len(group)
        out.append(base)
    return out


def cycles(frames: dict, k: int = CYCLE):
    """Yield (first_fn, pooled_segs) for each run of k consecutive frame
    numbers in an ordered {fn: segs} dict (non-overlapping)."""
    keys = sorted(frames)
    i = 0
    while i + k <= len(keys):
        run = keys[i:i + k]
        if run[-1] - run[0] == k - 1:
            yield run[0], pool_frames([frames[f] for f in run])
            i += k
        else:
            i += 1


POOLED_FIT_RMS_MAX = 0.022   # [m] roundness gate for dense (pooled) arcs


@contextmanager
def scaled_gates(k: int = CYCLE):
    """Temporarily adapt arcfit's gates to k-times denser lines:
    MIN_RUN x k (same physical arc length) and a looser circle-fit rms.
    A 7-beam arc fits bark with ~8 mm rms only because it has few residual
    degrees of freedom; the same arc at 30 beams shows the true ~12-20 mm
    surface roughness, and the sparse-tuned 13 mm gate then rejects every
    pooled arc (verified on cap_01_041611: 0/7 cycles at 13 mm, 7/7 at
    22 mm, with no per-phase bias between the interlaced sub-grids)."""
    old_run, old_rms = arcfit.MIN_RUN, arcfit.FIT_RMS_MAX
    arcfit.MIN_RUN = old_run * k
    if k > 1:
        arcfit.FIT_RMS_MAX = POOLED_FIT_RMS_MAX
    try:
        yield
    finally:
        arcfit.MIN_RUN, arcfit.FIT_RMS_MAX = old_run, old_rms
