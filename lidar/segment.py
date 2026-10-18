"""Per-log segmentation: cluster arc centers into INDIVIDUAL logs within a
single frame, so each log can be judged on its own (its axis vs neighbors
for skew, its center for zone assignment). Intra-frame, so mount jitter
does not corrupt it.

A log = a set of round-run arc centers that are collinear over a real
length and share one radius. Sequential RANSAC pulls out one log at a
time (handles parallel logs by their offset, and a skewed log by its
different axis direction). Machinery (short/curved) fails the length gate.
"""
from __future__ import annotations

import numpy as np

from .arcfit import zone_round_runs
from .zones import DEFAULT_ZONES, SENSOR_Y_OFFSET

DECK = {"x": (-6.3, -1.3), "y": (-2.72, 3.17)}   # full deck region

MIN_ARCS = 6            # arcs to call a log
LINE_TOL = 0.05         # center-to-axis inlier distance [m]
MIN_PAIR = 0.4          # RANSAC hypothesis pair separation [m]
MIN_LEN = 0.8           # a real log is long [m] (rejects machinery). Lowered
                        # 1.0->0.8 (2026-10-13): zone 4 (top-of-screw) logs are
                        # partially occluded by the screw trough, so only a
                        # ~0.9 m slice is visible -- 1.0 was dropping legit
                        # zone-4 logs. Screw itself still rejected by axis.
DEVIANT_MAX = 40.0      # axis deviation from cross-deck (0/180) above this =
                        # screw/auger flight (helix), not a log. A skewed-log
                        # anomaly stays under this; the screw sits at 42-81 deg.
DIA_MAX = 0.38          # logs bigger than this are machinery (screw flights
                        # fit ~0.39-0.46 m; real logs here <=0.30 m). Tunable
                        # against the optimizer truth once correlated.
R_TOL = 0.03            # per-arc radius agreement [m]
R_FRAC = 0.5            # fraction of inlier arcs that must agree
RANSAC_ITERS = 300
RANSAC_CHUNK = 32       # hypothesis pairs evaluated per vectorised chunk
MAX_LOGS = 15

# RANSAC hypothesis pairs. The per-iteration rng.choice(n, 2, replace=False)
# draw costs ~18 us on the mill PC (5.6 ms per 300-iteration round, several
# rounds per frame), far more than the vectorised geometry it feeds.
# _pairs_vectorised reproduces the SAME 300 pairs from the generator's raw
# 64-bit stream in ~0.2 ms and leaves the generator in the same state, so
# detect_logs' results are unchanged. It encodes what numpy's Generator
# does for size-2 draws without replacement (Floyd's algorithm with 32-bit
# Lemire bounded draws, then a 2-element shuffle via a Lemire draw of range
# 1), validated against rng.choice on 1500 random states; a self-test at
# first use disables it if a numpy upgrade ever changes that algorithm.
_M32 = np.uint64(0xFFFFFFFF)
_PAIRS_FAST_OK = None       # None = untested, True/False after _pairs_selftest


def _pairs_loop(rng, n):
    pairs = np.empty((RANSAC_ITERS, 2), dtype=np.int64)
    for k in range(RANSAC_ITERS):
        pairs[k] = rng.choice(n, 2, replace=False)
    return pairs


def _pairs_vectorised(rng, n):
    """RANSAC_ITERS x rng.choice(n, 2, replace=False) in one go, or None
    when the fast path does not apply (n < 3, or a Lemire rejection
    occurred, ~1e-7 per draw); the generator is then left untouched."""
    if n < 3:
        return None
    bg = rng.bit_generator
    st = bg.state
    need = 3 * RANSAC_ITERS                      # 2 picks + 1 shuffle word each
    buffered = st["has_uint32"]
    n_raw = (need - buffered + 1) // 2
    raws = bg.random_raw(n_raw).astype(np.uint64)
    u32 = np.empty(2 * n_raw + buffered, dtype=np.uint64)
    if buffered:
        u32[0] = st["uinteger"]
    u32[buffered::2] = raws & _M32
    u32[buffered + 1::2] = raws >> np.uint64(32)
    u32 = u32[:need].reshape(RANSAC_ITERS, 3)
    out = np.empty((RANSAC_ITERS, 2), dtype=np.int64)
    for col, j in ((0, n - 2), (1, n - 1)):      # Floyd: j = n-2, n-1
        m = u32[:, col] * np.uint64(j + 1)
        threshold = np.uint64((0xFFFFFFFF - j) % (j + 1))
        if np.any((m & _M32) < threshold):
            bg.state = st                        # rewind, let the loop do it
            return None
        out[:, col] = (m >> np.uint64(32)).astype(np.int64)
    coll = out[:, 1] == out[:, 0]
    out[coll, 1] = n - 1                         # second pick collided: use j
    swap = (u32[:, 2] >> np.uint64(31)) == 0     # shuffle draw of range 1
    tmp = out[swap, 0].copy()
    out[swap, 0] = out[swap, 1]
    out[swap, 1] = tmp
    used = need - buffered
    st2 = bg.state
    st2["has_uint32"] = used % 2
    st2["uinteger"] = int(raws[-1] >> np.uint64(32))
    bg.state = st2
    return out


def _pairs_selftest():
    for seed, n, pre in ((0, 7, 0), (0, 41, 1), (12345, 230, 2), (7, 12, 3)):
        r1, r2 = np.random.default_rng(seed), np.random.default_rng(seed)
        for _ in range(pre):
            r1.choice(50, 2, replace=False); r2.choice(50, 2, replace=False)
        fast = _pairs_vectorised(r1, n)
        if fast is None or not np.array_equal(fast, _pairs_loop(r2, n)) \
                or r1.bit_generator.state != r2.bit_generator.state:
            return False
    return True


def _ransac_pairs(rng, n):
    """The RANSAC_ITERS (a, b) index pairs that successive
    rng.choice(n, 2, replace=False) calls draw from rng's current state."""
    global _PAIRS_FAST_OK
    if _PAIRS_FAST_OK is None:
        _PAIRS_FAST_OK = _pairs_selftest()
    if _PAIRS_FAST_OK:
        pairs = _pairs_vectorised(rng, n)
        if pairs is not None:
            return pairs
    return _pairs_loop(rng, n)


def _zone_of(center):
    """Which DEFAULT_ZONE contains this xy center (by y band); else None.
    Applies SENSOR_Y_OFFSET to correct sensor drift."""
    x, y = center[0], center[1] - SENSOR_Y_OFFSET
    for name, z in DEFAULT_ZONES.items():
        (x0, x1), (y0, y1) = z["x"], z["y"]
        if x0 <= x <= x1 and y0 <= y <= y1:
            return name
    return None


def detect_logs(segs, region=DECK, debug=False):
    """Return a list of per-log dicts: center, axis, angle_deg, length_m,
    dia_mm, n_arcs, zone. With debug=True, returns (logs, rejects) where
    rejects carry a 'reason'."""
    centers, radii, runs = [], [], []
    for run, r, c3 in zone_round_runs(segs, region):     # batched circle fits
        centers.append(c3); radii.append(r); runs.append(run)
    if len(centers) < MIN_ARCS:
        return ([], []) if debug else []
    C = np.vstack(centers)
    R = np.asarray(radii)
    used = np.zeros(len(C), bool)
    rng = np.random.default_rng(0)
    logs, rejects = [], []
    while (~used).sum() >= MIN_ARCS and len(logs) < MAX_LOGS:
        idx = np.where(~used)[0]
        Cr = C[idx]
        # RANSAC over all RANSAC_ITERS hypothesis pairs at once (same pairs,
        # same per-element arithmetic as the old one-pair-per-iteration
        # loop; the first pair reaching the max inlier count wins, as the
        # loop's strict '>' did). Pairs closer than MIN_PAIR are skipped.
        pairs = _ransac_pairs(rng, len(idx))
        a, b = pairs[:, 0], pairs[:, 1]
        D = Cr[b] - Cr[a]
        L = np.linalg.norm(D, axis=1)
        okp = L >= MIN_PAIR
        if not okp.any():
            break
        a = a[okp]
        d = D[okp] / L[okp][:, None]                     # (m, 3) unit axes
        # in chunks of pairs so the (chunk, n, 3) temporaries stay in cache,
        # written into preallocated buffers; same arithmetic per element
        # (batched matmul = BLAS gemv per pair, like `q @ d`; norm = sqrt of
        # the summed squares, like np.linalg.norm(axis=1))
        m, n = len(a), len(Cr)
        CH = RANSAC_CHUNK
        q = np.empty((min(CH, m), n, 3))
        t = np.empty_like(q)
        best_inl, best = None, 0
        for s in range(0, m, CH):
            e = min(s + CH, m)
            qk, tk = q[:e - s], t[:e - s]
            np.subtract(Cr[None, :, :], Cr[a[s:e]][:, None, :], out=qk)   # q = Cr - Cr[a]
            dk = d[s:e]
            proj = np.matmul(qk, dk[:, :, None])[:, :, 0]                  # q @ d
            np.multiply(proj[:, :, None], dk[:, None, :], out=tk)          # outer(q@d, d)
            np.subtract(qk, tk, out=tk)
            np.multiply(tk, tk, out=tk)
            ss = np.add.reduce(tk, axis=2)
            inl = np.sqrt(ss, out=ss) < LINE_TOL
            counts = inl.sum(axis=1)
            j = int(np.argmax(counts))
            if counts[j] > best:                 # first pair reaching the max wins
                best, best_inl = int(counts[j]), inl[j].copy()
        if best_inl is None or best < MIN_ARCS:
            break
        sel = idx[best_inl]
        c = C[sel].mean(0)
        q = C[sel] - c
        axis = np.linalg.svd(q, full_matrices=False)[2][0]
        proj = (C[sel] - c) @ axis
        length = float(proj.max() - proj.min())
        r_in = R[sel]
        cons = float((np.abs(r_in - np.median(r_in)) < R_TOL).mean())
        used[sel] = True            # consume regardless (avoid reloop)
        P = np.vstack([runs[i] for i in sel])
        radial = (P - c) - np.outer((P - c) @ axis, axis)
        dia = 2 * float(np.median(np.linalg.norm(radial, axis=1)))
        # alt estimate: median of the per-arc fitted circle radii -- reads
        # truer than the axis-spread when only the log's top cap is visible
        dia_arc = 2 * float(np.median(r_in))
        # alt measures to compare against the optimizer: horizontal
        # across-axis WIDTH (chord) and vertical FACE HEIGHT (z-extent) of
        # the run points -- robustness differs under screw occlusion.
        zup = np.array([0.0, 0.0, 1.0])
        perp = np.cross(axis, zup)
        pn = float(np.linalg.norm(perp))
        if pn > 1e-6:
            hp = P @ (perp / pn)             # horizontal across-axis coord
            width = float(np.ptp(hp))
            # crown-doubled diameter (handles one-sided screw occlusion): the
            # crown (top of the arc) sits over the circle's horizontal center,
            # so crown-to-a-visible-edge is one radius. Double the LARGER half
            # (the un-occluded side reaches a full radius; the screw side is
            # truncated) -- works whichever side the screw blocks.
            ztop = np.percentile(P[:, 2], 80)
            crown_h = float(np.median(hp[P[:, 2] >= ztop]))
            # 2nd/98th percentile edges (not absolute min/max) so one stray
            # point on the clean side can't double into a false radius
            lo_e, hi_e = np.percentile(hp, 2), np.percentile(hp, 98)
            dia_crown = 2.0 * max(crown_h - float(lo_e),
                                  float(hi_e) - crown_h)
        else:
            width = dia_crown = 0.0
        # vertical extent WITHIN the cross-section (radial already removed the
        # along-axis/length-wise trend, so this isn't the deck tilt)
        height = float(np.ptp(radial[:, 2]))
        ang = float(np.degrees(np.arctan2(axis[1], axis[0])) % 180)
        rec = {
            "center": [round(float(v), 2) for v in c],
            "axis": [round(float(v), 3) for v in axis],
            "angle_deg": round(ang, 1),
            "length_m": round(length, 2),
            "dia_mm": int(round(dia * 1000)),
            "dia_arc_mm": int(round(dia_arc * 1000)),
            "width_mm": int(round(width * 1000)),
            "dia_crown_mm": int(round(dia_crown * 1000)),
            "height_mm": int(round(height * 1000)),
            "n_arcs": int(best),
            "zone": _zone_of(c),
            # per-arc (x position along deck, diameter mm) for taper fit
            "arc_x": [round(float(v), 2) for v in C[sel][:, 0]],
            "arc_dia": [int(round(float(v) * 2000)) for v in R[sel]],
        }
        dev = min(ang, 180 - ang)
        reason = None
        if length < MIN_LEN:
            reason = "too_short"
        elif cons < R_FRAC:
            reason = "radius_inconsistent"
        elif dia > DIA_MAX:
            reason = "too_fat(machinery)"
        elif dev > DEVIANT_MAX:
            reason = "deviant_axis(screw)"
        if reason:
            rec["reason"] = reason
            rejects.append(rec)
            continue
        logs.append(rec)
    return (logs, rejects) if debug else logs


CAM_ZONES = {"2_CamWaiting", "3_CamStaging"}
CAM_MIN_ARCS = 10       # a log on raised cams gives a dense cluster...
CAM_AXIS_TOL = 15.0     # ...on a near cross-deck axis (0/180); the cam
                        # itself is sparse and/or deviant. (A/B 080503 vs
                        # 081941; tune with more samples.)


def zone_presence(logs):
    """Per-zone presence from per-log centers. A log counts for the zone
    its center is in. detect_logs already rejects the screw (deviant axis)
    and oversized machinery; the only machinery that still passes is the
    cam (a round lobe indistinguishable from a round log by shape -- proven
    by the axis/arc-count/continuity tests), which is handled by the PLC
    cam-position gate, not here. So presence is simply: any detected log in
    the zone."""
    present = {z: False for z in DEFAULT_ZONES}
    for lg in logs:
        if lg["zone"] is not None:
            present[lg["zone"]] = True
    return present


def flag_skew(logs, tol_deg=20.0):
    """Mark logs whose axis deviates from the frame's dominant axis.
    Dominant = circular-ish median of log angles (mod 180)."""
    if len(logs) < 2:
        return logs
    angs = np.array([lg["angle_deg"] for lg in logs])
    # dominant axis via doubled-angle mean (handles 0/180 wraparound)
    rad = np.radians(2 * angs)
    dom = np.degrees(np.arctan2(np.sin(rad).mean(),
                                np.cos(rad).mean())) / 2 % 180
    for lg in logs:
        d = abs(lg["angle_deg"] - dom)
        d = min(d, 180 - d)
        lg["skew_dev_deg"] = round(d, 1)
        lg["skewed"] = bool(d > tol_deg)
    return logs
