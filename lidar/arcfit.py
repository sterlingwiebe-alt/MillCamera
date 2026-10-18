"""Arc/cylinder-based log measurement.

Works on the native scan-line structure instead of height blobs:

1. per layer line, find contiguous runs of beams inside the zone
2. circle-fit each run in its own plane; keep runs whose curvature radius
   is log-like (machinery is flat or sharp -> fails by physics)
3. pool the kept (round) points, PCA -> log axis
4. radius = median distance of round points to the axis; diameter = 2r
   (skew = angle between axis and the zone's flow-cross reference)
"""

from __future__ import annotations

import threading
from collections import OrderedDict

import numpy as np

from .pointcloud import load_segments

try:                                # stacked lstsq (numpy's own gufunc; the
    from numpy.linalg import _umath_linalg as _ul   # public wrapper is 2-D only)
except ImportError:                 # pragma: no cover
    _ul = None

R_MIN, R_MAX = 0.04, 0.40          # plausible log radius [m]
MAX_GAP = 0.12                     # run break on spatial jump [m] (loosened
                                   # 0.08->0.12 to keep sparse far-end runs)
MIN_RUN = 5                        # beams per run (6->5: use somewhat sparser
                                   # far-end returns for fuller coverage,
                                   # without the unstable 4-beam arc fits)
FIT_RMS_MAX = 0.013                # circle fit residual gate [m] (0.012->0.013)


def kasa_circle(xy):
    """Algebraic circle fit; returns (cx, cy, r, rms)."""
    x, y = xy[:, 0], xy[:, 1]
    A = np.column_stack((2 * x, 2 * y, np.ones(len(x))))
    b = x * x + y * y
    try:
        sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    except np.linalg.LinAlgError:
        return 0, 0, np.inf, np.inf
    cx, cy, c = sol
    r = np.sqrt(max(c + cx * cx + cy * cy, 1e-12))
    rms = float(np.sqrt(np.mean(
        (np.sqrt((x - cx) ** 2 + (y - cy) ** 2) - r) ** 2)))
    return cx, cy, r, rms


def line_runs(points):
    """Split one scan line's ordered xyz points into contiguous runs."""
    if len(points) < MIN_RUN:
        return
    jumps = np.linalg.norm(np.diff(points, axis=0), axis=1)
    cut = np.where(jumps > MAX_GAP)[0]
    start = 0
    for c in list(cut) + [len(points) - 1]:
        if c + 1 - start >= MIN_RUN:
            yield points[start:c + 1]
        start = c + 1


# Circle-fit memo: a run's fit depends only on its points, and the live
# tracker fits the same runs several times per frame (zone-4 band, deck
# inventory, the per-zone presence windows overlap). Keyed on the run's
# exact bytes, so a hit is the identical computation. Cleared when full.
_FIT_CACHE: dict = {}
_FIT_CACHE_MAX = 8192


def _fit_run(run):
    """(r, rms, center3d) of the circle fitted to one run in its own plane
    (see round_runs). Memoised on the run's exact point values."""
    key = run.tobytes()
    ent = _FIT_CACHE.get(key)
    if ent is None:
        c = run.mean(axis=0)
        q = run - c
        _, _, vt = np.linalg.svd(q, full_matrices=False)
        plane = q @ vt[:2].T          # project to run's own plane
        cx, cy, r, rms = kasa_circle(plane)
        center3d = c + cx * vt[0] + cy * vt[1]
        ent = (r, rms, center3d)
        if len(_FIT_CACHE) >= _FIT_CACHE_MAX:
            _FIT_CACHE.clear()
        _FIT_CACHE[key] = ent
    return ent


def _raise_linalg(err, flag):
    raise np.linalg.LinAlgError("SVD did not converge")


# numpy.linalg's own error handling around its LAPACK gufuncs
_LINALG_ERRSTATE = dict(call=_raise_linalg, invalid="call", over="ignore",
                        divide="ignore", under="ignore")


def _fit_group(R):
    """Circle fits for a stack R (k, n, 3) of k runs of equal length n:
    (r (k,), rms (k,), center3d (k, 3)). The same LAPACK routines numpy's
    svd / lstsq call for one run (gesdd, gelsd with rcond = eps * n) issued
    once for the stack -- per-matrix results are bit-identical -- and the
    same elementwise algebra as _fit_run / kasa_circle. Call inside
    np.errstate(**_LINALG_ERRSTATE); raises LinAlgError like numpy does."""
    k, n, _ = R.shape
    c = np.add.reduce(R, axis=1) / n                       # == run.mean(axis=0)
    q = R - c[:, None, :]
    vt = _ul.svd_s(q, signature="d->ddd")[2]               # (k, 3, 3)
    plane = q @ vt[:, :2, :].transpose(0, 2, 1)            # (k, n, 2)
    x, y = plane[..., 0], plane[..., 1]
    A = np.empty((k, n, 3))
    A[..., 0] = 2 * x
    A[..., 1] = 2 * y
    A[..., 2] = 1.0
    rcond = np.finfo(np.float64).eps * max(n, 3)
    sol = _ul.lstsq(A, (x * x + y * y)[..., None], rcond,
                    signature="ddd->ddid")[0][..., 0]      # (k, 3)
    cx, cy, cc = sol[:, 0], sol[:, 1], sol[:, 2]
    r = np.sqrt(np.maximum(cc + cx * cx + cy * cy, 1e-12))
    rms = np.sqrt(np.mean(
        (np.sqrt((x - cx[:, None]) ** 2 + (y - cy[:, None]) ** 2) - r[:, None]) ** 2,
        axis=1))
    center3d = c + cx[:, None] * vt[:, 0, :] + cy[:, None] * vt[:, 1, :]
    return r, rms, center3d


def fit_runs(runs):
    """[(r, rms, center3d)] for a list of runs: what _fit_run gives for each
    one, but the SVDs and least-squares solves are issued per run LENGTH as
    one stacked LAPACK call each (see _fit_group). Memoised."""
    out = [None] * len(runs)
    keys = [None] * len(runs)
    groups: dict = {}
    for i, run in enumerate(runs):
        k = run.tobytes()
        keys[i] = k
        ent = _FIT_CACHE.get(k)
        if ent is not None:
            out[i] = ent
        else:
            groups.setdefault(len(run), []).append(i)
    if not groups:
        return out
    if len(_FIT_CACHE) >= _FIT_CACHE_MAX:
        _FIT_CACHE.clear()
    with np.errstate(**_LINALG_ERRSTATE):
        for n, ids in groups.items():
            fits = None
            if _ul is not None:
                try:
                    r, rms, c3 = _fit_group(np.stack([runs[i] for i in ids]))
                    fits = [(r[j], float(rms[j]), c3[j]) for j in range(len(ids))]
                except np.linalg.LinAlgError:
                    fits = None                   # one bad matrix: do them singly
            if fits is None:
                fits = [_fit_run(runs[i]) for i in ids]
            for j, i in enumerate(ids):
                out[i] = fits[j]
                _FIT_CACHE[keys[i]] = fits[j]
    return out


def round_runs(points):
    """Yield (run_points, radius, center3d) for log-curvature runs.
    center3d = the fitted circle's center lifted back to 3D -- it lies on
    (or very near) the cylinder axis, regardless of surface contamination."""
    for run in line_runs(points):
        r, rms, center3d = _fit_run(run)
        if R_MIN <= r <= R_MAX and rms <= FIT_RMS_MAX:
            yield run, float(r), center3d.copy()


def zone_runs(segs, zone, echo: int = 0):
    """Every run (contiguous beams, no jump > MAX_GAP, >= MIN_RUN points)
    inside the zone over a whole frame, in the order the nested loops
    `for seg: for lp in zone_line_points(seg): for run in line_runs(lp)`
    produce them -- computed in a handful of array operations instead."""
    if not isinstance(segs, (list, tuple)):
        segs = list(segs)
    (x0, x1), (y0, y1) = zone["x"], zone["y"]
    P, starts, ends = frame_points(segs, echo)
    if len(P) == 0:
        return []
    inz = ((P[:, 0] >= x0) & (P[:, 0] <= x1) &
           (P[:, 1] >= y0) & (P[:, 1] <= y1))
    counts = np.add.reduceat(inz.astype(np.int32), starts)
    sel = np.nonzero(counts >= MIN_RUN)[0]
    if len(sel) == 0:
        return []
    lps = [P[starts[i]:ends[i]][inz[starts[i]:ends[i]]] for i in sel]
    L = np.concatenate(lps)
    bounds = np.cumsum([len(lp) for lp in lps])            # line ends in L
    jumps = np.linalg.norm(np.diff(L, axis=0), axis=1)     # same pairs as per line
    big = jumps > MAX_GAP
    big[bounds[:-1] - 1] = True                            # a line end always cuts
    runs = []
    start = 0
    for c in np.nonzero(big)[0].tolist() + [len(L) - 1]:
        if c + 1 - start >= MIN_RUN:
            runs.append(L[start:c + 1])
        start = c + 1
    return runs


def zone_round_runs(segs, zone, echo: int = 0):
    """round_runs over every line of every segment inside the zone, in the
    order `for seg: for lp in zone_line_points: for run in round_runs(lp)`
    gives, with all the circle fits of the pass done in one batch."""
    runs = zone_runs(segs, zone, echo)
    for run, (r, rms, center3d) in zip(runs, fit_runs(runs)):
        if R_MIN <= r <= R_MAX and rms <= FIT_RMS_MAX:
            yield run, float(r), center3d.copy()


def prefit_zones(segs, zones, echo: int = 0):
    """Fit the runs of several zone passes over one frame in ONE batch, so
    the per-zone zone_round_runs calls that follow hit the memo. Fewer,
    larger length groups = the per-group LAPACK/numpy call overhead paid
    once per frame instead of once per zone."""
    runs = []
    for zone in zones:
        runs.extend(zone_runs(segs, zone, echo))
    fit_runs(runs)


# Per-segment point cache: the tracker scans every frame for several zones
# (zone-4 band each frame; deck inventory + 7 presence windows every 4th),
# and converting the 16 x 60 beams of a segment to xyz is the same work each
# time. Keyed on the segment OBJECT (identity, held so the id can't be
# reused) and validated on its Modules list, so a pooled/copied segment
# never hits a stale entry. Bounded to a few frames' worth of segments.
_PTS_CACHE: "OrderedDict[tuple, tuple]" = OrderedDict()
_PTS_CACHE_MAX = 64
_PTS_LOCK = threading.Lock()


def _lines_xyz_single(mods, echo):
    """Per-line fallback of _modules_xyz (ragged lines): the original loop."""
    parts, counts = [], []
    for mod in mods:
        scale = mod.get("DistanceScalingFactor", 1.0)
        phis = mod["Phi"]
        for li, line in enumerate(mod["SegmentData"]):
            theta = np.asarray(line["ChannelTheta"], dtype=np.float64)
            dist = np.asarray(line["Distance"][echo], dtype=np.float64) * scale
            d = dist / 1000.0
            ok = d > 0.05
            if not np.any(ok):
                counts.append(0)
                continue
            phi = float(phis[li])
            cp, sp = np.cos(phi), np.sin(phi)
            parts.append(np.column_stack((d[ok] * cp * np.cos(theta[ok]),
                                          d[ok] * cp * np.sin(theta[ok]),
                                          d[ok] * sp)))
            counts.append(len(parts[-1]))
    P = np.vstack(parts) if parts else np.zeros((0, 3))
    return P, np.asarray(counts, dtype=np.intp)


def _modules_xyz(mods, echo):
    """(P, counts): xyz of every valid (> 5 cm) return of the modules as one
    (N, 3) array in module/line/beam order, and the number of valid returns
    per scan line (0 for lines without any). Vectorised over all lines of
    all modules; the arithmetic is the same per element as the old per-line
    loop (bit-identical output)."""
    try:
        theta = np.asarray([ln["ChannelTheta"] for m in mods for ln in m["SegmentData"]],
                           dtype=np.float64)
        dist = np.asarray([ln["Distance"][echo] for m in mods for ln in m["SegmentData"]],
                          dtype=np.float64)
        if theta.ndim != 2 or dist.shape != theta.shape or len(theta) == 0:
            raise ValueError
    except ValueError:                       # ragged (or no) lines: one at a time
        return _lines_xyz_single(mods, echo)
    # per-line scale factor (module's) and cos/sin(phi) as python-float
    # scalars, exactly as the per-line loop used them
    scales, cp, sp = [], [], []
    for mod in mods:
        scale = mod.get("DistanceScalingFactor", 1.0)
        phis = mod["Phi"]
        for li in range(len(mod["SegmentData"])):
            scales.append(scale)
            cp.append(np.cos(float(phis[li])))
            sp.append(np.sin(float(phis[li])))
    cp = np.array(cp)[:, None]
    sp = np.array(sp)[:, None]
    if all(s == scales[0] for s in scales):
        dist = dist * scales[0]
    else:
        dist = dist * np.array(scales, dtype=np.float64)[:, None]
    d = dist / 1000.0
    ok = d > 0.05
    xyz = np.stack(((d * cp) * np.cos(theta), (d * cp) * np.sin(theta), d * sp),
                   axis=-1)                                  # (L, B, 3)
    return xyz.reshape(-1, 3)[ok.ravel()], ok.sum(axis=1)


def seg_points(seg, echo: int = 0):
    """(P, starts, ends): all valid returns of the segment as one (N, 3)
    array, plus the [start, end) row range of each scan line that has any
    (lines without returns are dropped). Cached per segment object."""
    key = (id(seg), echo)
    mods = seg.get("Modules", [])
    with _PTS_LOCK:
        ent = _PTS_CACHE.get(key)
        if ent is not None and ent[0] is seg and ent[1] is mods:
            _PTS_CACHE.move_to_end(key)
            return ent[2]
    P, cnt = _modules_xyz(mods, echo)
    cnt = cnt[cnt > 0]                       # lines without returns are dropped
    ends = np.cumsum(cnt)
    starts = ends - cnt
    val = (P, starts.astype(np.intp), ends.astype(np.intp))
    with _PTS_LOCK:
        _PTS_CACHE[key] = (seg, mods, val)
        _PTS_CACHE.move_to_end(key)
        while len(_PTS_CACHE) > _PTS_CACHE_MAX:
            _PTS_CACHE.popitem(last=False)
    return val


def zone_line_points(seg, zone, echo: int = 0):
    """Per scan line: ordered xyz points inside the zone band."""
    (x0, x1), (y0, y1) = zone["x"], zone["y"]
    P, starts, ends = seg_points(seg, echo)
    if len(P) == 0:
        return
    inz = ((P[:, 0] >= x0) & (P[:, 0] <= x1) &
           (P[:, 1] >= y0) & (P[:, 1] <= y1))
    counts = np.add.reduceat(inz.astype(np.int32), starts)
    for li in np.nonzero(counts >= MIN_RUN)[0]:
        s, e = starts[li], ends[li]
        yield P[s:e][inz[s:e]]


# Whole-frame point cache (the 12 segments' cached points concatenated),
# keyed on the identity of the segment objects, a couple of frames deep.
_FRAME_CACHE: "OrderedDict[tuple, tuple]" = OrderedDict()
_FRAME_CACHE_MAX = 4


def frame_points(segs, echo: int = 0):
    """(P, starts, ends) for all segments of a frame: seg_points of each,
    concatenated in order, with line ranges offset accordingly."""
    key = (tuple(id(s) for s in segs), echo)
    with _PTS_LOCK:
        ent = _FRAME_CACHE.get(key)
        if ent is not None and len(ent[0]) == len(segs) \
                and all(a is b for a, b in zip(ent[0], segs)):
            _FRAME_CACHE.move_to_end(key)
            return ent[1]
    parts, st, en, off = [], [], [], 0
    for seg in segs:
        P, s, e = seg_points(seg, echo)
        parts.append(P)
        st.append(s + off)
        en.append(e + off)
        off += len(P)
    if parts:
        val = (np.vstack(parts), np.concatenate(st), np.concatenate(en))
    else:
        val = (np.zeros((0, 3)), np.zeros(0, dtype=np.intp), np.zeros(0, dtype=np.intp))
    with _PTS_LOCK:
        _FRAME_CACHE[key] = (tuple(segs), val)
        _FRAME_CACHE.move_to_end(key)
        while len(_FRAME_CACHE) > _FRAME_CACHE_MAX:
            _FRAME_CACHE.popitem(last=False)
    return val


class MachineryMask:
    """Dwell-learned machine map (lidar/build_mask.py). Arc centers that
    land in a flagged cell belong to static round steel, not logs."""

    def __init__(self, npz_path):
        import numpy as _np
        m = _np.load(npz_path)
        self.mask = m["mask"]
        self.x0 = float(m["x0"])
        self.y0 = float(m["y0"])
        self.cell = float(m["cell"])

    def hit(self, c3) -> bool:
        ix = int((c3[0] - self.x0) / self.cell)
        iy = int((c3[1] - self.y0) / self.cell)
        return (0 <= iy < self.mask.shape[0]
                and 0 <= ix < self.mask.shape[1]
                and bool(self.mask[iy, ix]))


# log-ness gates for detect_log (single frame)
DET_MIN_ARCS = 6          # arcs after masking
DET_LINE_TOL = 0.04       # center-to-axis inlier distance [m]
DET_MIN_INLIERS = 6
DET_MIN_SPAN = 0.6        # inlier centers' extent along axis [m]
DET_MIN_PAIR_SEP = 0.3    # RANSAC: hypothesis pair separation [m]
DET_MAX_PAIRS = 300
DET_R_TOL = 0.03          # per-arc radius agreement with median [m]
DET_R_FRAC = 0.6          # fraction of inliers that must agree


def _best_line(C):
    """RANSAC line through 3D centers: pairs as axis hypotheses, keep the
    one with most inliers (plain PCA collapses when >50% of the centers
    are clutter). Returns (point, axis, inlier_bool) or None."""
    n = len(C)
    ii, jj = np.triu_indices(n, k=1)
    if len(ii) > DET_MAX_PAIRS:
        pick = np.random.default_rng(0).choice(len(ii), DET_MAX_PAIRS,
                                               replace=False)
        ii, jj = ii[pick], jj[pick]
    best, best_keep = 0, None
    for a, b in zip(ii, jj):
        d = C[b] - C[a]
        L = np.linalg.norm(d)
        if L < DET_MIN_PAIR_SEP:
            continue
        axis = d / L
        q = C - C[a]
        dist = np.linalg.norm(q - np.outer(q @ axis, axis), axis=1)
        keep = dist < DET_LINE_TOL
        if keep.sum() > best:
            best, best_keep = int(keep.sum()), keep
    if best_keep is None or best < 3:
        return None
    # refine on the winning set
    c = C[best_keep].mean(axis=0)
    q = C[best_keep] - c
    _, _, vt = np.linalg.svd(q, full_matrices=False)
    axis = vt[0]
    dist = np.linalg.norm((C - c) - np.outer((C - c) @ axis, axis), axis=1)
    return c, axis, dist < DET_LINE_TOL


def detect_log(radii, pts, centers, mask=None):
    """Strict per-frame log detector: a log is a SET of arcs whose centers
    are collinear over a real length AND share one radius. Scattered
    machinery arcs fail one of the gates. Returns dict(present, arcs,
    dia_mm); dia_mm = -1 when absent."""
    none = {"present": False, "arcs": len(radii), "dia_mm": -1}
    if len(centers) < DET_MIN_ARCS:
        return none
    C = np.vstack(centers)
    R = np.asarray(radii, dtype=np.float64)
    if mask is not None:
        free = np.array([not mask.hit(c) for c in C])
        C, R = C[free], R[free]
        pts = [p for p, f in zip(pts, free) if f]
        if len(C) < DET_MIN_ARCS:
            return none
    fit = _best_line(C)
    if fit is None:
        return none
    c, axis, keep = fit
    if keep.sum() < DET_MIN_INLIERS:
        return none
    proj = (C[keep] - c) @ axis
    if proj.max() - proj.min() < DET_MIN_SPAN:
        return none
    r_in = R[keep]
    r_med = float(np.median(r_in))
    cons = np.abs(r_in - r_med) < DET_R_TOL
    if cons.mean() < DET_R_FRAC:
        return none
    # diameter from the consistent inlier arcs' points vs the axis
    idx = np.where(keep)[0][cons]
    P = np.vstack([pts[i] for i in idx])
    qp = P - c
    radial = qp - np.outer(qp @ axis, axis)
    dia = 2 * float(np.median(np.linalg.norm(radial, axis=1)))
    return {"present": True, "arcs": int(keep.sum()),
            "dia_mm": int(round(dia * 1000))}


def measure_zone(rec_dir, zone, max_frames: int = 20):
    """Cylinder measurement over a recording for one zone."""
    arc_radii = []
    round_pts = []
    centers = []
    frames_seen = set()
    for seg in load_segments(rec_dir):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        if len(frames_seen) >= max_frames and fn not in frames_seen:
            break
        frames_seen.add(fn)
        for pts in zone_line_points(seg, zone):
            for run, r, c3 in round_runs(pts):
                arc_radii.append(r)
                round_pts.append(run)
                centers.append(c3)
    if not round_pts:
        return {"present": False, "arcs": 0}
    P = np.vstack(round_pts)
    C = np.vstack(centers)
    # axis = robust line through the arc CENTERS (contamination-immune):
    # iterate PCA with outlier rejection on center-to-line distance
    keep = np.ones(len(C), dtype=bool)
    for _ in range(3):
        c = C[keep].mean(axis=0)
        q = C[keep] - c
        _, _, vt = np.linalg.svd(q, full_matrices=False)
        axis = vt[0]
        dall = np.linalg.norm((C - c) - np.outer((C - c) @ axis, axis),
                              axis=1)
        med = np.median(dall[keep])
        keep = dall < max(3 * med, 0.05)
    # radius = median point-to-axis distance of the round points
    qp = P - c
    proj = qp @ axis
    radial = qp - np.outer(proj, axis)
    r_axis = float(np.median(np.linalg.norm(radial, axis=1)))
    return {
        "present": True,
        "arcs": len(arc_radii),
        "points": len(P),
        "diameter_axis_mm": round(2 * r_axis * 1000),
        "diameter_arcfit_mm": round(2 * float(np.median(arc_radii)) * 1000),
        "arc_radius_iqr_mm": [round(float(np.percentile(arc_radii, q_)) * 2000)
                              for q_ in (25, 75)],
        "axis_vector": [round(float(v), 3) for v in axis],
        "axis_angle_xy_deg": round(float(np.degrees(
            np.arctan2(axis[1], axis[0]))) % 180, 1),
        "length_along_axis_m": round(float(proj.max() - proj.min()), 2),
        "center": [round(float(v), 2) for v in c],
    }
