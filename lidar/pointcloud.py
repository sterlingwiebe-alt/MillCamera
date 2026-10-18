"""Point-cloud assembly and rendering for multiScan Compact recordings.

Loads capture.py recordings, converts segments to XYZ points, assembles
full-revolution frames, fits the dominant ground/deck plane, and renders
top-down height maps.
"""

from __future__ import annotations

import pickle
from collections import defaultdict
from pathlib import Path

import numpy as np


def load_segments(rec_dir: str | Path):
    rec_dir = Path(rec_dir)
    for pkl in sorted(rec_dir.glob("segments_*.pkl")):
        try:
            with open(pkl, "rb") as f:
                data = pickle.load(f)
        except (EOFError, pickle.UnpicklingError):
            # a trailing file truncated by stopping the recorder mid-write;
            # skip it (the batch before it is intact)
            continue
        yield from data["segments"]


def segment_points(seg, echo: int = 0, min_dist: float = 0.05):
    """All points of one segment -> (N,3) xyz in meters + (N,) rssi."""
    pts = []
    rssis = []
    for mod in seg.get("Modules", []):
        scale = mod.get("DistanceScalingFactor", 1.0)
        phis = mod["Phi"]
        for li, line in enumerate(mod["SegmentData"]):
            theta = np.asarray(line["ChannelTheta"], dtype=np.float64)
            dist = np.asarray(line["Distance"][echo], dtype=np.float64) * scale
            rssi = np.asarray(line["Rssi"][echo], dtype=np.float64)
            d_m = dist / 1000.0          # mm -> m
            ok = d_m > min_dist
            if not np.any(ok):
                continue
            phi = float(phis[li])
            cp, sp = np.cos(phi), np.sin(phi)
            d = d_m[ok]
            th = theta[ok]
            pts.append(np.column_stack((d * cp * np.cos(th),
                                        d * cp * np.sin(th),
                                        d * sp)))
            rssis.append(rssi[ok])
    if not pts:
        return np.empty((0, 3)), np.empty((0,))
    return np.vstack(pts), np.concatenate(rssis)


def assemble_frames(rec_dir, max_frames: int = 0):
    """Group segments by module FrameNumber -> dict frame -> list[seg]."""
    frames: dict = defaultdict(list)
    for seg in load_segments(rec_dir):
        mods = seg.get("Modules", [])
        if not mods:
            continue
        fn = mods[0].get("FrameNumber", 0)
        frames[fn].append(seg)
        if max_frames and len(frames) > max_frames + 2:
            break
    # drop first/last (likely partial revolutions)
    keys = sorted(frames.keys())
    if len(keys) > 2:
        keys = keys[1:-1]
    if max_frames:
        keys = keys[:max_frames]
    return {k: frames[k] for k in keys}


def frame_cloud(segs, echo: int = 0):
    pts = []
    rssis = []
    for seg in segs:
        p, r = segment_points(seg, echo)
        if len(p):
            pts.append(p)
            rssis.append(r)
    if not pts:
        return np.empty((0, 3)), np.empty((0,))
    return np.vstack(pts), np.concatenate(rssis)


def accumulate(rec_dir, n_frames: int = 20, echo: int = 0):
    """Average static scene: concatenate points of n complete frames."""
    frames = assemble_frames(rec_dir, max_frames=n_frames)
    clouds = []
    seg_counts = []
    for fn, segs in frames.items():
        p, _ = frame_cloud(segs, echo)
        clouds.append(p)
        seg_counts.append(len(segs))
    pts = np.vstack(clouds) if clouds else np.empty((0, 3))
    return pts, {"frames_used": len(clouds),
                 "segments_per_frame": seg_counts[:10],
                 "points": len(pts)}


def topdown_image(pts, cell: float = 0.02, z_range=None,
                  x_range=None, y_range=None):
    """Rasterize: grid in x/y, value = max z per cell. Returns image + extents."""
    import cv2
    if x_range is None:
        x_range = (np.percentile(pts[:, 0], 1), np.percentile(pts[:, 0], 99))
    if y_range is None:
        y_range = (np.percentile(pts[:, 1], 1), np.percentile(pts[:, 1], 99))
    sel = ((pts[:, 0] >= x_range[0]) & (pts[:, 0] <= x_range[1]) &
           (pts[:, 1] >= y_range[0]) & (pts[:, 1] <= y_range[1]))
    p = pts[sel]
    if z_range is not None:
        zs = (p[:, 2] >= z_range[0]) & (p[:, 2] <= z_range[1])
        p = p[zs]
    w = int((x_range[1] - x_range[0]) / cell) + 1
    h = int((y_range[1] - y_range[0]) / cell) + 1
    img = np.full((h, w), np.nan, dtype=np.float32)
    ix = ((p[:, 0] - x_range[0]) / cell).astype(int)
    iy = ((p[:, 1] - y_range[0]) / cell).astype(int)
    for x, y, z in zip(ix, iy, p[:, 2]):
        if np.isnan(img[y, x]) or z > img[y, x]:
            img[y, x] = z
    return img, x_range, y_range


def render_heightmap(img, x_range, y_range, out_path,
                     z_clip=None, title=""):
    import cv2
    v = img.copy()
    if z_clip:
        v = np.clip(v, z_clip[0], z_clip[1])
    lo = np.nanmin(v)
    hi = np.nanmax(v)
    norm = (v - lo) / max(hi - lo, 1e-6)
    norm[np.isnan(norm)] = 0
    u8 = (norm * 255).astype(np.uint8)
    color = cv2.applyColorMap(u8, cv2.COLORMAP_TURBO)
    color[np.isnan(img)] = (30, 30, 30)
    color = cv2.flip(color, 0)   # y up
    scale = max(1, int(900 / max(color.shape[:2])))
    if scale > 1:
        color = cv2.resize(color, None, fx=scale, fy=scale,
                           interpolation=cv2.INTER_NEAREST)
    cv2.putText(color, f"{title} x:[{x_range[0]:.1f},{x_range[1]:.1f}]m "
                       f"y:[{y_range[0]:.1f},{y_range[1]:.1f}]m "
                       f"z:[{lo:.2f},{hi:.2f}]m",
                (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1,
                cv2.LINE_AA)
    cv2.imwrite(str(out_path), color)
    return out_path
