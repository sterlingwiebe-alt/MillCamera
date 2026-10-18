"""Zone analysis for the log deck: piece segmentation, diameter, skew.

Zones are axis-aligned bands in SENSOR-frame x/y (no global deck plane --
the mount will move; zones get re-marked, math stays). Each zone is
analyzed from a top-down height grid:

  pieces   = connected components rising above the zone's local surface
  diameter = crown height above local surface, cross-checked with the
             component's width across its own axis
  skew     = angle between the piece axis (PCA) and the zone's reference
             axis
"""

from __future__ import annotations

import numpy as np

CELL = 0.025


def zone_grid(pts, zone):
    """Height grid (max z per cell) for points inside the zone band."""
    (x0, x1), (y0, y1) = zone["x"], zone["y"]
    sel = ((pts[:, 0] >= x0) & (pts[:, 0] <= x1) &
           (pts[:, 1] >= y0) & (pts[:, 1] <= y1))
    p = pts[sel]
    w = int((x1 - x0) / CELL) + 1
    h = int((y1 - y0) / CELL) + 1
    img = np.full((h, w), np.nan, dtype=np.float32)
    if len(p) == 0:
        return img, p
    ix = np.clip(((p[:, 0] - x0) / CELL).astype(int), 0, w - 1)
    iy = np.clip(((p[:, 1] - y0) / CELL).astype(int), 0, h - 1)
    for x, y, z in zip(ix, iy, p[:, 2]):
        if np.isnan(img[y, x]) or z > img[y, x]:
            img[y, x] = z
    return img, p


def analyze_zone(pts, zone, rise: float = 0.06, min_cells: int = 25):
    """Segment pieces in a zone and measure them.

    Returns dict with per-piece records:
      diameter_h  crown height above local surface [m]
      diameter_w  width across the piece axis [m]
      axis_deg    piece axis angle in the x/y plane [deg]
      length      extent along the piece axis [m]
      center      (x, y) in sensor frame
    """
    import cv2
    img, p = zone_grid(pts, zone)
    valid = ~np.isnan(img)
    if valid.sum() < min_cells:
        return {"pieces": [], "coverage": 0.0}
    # local surface = low percentile of the zone's own heights
    surface = np.nanpercentile(img, 25)
    mask = ((img - surface) > rise) & valid
    mask_u8 = mask.astype(np.uint8)
    mask_u8 = cv2.morphologyEx(mask_u8, cv2.MORPH_CLOSE,
                               np.ones((3, 3), np.uint8))
    n, labels, stats, _ = cv2.connectedComponentsWithStats(mask_u8)
    pieces = []
    for i in range(1, n):
        if stats[i, cv2.CC_STAT_AREA] < min_cells:
            continue
        ys, xs = np.where(labels == i)
        zs = img[ys, xs]
        # local reference: median height of the ring of valid surface cells
        # immediately around this piece (its own support, not the zone's
        # global low -- decks here have multiple levels)
        comp = (labels == i).astype(np.uint8)
        ring = (cv2.dilate(comp, np.ones((11, 11), np.uint8)) > 0) \
            & (comp == 0) & valid
        local_ref = float(np.nanmedian(img[ring])) if ring.any() else surface
        # cell coords -> meters in sensor frame
        mx = zone["x"][0] + xs * CELL
        my = zone["y"][0] + ys * CELL
        # piece axis by PCA over cell positions
        coords = np.column_stack((mx, my))
        c = coords.mean(axis=0)
        q = coords - c
        cov = q.T @ q / len(q)
        evals, evecs = np.linalg.eigh(cov)
        axis = evecs[:, -1]                     # major axis
        along = q @ axis
        across = q @ evecs[:, 0]
        crown = np.nanpercentile(zs, 95)
        pieces.append({
            "center": (round(float(c[0]), 3), round(float(c[1]), 3)),
            "diameter_h": round(float(crown - local_ref), 3),
            # side-on viewing: the piece's own vertical extent IS the
            # visible face height ~ diameter (robust when the sensor looks
            # across the deck rather than down at it)
            "diameter_z": round(float(np.nanpercentile(zs, 97)
                                      - np.nanpercentile(zs, 3)), 3),
            "local_ref_z": round(local_ref, 3),
            "diameter_w": round(float(np.percentile(across, 97)
                                      - np.percentile(across, 3)), 3),
            "length": round(float(along.max() - along.min()), 3),
            "axis_deg": round(float(np.degrees(
                np.arctan2(axis[1], axis[0]))) % 180, 1),
            "cells": int(stats[i, cv2.CC_STAT_AREA]),
            "crown_z": round(float(crown), 3),
        })
    pieces.sort(key=lambda r: r["center"][1], reverse=True)
    return {"pieces": pieces, "surface_z": round(float(surface), 3),
            "coverage": round(float(mask.sum()) / valid.sum(), 3)}


DEFAULT_ZONES = {
    # sensor-frame bands. Flow: outfeed (low y) -> infeed (high y). Zones 3
    # & 4 are the diameter-measurement zones; 4b (PreScrew) is a new upstream
    # vantage added 2026-10-13 for a cleaner, less screw-occluded read.
    #
    # NARROWED + RE-CENTERED 2026-10-13 on the actual dwell peaks from 5 min
    # of operation (where logs pause): z3 ~-1.37, z4 ~-0.62, 4b ~-0.02,
    # z5 ~+0.57. Tight bands with don't-care gaps between so a log is
    # measured only when parked at its station, not mid-transition (the wide
    # bands were catching transitioning logs -> short/under-read zone 4).
    # RENUMBERED 2026-10-13: new zone inserted as 5, old 5->6, old 6->7
    # (7 zones total). Flow infeed(7)->outfeed(1).
    "7_InfeedDeck":      {"x": (-6.3, -1.3), "y": (1.60, 3.17)},
    "6_ScrewDeck":       {"x": (-6.3, -1.3), "y": (0.35, 1.45)},
    "5_MidScrew":        {"x": (-6.3, -1.3), "y": (-0.20, 0.15)},   # NEW
    "4_TopOfScrew":      {"x": (-6.3, -1.3), "y": (-0.88, -0.42)},
    "3_CamStaging":      {"x": (-6.3, -1.3), "y": (-1.58, -1.15)},
    "2_CamWaiting":      {"x": (-6.3, -1.3), "y": (-1.93, -1.59)},
    "1_OutfeedConveyor": {"x": (-6.3, -1.3), "y": (-2.72, -1.93)},
}

# The sensor has drifted ~0.15 m up-deck from where these bands were set
# (mount jitter/settling), so a measured log reads ~0.15 m high. Subtract
# this from a measured y before the band lookup to re-align attribution.
# PROVISIONAL (from snapshot 094813 refs); tune against the live deck.
SENSOR_Y_OFFSET = 0.15
