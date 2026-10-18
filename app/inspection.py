"""Inspection engine: template matching and threshold zones inside ROIs.

Design notes
------------
* Template matching uses TM_CCOEFF_NORMED on grayscale, which is invariant
  to linear brightness/contrast changes -- the right default given the
  station has "somewhat consistent" lighting.
* Threshold zones support grayscale and HSV. For color checks under
  varying lighting, HSV hue+saturation bounds hold up much better than RGB.
* All coordinates are pixels in the full camera image. Match coordinates
  reported are the CENTER of the matched template.
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from typing import Optional

import cv2
import numpy as np

TEMPLATE = "template"
THRESHOLD = "threshold"


@dataclass
class Roi:
    """One inspection zone. rect = [x, y, w, h] in full-image pixels."""
    name: str
    kind: str                      # 'template' | 'threshold'
    rect: list                     # search region (template) or zone (threshold)
    enabled: bool = True

    # -- template tool -----------------------------------------------------
    template_file: str = ""        # relative to the recipe folder
    score_threshold: float = 0.80  # 0..1
    multi_match: bool = False
    max_matches: int = 5

    # -- threshold tool ----------------------------------------------------
    color_space: str = "gray"      # 'gray' | 'hsv'
    lower: list = field(default_factory=lambda: [0, 0, 0])    # gray uses [v,0,0]
    upper: list = field(default_factory=lambda: [255, 255, 255])
    min_percent: float = 50.0      # pass if min% <= in-range% <= max%
    max_percent: float = 100.0

    def to_dict(self) -> dict:
        return {
            "name": self.name, "kind": self.kind, "rect": list(self.rect),
            "enabled": self.enabled, "template_file": self.template_file,
            "score_threshold": self.score_threshold,
            "multi_match": self.multi_match, "max_matches": self.max_matches,
            "color_space": self.color_space,
            "lower": list(self.lower), "upper": list(self.upper),
            "min_percent": self.min_percent, "max_percent": self.max_percent,
        }

    @staticmethod
    def from_dict(d: dict) -> "Roi":
        return Roi(
            name=d["name"], kind=d["kind"], rect=list(d["rect"]),
            enabled=d.get("enabled", True),
            template_file=d.get("template_file", ""),
            score_threshold=d.get("score_threshold", 0.80),
            multi_match=d.get("multi_match", False),
            max_matches=d.get("max_matches", 5),
            color_space=d.get("color_space", "gray"),
            lower=list(d.get("lower", [0, 0, 0])),
            upper=list(d.get("upper", [255, 255, 255])),
            min_percent=d.get("min_percent", 50.0),
            max_percent=d.get("max_percent", 100.0),
        )


@dataclass
class Match:
    x: int              # center, full-image pixels
    y: int
    score: float


@dataclass
class RoiResult:
    name: str
    kind: str
    passed: bool
    matches: list = field(default_factory=list)   # list[Match]
    in_range_percent: float = 0.0                 # threshold tool
    detail: str = ""

    def to_dict(self) -> dict:
        return {
            "name": self.name, "kind": self.kind, "passed": self.passed,
            "matches": [{"x": m.x, "y": m.y, "score": round(m.score, 4)} for m in self.matches],
            "in_range_percent": round(self.in_range_percent, 2),
            "detail": self.detail,
        }


@dataclass
class InspectionResult:
    passed: bool
    rois: list                     # list[RoiResult]
    elapsed_ms: float
    frame_id: int = 0
    wall_time: float = 0.0
    result_id: int = 0
    source: str = ""

    def to_dict(self) -> dict:
        return {
            "passed": self.passed,
            "rois": [r.to_dict() for r in self.rois],
            "elapsed_ms": round(self.elapsed_ms, 1),
            "frame_id": self.frame_id,
            "wall_time": self.wall_time,
            "result_id": self.result_id,
            "source": self.source,
        }


def _clamp_rect(rect, w, h):
    x, y, rw, rh = [int(v) for v in rect]
    x = max(0, min(x, w - 1))
    y = max(0, min(y, h - 1))
    rw = max(1, min(rw, w - x))
    rh = max(1, min(rh, h - y))
    return x, y, rw, rh


def _nms(points: list[Match], min_dist: int) -> list[Match]:
    """Greedy non-maximum suppression by score, radius = min_dist px."""
    points = sorted(points, key=lambda m: m.score, reverse=True)
    kept: list[Match] = []
    for p in points:
        if all((p.x - k.x) ** 2 + (p.y - k.y) ** 2 >= min_dist ** 2 for k in kept):
            kept.append(p)
    return kept


def _refine(region: np.ndarray, template_gray: np.ndarray,
            cx: int, cy: int, margin: int) -> tuple[int, int, float]:
    """Full-resolution match in a small window around a coarse candidate.
    (cx, cy) is the candidate top-left in region coords. Returns refined
    top-left + score."""
    th, tw = template_gray.shape[:2]
    rh, rw = region.shape[:2]
    x0 = max(0, cx - margin)
    y0 = max(0, cy - margin)
    x1 = min(rw, cx + tw + margin)
    y1 = min(rh, cy + th + margin)
    sub = region[y0:y1, x0:x1]
    if sub.shape[0] < th or sub.shape[1] < tw:
        return cx, cy, -1.0
    scores = cv2.matchTemplate(sub, template_gray, cv2.TM_CCOEFF_NORMED)
    _, max_val, _, max_loc = cv2.minMaxLoc(scores)
    return x0 + max_loc[0], y0 + max_loc[1], float(max_val)


def run_template_roi(image_gray: np.ndarray, roi: Roi, template_gray: np.ndarray) -> RoiResult:
    h, w = image_gray.shape[:2]
    x, y, rw, rh = _clamp_rect(roi.rect, w, h)
    region = image_gray[y:y + rh, x:x + rw]
    th, tw = template_gray.shape[:2]

    if th > rh or tw > rw:
        return RoiResult(roi.name, TEMPLATE, False,
                         detail=f"template {tw}x{th} larger than search region {rw}x{rh}")

    # Coarse-to-fine pyramid: match at 1/f scale, refine full-res in a small
    # window. Cuts a 20 ms full-res search to a few ms; f capped so the
    # shrunken template keeps enough texture to be found reliably.
    f = 1
    while (f < 8 and min(th, tw) // (f * 2) >= 24
           and min(rh, rw) // (f * 2) >= 48):
        f *= 2

    matches: list[Match] = []
    best_detail = ""
    if f == 1:
        scores = cv2.matchTemplate(region, template_gray, cv2.TM_CCOEFF_NORMED)
        if roi.multi_match:
            ys, xs = np.where(scores >= roi.score_threshold)
            cand = [(int(cx), int(cy), float(scores[cy, cx]))
                    for cy, cx in zip(ys.tolist(), xs.tolist())]
        else:
            _, max_val, _, max_loc = cv2.minMaxLoc(scores)
            cand = [(max_loc[0], max_loc[1], float(max_val))]
        refined = cand
    else:
        small_region = cv2.resize(region, (max(rw // f, 1), max(rh // f, 1)),
                                  interpolation=cv2.INTER_AREA)
        small_tpl = cv2.resize(template_gray, (max(tw // f, 1), max(th // f, 1)),
                               interpolation=cv2.INTER_AREA)
        scores = cv2.matchTemplate(small_region, small_tpl, cv2.TM_CCOEFF_NORMED)
        # coarse scores differ slightly from full-res -> search leniently,
        # apply the real threshold after refinement
        coarse_thr = max(0.3, roi.score_threshold - 0.10)
        if roi.multi_match:
            ys, xs = np.where(scores >= coarse_thr)
            coarse = [Match(x=int(cx), y=int(cy), score=float(scores[cy, cx]))
                      for cy, cx in zip(ys.tolist(), xs.tolist())]
            sh, sw = small_tpl.shape[:2]
            coarse = _nms(coarse, min_dist=max(4, min(sw, sh) // 2))[: roi.max_matches * 2]
        else:
            _, max_val, _, max_loc = cv2.minMaxLoc(scores)
            coarse = [Match(x=max_loc[0], y=max_loc[1], score=float(max_val))]
        refined = []
        for c in coarse:
            rx, ry, rs = _refine(region, template_gray, c.x * f, c.y * f,
                                 margin=2 * f + 2)
            refined.append((rx, ry, rs))

    cand_matches = [Match(x=x + rx + tw // 2, y=y + ry + th // 2, score=rs)
                    for rx, ry, rs in refined]
    passing = [m for m in cand_matches if m.score >= roi.score_threshold]
    if roi.multi_match:
        matches = _nms(passing, min_dist=max(8, min(tw, th) // 2))[: roi.max_matches]
    else:
        matches = passing[:1]

    if not matches:
        best = max((m.score for m in cand_matches), default=0.0)
        return RoiResult(roi.name, TEMPLATE, False, matches=[],
                         detail=f"best score {best:.3f} < {roi.score_threshold:.2f}")
    return RoiResult(roi.name, TEMPLATE, passed=True, matches=matches,
                     detail=f"{len(matches)} match(es)")


def run_threshold_roi(image_bgr: np.ndarray, roi: Roi) -> RoiResult:
    h, w = image_bgr.shape[:2]
    x, y, rw, rh = _clamp_rect(roi.rect, w, h)
    region = image_bgr[y:y + rh, x:x + rw]

    if roi.color_space == "hsv":
        conv = cv2.cvtColor(region, cv2.COLOR_BGR2HSV)
        lo = np.array(roi.lower[:3], dtype=np.uint8)
        hi = np.array(roi.upper[:3], dtype=np.uint8)
        # hue wrap-around (e.g. red: lo H=170, hi H=10)
        if lo[0] > hi[0]:
            m1 = cv2.inRange(conv, np.array([lo[0], lo[1], lo[2]], np.uint8),
                             np.array([179, hi[1], hi[2]], np.uint8))
            m2 = cv2.inRange(conv, np.array([0, lo[1], lo[2]], np.uint8),
                             np.array([hi[0], hi[1], hi[2]], np.uint8))
            mask = cv2.bitwise_or(m1, m2)
        else:
            mask = cv2.inRange(conv, lo, hi)
    else:
        gray = cv2.cvtColor(region, cv2.COLOR_BGR2GRAY)
        mask = cv2.inRange(gray, int(roi.lower[0]), int(roi.upper[0]))

    pct = 100.0 * float(np.count_nonzero(mask)) / mask.size
    passed = roi.min_percent <= pct <= roi.max_percent
    return RoiResult(roi.name, THRESHOLD, passed, in_range_percent=pct,
                     detail=f"{pct:.1f}% in range (need {roi.min_percent:.0f}-{roi.max_percent:.0f}%)")


class InspectionEngine:
    """Runs a recipe's ROIs against an image. Templates are cached as
    grayscale arrays keyed by file path + mtime."""

    def __init__(self) -> None:
        self._template_cache: dict = {}

    def load_template(self, path: str) -> Optional[np.ndarray]:
        import os
        try:
            mtime = os.path.getmtime(path)
        except OSError:
            return None
        key = (path, mtime)
        if key not in self._template_cache:
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            if img is None:
                return None
            # keep the cache from growing across template re-teaches
            for k in [k for k in self._template_cache if k[0] == path]:
                del self._template_cache[k]
            self._template_cache[key] = img
        return self._template_cache[key]

    def run(self, image_bgr: np.ndarray, rois: list[Roi],
            template_dir: str) -> InspectionResult:
        from pathlib import Path
        t0 = time.perf_counter()
        gray = None
        results: list[RoiResult] = []
        for roi in rois:
            if not roi.enabled:
                continue
            if roi.kind == TEMPLATE:
                if gray is None:
                    gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY)
                tpl = self.load_template(str(Path(template_dir) / roi.template_file)) \
                    if roi.template_file else None
                if tpl is None:
                    results.append(RoiResult(roi.name, TEMPLATE, False,
                                             detail="template image missing"))
                    continue
                results.append(run_template_roi(gray, roi, tpl))
            elif roi.kind == THRESHOLD:
                results.append(run_threshold_roi(image_bgr, roi))
        elapsed = (time.perf_counter() - t0) * 1000.0
        overall = all(r.passed for r in results) if results else False
        return InspectionResult(passed=overall, rois=results, elapsed_ms=elapsed)


def annotate(image_bgr: np.ndarray, rois: list[Roi],
             result: Optional[InspectionResult]) -> np.ndarray:
    """Draw ROI boxes and match markers on a copy of the image."""
    out = image_bgr.copy()
    res_by_name = {r.name: r for r in result.rois} if result else {}
    for roi in rois:
        if not roi.enabled:
            continue
        x, y, w, h = [int(v) for v in roi.rect]
        rr = res_by_name.get(roi.name)
        if rr is None:
            color = (0, 200, 255)          # yellow: not evaluated
        else:
            color = (0, 200, 0) if rr.passed else (0, 0, 255)
        cv2.rectangle(out, (x, y), (x + w, y + h), color, 3)
        label = roi.name
        if rr is not None and rr.kind == THRESHOLD:
            label += f" {rr.in_range_percent:.0f}%"
        cv2.putText(out, label, (x + 4, max(y - 8, 20)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2, cv2.LINE_AA)
        if rr is not None:
            for m in rr.matches:
                cv2.drawMarker(out, (m.x, m.y), (255, 128, 0),
                               cv2.MARKER_CROSS, 40, 3)
                cv2.putText(out, f"{m.score:.2f}", (m.x + 12, m.y - 12),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 128, 0), 2, cv2.LINE_AA)
    if result is not None:
        txt = "PASS" if result.passed else "FAIL"
        color = (0, 200, 0) if result.passed else (0, 0, 255)
        cv2.putText(out, f"{txt}  {result.elapsed_ms:.0f} ms", (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 2.0, color, 4, cv2.LINE_AA)
    return out
