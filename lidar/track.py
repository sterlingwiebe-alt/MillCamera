"""Multi-frame log tracking.

Associates per-frame segmentation detections (lidar.segment.detect_logs)
into tracks across frames, then classifies each track as a MOVING log or a
STATIONARY machine part by how far it travels along the flow axis (y). A
log translates through the deck; a cam or screw flight stays put. This is
the motion-based machinery discrimination -- and because association is
within each sensor frame, it is jitter-tolerant (unlike beam-background).

Use incrementally for the live bridge:
    tk = LogTracker()
    for frame_segs in stream:
        tk.update(detect_logs(frame_segs))
        logs = tk.live_logs()          # moving tracks present now

or over a whole recording for offline analysis:
    tracks = track_recording(rec)
    logs = [t for t in tracks if t.is_moving()]
"""
from __future__ import annotations

from collections import defaultdict

import numpy as np

from .pointcloud import load_segments
from .segment import detect_logs
from .zones import DEFAULT_ZONES, SENSOR_Y_OFFSET

GATE_Y = 0.25        # max |dy| frame-to-frame to associate [m]
GATE_DIA = 60        # max |ddia| to associate [mm]
MAX_GAP = 6          # close a track after this many missed frames
MIN_LEN = 5          # min detections for a usable track
MOVE_THRESH = 0.35   # y travel above this = moving (log) vs machine [m]


class Track:
    def __init__(self, tid, frame, det):
        self.id = tid
        self.pts = [(frame, det)]      # [(frame_idx, log-dict)]
        self.last_frame = frame

    @property
    def last(self):
        return self.pts[-1][1]

    @property
    def n(self):
        return len(self.pts)

    @property
    def frames(self):
        return self.pts[0][0], self.pts[-1][0]

    def _ys(self):
        return np.array([d["center"][1] for _, d in self.pts])

    @property
    def y_range(self):
        ys = self._ys()
        return float(ys.max() - ys.min()) if len(ys) else 0.0

    def is_moving(self, thresh=MOVE_THRESH):
        return self.y_range > thresh

    def diameter(self, band=None, key="dia_arc_mm"):
        """Median diameter, optionally only over detections whose y center
        falls in `band` (lo, hi) -- e.g. the clean top-of-screw view.
        Defaults to the arc-radius estimate (dia_arc_mm), which is
        unbiased vs the optimizer (~13 mm); falls back to dia_mm."""
        pts = self.pts
        if band is not None:
            pts = [(f, d) for f, d in pts
                   if band[0] <= d["center"][1] <= band[1]]
        ds = [d.get(key, d["dia_mm"]) for _, d in pts]
        return int(np.median(ds)) if ds else -1

    def zone(self, smooth=5):
        """Zone of the track's smoothed recent center (by y band). Uses the
        median position over the last `smooth` frames so single-frame mount
        jitter can't flip the assignment across a zone boundary."""
        recent = self.pts[-smooth:]
        x = float(np.median([d["center"][0] for _, d in recent]))
        y = float(np.median([d["center"][1] for _, d in recent])) - SENSOR_Y_OFFSET
        for name, z in DEFAULT_ZONES.items():
            (x0, x1), (y0, y1) = z["x"], z["y"]
            if x0 <= x <= x1 and y0 <= y <= y1:
                return name
        return None

    def add(self, frame, det):
        self.pts.append((frame, det))
        self.last_frame = frame


class LogTracker:
    def __init__(self, gate_y=GATE_Y, gate_dia=GATE_DIA, max_gap=MAX_GAP,
                 min_len=MIN_LEN, move_thresh=MOVE_THRESH):
        self.gate_y = gate_y
        self.gate_dia = gate_dia
        self.max_gap = max_gap
        self.min_len = min_len
        self.move_thresh = move_thresh
        self.active = []
        self.closed = []
        self._next = 0
        self.frame = -1

    def update(self, dets):
        """Advance one frame with the current detections (log dicts)."""
        self.frame += 1
        used = [False] * len(dets)
        for tr in self.active:
            ly, ld = tr.last["center"][1], tr.last["dia_mm"]
            best, bj = 1e9, -1
            for j, d in enumerate(dets):
                if used[j]:
                    continue
                dy = abs(d["center"][1] - ly)
                dd = abs(d["dia_mm"] - ld)
                if dy < self.gate_y and dd < self.gate_dia:
                    cost = dy + dd / 1000.0
                    if cost < best:
                        best, bj = cost, j
            if bj >= 0:
                used[bj] = True
                tr.add(self.frame, dets[bj])
        for j, d in enumerate(dets):
            if not used[j]:
                self.active.append(Track(self._next, self.frame, d))
                self._next += 1
        still = []
        for tr in self.active:
            if self.frame - tr.last_frame > self.max_gap:
                self.closed.append(tr)
            else:
                still.append(tr)
        self.active = still

    def live_logs(self):
        """Active tracks that already look like moving logs (for the live
        bridge). Has motion-onset latency -- a brand-new track hasn't
        travelled yet, so it isn't a log until it does."""
        return [t for t in self.active
                if t.n >= self.min_len and t.is_moving(self.move_thresh)]

    def all_tracks(self):
        """Every usable track (active + closed), for offline analysis."""
        return [t for t in self.closed + self.active if t.n >= self.min_len]


def track_recording(recording, region=None):
    """Run segmentation + tracking over a whole recording; return tracks."""
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
    for fn in order:
        dets = (detect_logs(frames[fn]) if region is None
                else detect_logs(frames[fn], region=region))
        tk.update(dets)
    return tk.all_tracks()
