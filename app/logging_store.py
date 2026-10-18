"""On-demand inspection logging: images + result JSON to disk.

Modes:
  off       -- nothing saved
  failures  -- save only failed inspections
  next_n    -- save the next N inspections (pass or fail), then auto-off
"""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path

import cv2


class InspectionLogger:
    def __init__(self, root: str) -> None:
        self.root = Path(root)
        self.mode = "off"            # 'off' | 'failures' | 'next_n'
        self.remaining = 0
        self.saved_count = 0
        self._lock = threading.Lock()

    def set_mode(self, mode: str, count: int = 0) -> None:
        with self._lock:
            self.mode = mode
            self.remaining = int(count) if mode == "next_n" else 0
            if mode != "off":
                self.saved_count = 0

    def status(self) -> dict:
        with self._lock:
            return {"mode": self.mode, "remaining": self.remaining,
                    "saved_count": self.saved_count}

    def maybe_log(self, image_bgr, result) -> None:
        with self._lock:
            if self.mode == "off":
                return
            if self.mode == "failures" and result.passed:
                return
            if self.mode == "next_n":
                if self.remaining <= 0:
                    self.mode = "off"
                    return
                self.remaining -= 1
                if self.remaining == 0:
                    self.mode = "off"
            self.saved_count += 1

        day = time.strftime("%Y-%m-%d", time.localtime(result.wall_time or time.time()))
        folder = self.root / day
        folder.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%H%M%S", time.localtime(result.wall_time or time.time()))
        verdict = "PASS" if result.passed else "FAIL"
        base = f"{stamp}_{result.result_id:06d}_{verdict}"
        cv2.imwrite(str(folder / f"{base}.jpg"), image_bgr,
                    [cv2.IMWRITE_JPEG_QUALITY, 92])
        with open(folder / f"{base}.json", "w", encoding="utf-8") as f:
            json.dump(result.to_dict(), f, indent=2)
