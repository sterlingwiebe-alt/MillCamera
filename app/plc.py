"""PLC result delivery.

AdsPlc talks to the Beckhoff CX8200 over ADS (TwinCAT 3, PLC port 851)
and writes into GVL_Vision (see plc/GVL_Vision.TcGVL). MockPlc has the
same interface and just remembers what would have been written -- used
until the ADS route is configured, and for bench testing.

Write protocol (matches FB_VisionInspect on the PLC side):
  1. all result data is written in ONE ADS sum-command
  2. nResultId is written LAST -- the PLC treats a change of nResultId as
     "new result available", so data is always complete when it fires.
"""

from __future__ import annotations

import ctypes
import os
import threading
import time
from pathlib import Path
from typing import Optional

MAX_ROIS = 8  # must match the array sizes in GVL_Vision

# Python 3.8+ does not search PATH for ctypes DLLs, so pyads cannot find
# TcAdsDll.dll unless its folder is registered explicitly.
_TCADS_DIRS = [
    r"C:\Program Files (x86)\Beckhoff\TwinCAT\Common64",
    r"C:\TwinCAT\Common64",
    r"C:\TwinCAT\3.1\Common64",
]


_dll_dir_added = False


def _ensure_ads_dll() -> None:
    """Idempotent: add_dll_directory must run ONCE -- calling it per
    reconnect attempt grows the process DLL search path until Windows
    rejects it (WinError 206)."""
    global _dll_dir_added
    if _dll_dir_added:
        return
    for d in _TCADS_DIRS:
        if Path(d, "TcAdsDll.dll").exists():
            os.add_dll_directory(d)
            break
    _dll_dir_added = True


class BasePlc:
    def __init__(self) -> None:
        self.connected = False
        self.last_error = ""
        self.writes = 0

    def connect(self) -> None: ...
    def close(self) -> None: ...

    def write_result(self, result_id: int, passed: bool, roi_results: list,
                     elapsed_ms: float) -> None: ...

    def write_heartbeat(self, value: int) -> None: ...

    def info(self) -> dict:
        return {
            "type": type(self).__name__,
            "connected": self.connected,
            "last_error": self.last_error,
            "writes": self.writes,
            "last_write_ms": round(getattr(self, "last_write_ms", 0.0), 1),
        }


def _flatten(result_id: int, passed: bool, roi_results: list, elapsed_ms: float) -> dict:
    """Build the symbol->value dict (everything except nResultId)."""
    roi_pass = [False] * MAX_ROIS
    match_x = [0] * MAX_ROIS
    match_y = [0] * MAX_ROIS
    score = [0.0] * MAX_ROIS
    for i, rr in enumerate(roi_results[:MAX_ROIS]):
        roi_pass[i] = bool(rr.passed)
        if rr.matches:
            match_x[i] = int(rr.matches[0].x)
            match_y[i] = int(rr.matches[0].y)
            score[i] = float(rr.matches[0].score)
        elif rr.kind == "threshold":
            score[i] = float(rr.in_range_percent) / 100.0
    values = {
        "GVL_Vision.stResult.bPass": bool(passed),
        "GVL_Vision.stResult.nRoiCount": min(len(roi_results), MAX_ROIS),
        "GVL_Vision.stResult.rProcTimeMs": float(elapsed_ms),
    }
    for i in range(MAX_ROIS):
        values[f"GVL_Vision.stResult.aRoiPass[{i + 1}]"] = roi_pass[i]
        values[f"GVL_Vision.stResult.aMatchX[{i + 1}]"] = match_x[i]
        values[f"GVL_Vision.stResult.aMatchY[{i + 1}]"] = match_y[i]
        values[f"GVL_Vision.stResult.aScore[{i + 1}]"] = score[i]
    return values


class AdsPlc(BasePlc):
    """Writes the whole result as ONE packed ST_VisionResult struct through a
    persistent symbol handle -- a single ADS round trip (~17 ms on the
    CX8200, vs ~80+ ms for name-based writes which re-resolve handles every
    call). nResultId is the struct's last field, so the PLC's new-result
    signal arrives in the same write as the data."""

    # must match ST_VisionResult with pack_mode 1 (114 bytes)
    _STRUCT_FMT = "<BB8B8i8i8ffI"

    def __init__(self, ams_net_id: str, ip_address: str = "", port: int = 851) -> None:
        super().__init__()
        self.ams_net_id = ams_net_id
        self.ip_address = ip_address or None
        self.port = port
        self._conn = None
        self._h_result = None
        self._h_beat = None
        # RLock: connect() -> close() nests acquisition of the same lock
        self._lock = threading.RLock()

    def connect(self) -> None:
        _ensure_ads_dll()
        import pyads
        with self._lock:
            if self._conn is not None:
                self.close()
            try:
                self._conn = pyads.Connection(self.ams_net_id, self.port, self.ip_address)
                self._conn.open()
                # probe the link so 'connected' means something
                state = self._conn.read_state()
                # persistent handles; re-acquired on every reconnect because
                # a PLC re-download invalidates them
                self._h_result = self._conn.get_handle("GVL_Vision.stResult")
                self._h_beat = self._conn.get_handle("GVL_Vision.nHeartbeat")
            except Exception:
                self.close()
                raise
            self.connected = True
            self.last_error = ""
            self.plc_state = state

    def close(self) -> None:
        with self._lock:
            if self._conn is not None:
                for h in (self._h_result, self._h_beat):
                    if h is not None:
                        try:
                            self._conn.release_handle(h)
                        except Exception:
                            pass
                self._h_result = self._h_beat = None
                try:
                    self._conn.close()
                except Exception:
                    pass
                self._conn = None
            self.connected = False

    def _pack(self, result_id, passed, roi_results, elapsed_ms) -> bytes:
        import struct
        roi_pass = [0] * MAX_ROIS
        match_x = [0] * MAX_ROIS
        match_y = [0] * MAX_ROIS
        score = [0.0] * MAX_ROIS
        for i, rr in enumerate(roi_results[:MAX_ROIS]):
            roi_pass[i] = 1 if rr.passed else 0
            if rr.matches:
                match_x[i] = int(rr.matches[0].x)
                match_y[i] = int(rr.matches[0].y)
                score[i] = float(rr.matches[0].score)
            elif rr.kind == "threshold":
                score[i] = float(rr.in_range_percent) / 100.0
        return struct.pack(self._STRUCT_FMT,
                           1 if passed else 0,
                           min(len(roi_results), MAX_ROIS),
                           *roi_pass, *match_x, *match_y, *score,
                           float(elapsed_ms),
                           int(result_id) & 0xFFFFFFFF)

    def write_result(self, result_id, passed, roi_results, elapsed_ms) -> None:
        from pyads.constants import ADSIGRP_SYM_VALBYHND
        with self._lock:
            if self._conn is None or self._h_result is None:
                raise RuntimeError("ADS not connected")
            data = self._pack(result_id, passed, roi_results, elapsed_ms)
            t0 = time.perf_counter()
            try:
                # ctypes array form: plc_datatype=None silently sends NOTHING
                arr_t = ctypes.c_ubyte * len(data)
                self._conn.write(ADSIGRP_SYM_VALBYHND, self._h_result,
                                 list(data), arr_t)
            except Exception:
                # stale handle after a PLC re-download -> force reconnect
                self.connected = False
                raise
            self.last_write_ms = (time.perf_counter() - t0) * 1000.0
            self.writes += 1

    def write_heartbeat(self, value: int) -> None:
        import pyads
        from pyads.constants import ADSIGRP_SYM_VALBYHND
        with self._lock:
            if self._conn is None or self._h_beat is None:
                return
            try:
                self._conn.write(ADSIGRP_SYM_VALBYHND, self._h_beat,
                                 int(value) & 0xFFFFFFFF, pyads.PLCTYPE_UDINT)
            except Exception:
                self.connected = False
                raise


class RockwellPlc(BasePlc):
    """ControlLogix/CompactLogix over EtherNet/IP (CIP explicit messaging,
    pylogix -- chosen over pycomm3 because it needs no tag-list upload, so
    the same code path works on real controllers AND simple CIP servers/
    emulators used for bench testing). No vendor software needed on the PC.

    Transport contract (mirrors the Beckhoff single-write pattern):
    everything goes into ONE controller-scope DINT[40] tag in ONE atomic
    write, with ResultId as the LAST element -- the PLC treats a change of
    Vision_Transport[39] as "new result", by which time the rest of the
    array is guaranteed current.

    DINT[40] layout (documented for the PLC-side routine):
      [0]      Pass (0/1)
      [1]      RoiCount
      [2..9]   RoiPass[1..8]        (0/1)
      [10..17] MatchX[1..8]         (pixels / units)
      [18..25] MatchY[1..8]
      [26..33] ScoreMilli[1..8]     (score x1000, 0..1000)
      [34]     ProcTime_x10ms       (elapsed ms x10)
      [35..38] reserved
      [39]     ResultId             (LAST -- completion flag)
    """

    TRANSPORT_TAG = "Vision_Transport"
    HEARTBEAT_TAG = "Vision_Heartbeat"

    def __init__(self, ip_address: str, slot: int = 0,
                 port: int = 44818) -> None:
        super().__init__()
        self.ip_address = ip_address
        self.slot = slot
        self.port = port    # non-default only for local test servers --
        self._conn = None   # FactoryTalk Linx owns 44818 on Rockwell PCs
        self._lock = threading.RLock()

    def connect(self) -> None:
        from pylogix import PLC
        with self._lock:
            if self._conn is not None:
                self.close()
            try:
                # normal routed mode (real ControlLogix/CompactLogix)
                self._conn = PLC(ip_address=self.ip_address,
                                 slot=self.slot, port=self.port)
                r = self._conn.Read(f"{self.TRANSPORT_TAG}[0]")
                if r.Status != "Success":
                    # unrouted mode: simple CIP servers / emulators (cpppo)
                    # don't understand the backplane route path
                    try:
                        self._conn.Close()
                    except Exception:
                        pass
                    self._conn = PLC(ip_address=self.ip_address,
                                     Micro800=True, port=self.port)
                    r = self._conn.Read(f"{self.TRANSPORT_TAG}[0]")
                if r.Status != "Success":
                    raise RuntimeError(
                        f"tag {self.TRANSPORT_TAG} not readable: {r.Status}")
            except Exception:
                self.close()
                raise
            self.connected = True
            self.last_error = ""

    def close(self) -> None:
        with self._lock:
            if self._conn is not None:
                try:
                    self._conn.Close()
                except Exception:
                    pass
                self._conn = None
            self.connected = False

    def _pack(self, result_id, passed, roi_results, elapsed_ms) -> list:
        vals = [0] * 40
        vals[0] = 1 if passed else 0
        vals[1] = min(len(roi_results), MAX_ROIS)
        for i, rr in enumerate(roi_results[:MAX_ROIS]):
            vals[2 + i] = 1 if rr.passed else 0
            if rr.matches:
                vals[10 + i] = int(rr.matches[0].x)
                vals[18 + i] = int(rr.matches[0].y)
                vals[26 + i] = int(rr.matches[0].score * 1000)
            elif rr.kind == "threshold":
                vals[26 + i] = int(rr.in_range_percent * 10)
        vals[34] = int(elapsed_ms * 10)
        vals[39] = int(result_id) & 0x7FFFFFFF
        return vals

    def write_result(self, result_id, passed, roi_results, elapsed_ms) -> None:
        with self._lock:
            if self._conn is None:
                raise RuntimeError("EtherNet/IP not connected")
            vals = self._pack(result_id, passed, roi_results, elapsed_ms)
            t0 = time.perf_counter()
            try:
                # a list value writes len(list) consecutive elements in one
                # request -- the whole result lands atomically, ResultId last
                r = self._conn.Write(f"{self.TRANSPORT_TAG}[0]", vals)
                if r.Status != "Success":
                    raise RuntimeError(f"write failed: {r.Status}")
            except Exception:
                self.connected = False
                raise
            self.last_write_ms = (time.perf_counter() - t0) * 1000.0
            self.writes += 1

    def write_heartbeat(self, value: int) -> None:
        with self._lock:
            if self._conn is None:
                return
            try:
                r = self._conn.Write(self.HEARTBEAT_TAG, int(value) & 0x7FFFFFFF)
                if r.Status != "Success":
                    raise RuntimeError(f"heartbeat write failed: {r.Status}")
            except Exception:
                self.connected = False
                raise


class MockPlc(BasePlc):
    """Stands in for the CX8200 until the ADS route exists."""

    def __init__(self) -> None:
        super().__init__()
        self.last_write: Optional[dict] = None
        self.heartbeat = 0

    def connect(self) -> None:
        self.connected = True

    def close(self) -> None:
        self.connected = False

    def write_result(self, result_id, passed, roi_results, elapsed_ms) -> None:
        values = _flatten(result_id, passed, roi_results, elapsed_ms)
        values["GVL_Vision.stResult.nResultId"] = result_id
        self.last_write = {"time": time.time(), "values": values}
        self.writes += 1

    def write_heartbeat(self, value: int) -> None:
        self.heartbeat = value

    def info(self) -> dict:
        d = super().info()
        d["last_write"] = self.last_write
        return d
