"""PLC OUTPUT -- publish the live deck state to Vision_Transport (DINT[40]).

Runs as a thread inside interlace_capture.py (enable with --plc-out). Takes
the FlowTracker's state directly (presence per zone, zone-4 pass in
progress, last completed pass, last optimizer pairing, cam gate) and writes
all 40 DINTs in ONE write at RATE_HZ, ResultId ([39]) changing last, plus
Vision_Heartbeat once a second -- the same contract as the vision app.

Layout v2 is documented in plc/Vision_Transport_v2.md (approved by the user
2026-10-15). Only Vision_Transport and Vision_Heartbeat are ever written.
"""
from __future__ import annotations

import threading
import time

import numpy as np

TRANSPORT_TAG = "Vision_Transport"
HEARTBEAT_TAG = "Vision_Heartbeat"
RATE_HZ = 5.0
LAYOUT_VERSION = 2
ZONES = ["1_OutfeedConveyor", "2_CamWaiting", "3_CamStaging", "4_TopOfScrew",
         "5_MidScrew", "6_ScrewDeck", "7_InfeedDeck"]
STALE_S = 1.0


class PlcOut(threading.Thread):
    def __init__(self, flow, tap, ip, slot):
        super().__init__(daemon=True)
        self.flow, self.tap, self.ip, self.slot = flow, tap, ip, slot
        self.stop = False
        self.rid = 0
        self.writes = 0
        self.errors = 0

    # ------------------------------------------------------------ packing
    def pack(self):
        f = self.flow
        now = time.time()
        fresh = 1 if (now - getattr(f, "_pub_t", 0.0)) < STALE_S else 0
        zones = getattr(f, "last_zones", {}) or {}
        cam_moving = getattr(f, "last_cam_moving", {}) or {}
        v = [0] * 40
        v[0] = fresh
        v[1] = len(ZONES)
        for i, z in enumerate(ZONES):
            zi = zones.get(z)
            if zi and zi.get("cam_moving"):
                v[2 + i] = 2
            elif zi and zi.get("present"):
                v[2 + i] = 1
            d = zi.get("dia") if zi else None
            v[9 + i] = int(d) if d else -1
        cur = getattr(f, "_cur", None)
        if cur and cur.get("dias"):
            dias = np.asarray(cur["dias"], float)
            v[12] = int(round(np.median(dias)))               # zone 4 live median
            v[16] = len(dias)
            travel = (cur["ys"][0] - cur["ys"][-1]) * 1000.0
            v[17] = 1 if abs(travel) > 60 else 0
        if f.passes:
            p = f.passes[-1]
            v[18] = int(p["dia"]); v[19] = int(p["n"]); v[20] = int(p["iqr"])
            v[21] = 1 if p["moving"] else 0
            v[22] = 0 if (p["iqr"] > 15 or p["n"] < 20) else 1
            v[23] = len(f.passes)
        # cam angle from the same instant as the gate bits and presence states
        cam = getattr(f, "last_cam", None)
        v[24] = int(round((cam % 360.0) * 10)) if cam is not None else -1
        bits = 0
        if cam_moving.get("2_CamWaiting"):
            bits |= 1
        if cam_moving.get("3_CamStaging"):
            bits |= 2
        v[25] = bits
        lp = getattr(f, "_last_pair", None)
        if lp:
            v[26] = int(lp["lidar"]); v[27] = int(lp["sed"]); v[28] = int(lp["led"])
            v[29] = int(round(lp["band_err"]))
        v[38] = LAYOUT_VERSION
        self.rid = (self.rid + 1) & 0x7FFFFFFF
        v[39] = self.rid
        return [int(x) for x in v]

    # -------------------------------------------------------------- loop
    def run(self):
        from pylogix import PLC
        period = 1.0 / RATE_HZ
        hb, last_hb = 0, 0.0
        while not self.stop:
            try:
                with PLC(ip_address=self.ip, slot=self.slot) as plc:
                    print(f"PLC OUT: writing {TRANSPORT_TAG}[0..39] at {RATE_HZ:.0f} Hz "
                          f"+ {HEARTBEAT_TAG} 1 Hz -> {self.ip}", flush=True)
                    while not self.stop:
                        t0 = time.time()
                        vals = self.pack()
                        r = plc.Write(f"{TRANSPORT_TAG}[0]", vals)
                        if r.Status != "Success":
                            self.errors += 1
                            if self.errors in (1, 10, 100) or self.errors % 1000 == 0:
                                print(f"PLC OUT write failed ({self.errors}): {r.Status}", flush=True)
                        else:
                            self.writes += 1
                        if t0 - last_hb >= 1.0:
                            hb = (hb + 1) & 0x7FFFFFFF
                            hr = plc.Write(HEARTBEAT_TAG, hb)
                            if hr.Status != "Success" and not getattr(self, "_hb_warned", False):
                                print(f"PLC OUT: {HEARTBEAT_TAG} not writable ({hr.Status}) -- "
                                      f"create it as a controller-scope DINT, Read/Write; "
                                      f"Vision_Transport is still being written", flush=True)
                                self._hb_warned = True
                            last_hb = t0
                        time.sleep(max(0.0, period - (time.time() - t0)))
            except Exception as e:          # noqa: BLE001
                print(f"PLC OUT: {type(e).__name__}: {e} -- reconnecting", flush=True)
                time.sleep(3.0)
