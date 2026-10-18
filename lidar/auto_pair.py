"""Auto-paired zone-4 diameter verification.

Flip the trigger tag at a PAUSE (logs parked in 2/3/4, takeaway clear). On
the rising edge this:
  1. captures the zone-4 diameter (circle-fit AVG) + a presence snapshot,
  2. marks that instant as the reset,
  3. then reads the PLC's own zone-4 slot, TestLog[2], which the ladder
     latches when the zone-4 log reaches the optimizer after restart
     (TestLog resets on each line restart; SED/LED already in mm). Pairs
     TestLog[2] against the captured zone-4 circle-fit, reporting the error.

The PLC does the log tracking (which optimizer log is zone 4) -- we no
longer infer it from the raw register sequence. Re-arms after each pair.

    python lidar/auto_pair.py --tag Vision_Capture --ack
"""
from __future__ import annotations

import argparse
import pickle
import sys
import threading
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC
from lidar.live_bridge2 import FrameBuffer
from lidar.probe_zones import (PRESENCE_WINDOWS, measure, z4_estimators,
                               z4_height_features)

CAM_TAG = "CamKicker_Servo.ActualPosition"
# zone-4 slot the PLC latches (already mm -- PLC does the in->mm conversion)
TESTLOG4 = ["TestLog[2].SED", "TestLog[2].LED", "TestLog[2].Length"]


def read_testlog4(plc):
    """(SED, LED, Length) mm from TestLog[2] (zone-4 slot), or None."""
    r = plc.Read(TESTLOG4)
    try:
        vals = [resp.Value for resp in r]
    except TypeError:          # single Response if list collapsed
        return None
    if any(v is None for v in vals):
        return None
    try:
        return tuple(int(v) for v in vals)
    except (TypeError, ValueError):
        return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--wait", type=float, default=150,
                    help="max s to wait for TestLog[2] to fill after restart")
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--duration", type=float, default=7200)
    ap.add_argument("--dump", default="", help="dir to save raw frames + CSV "
                    "(labeled with the paired optimizer truth)")
    args = ap.parse_args()

    dump = Path(args.dump) if args.dump else None
    csv = None
    if dump:
        dump.mkdir(parents=True, exist_ok=True)
        csv = dump / "pairs.csv"
        if not csv.exists():
            csv.write_text("reset,time,cam,circ,method,mean,p90,p95,binmax_med,"
                           "n_arcs,h_max,h_p90,h_med,h_ncell,h_fx,h_fy,"
                           "opt_sed,opt_led,opt_mid,opt_len,"
                           "err_mid,err_sed,pkl\n")

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    t0 = time.time()
    last = False
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"watching '{args.tag}' -- flip at PAUSE; zone 4 read from "
              f"PLC TestLog[2] after restart", flush=True)
        while time.time() - t0 < args.duration:
            r = plc.Read(args.tag)
            if r.Status != "Success":
                print("trigger read failed:", r.Status, flush=True)
                time.sleep(2); continue
            v = bool(r.Value)
            if v and not last:
                n += 1
                cr = plc.Read(CAM_TAG)
                cam = round(float(cr.Value), 1) if cr.Status == "Success" else -1
                tb = time.time()
                frames = {}
                while time.time() - tb < args.seconds:
                    fn, segs = buf.latest_complete()
                    if segs and fn not in frames:
                        frames[fn] = segs
                    time.sleep(0.02)
                pres, dia4, _ = measure(frames)
                circ = dia4["avg"] if dia4 else None
                meth = dia4.get("method", "?") if dia4 else "-"
                est = z4_estimators(frames)
                hf = z4_height_features(frames)
                pr = " ".join(z.split("_")[0] for z in PRESENCE_WINDOWS
                              if pres[z][0])
                print(f"\n[{time.strftime('%H:%M:%S')}] RESET #{n} cam={cam} "
                      f"-- present:[{pr}]  zone4 circ={circ} ({meth})", flush=True)
                if est:
                    print(f"  estimators: mean={est['mean']} p90={est['p90']} "
                          f"p95={est['p95']} binmax_med={est['binmax_med']} "
                          f"(n={est['n']})", flush=True)
                if hf:
                    print(f"  height: h_max={hf['h_max']} h_p90={hf['h_p90']} "
                          f"h_med={hf['h_med']} (ncell={hf['ncell']} "
                          f"fx={hf['fx']} fy={hf['fy']})", flush=True)
                if args.ack:
                    plc.Write(args.tag, 0)
                pkl = ""
                if dump:
                    pkl = f"reset_{n:02d}_{time.strftime('%H%M%S')}.pkl"
                    with open(dump / pkl, "wb") as f:
                        pickle.dump({"frames": frames, "cam": cam, "circ": circ,
                                     "method": meth, "pres": pres}, f)
                    print(f"  frames saved: {pkl} ({len(frames)})", flush=True)
                if circ is None:
                    print("  no zone-4 log captured -- skip", flush=True)
                    last = v
                    continue
                # Wait for the PLC's zone-4 slot TestLog[2] to fill. It holds
                # the last stop's value (anchor) until the line restarts, which
                # resets TestLog, then the ladder writes the real zone-4 log.
                # So: wait for a fresh, stable value that differs from anchor.
                anchor = read_testlog4(plc)
                print(f"  anchor TestLog[2]={anchor} -- restart the line; "
                      f"waiting for zone-4 log...", flush=True)
                cand, ccount = None, 0
                STABLE = 3          # hold ~0.6s -> ignore mid-scan writes
                result = None
                tw = time.time()
                while time.time() - tw < args.wait:
                    k = read_testlog4(plc)
                    if k is None or k[0] <= 0:
                        cand, ccount = None, 0
                        time.sleep(0.2); continue
                    if k == cand:
                        ccount += 1
                    else:
                        cand, ccount = k, 1
                    if ccount >= STABLE and k != anchor:
                        result = k
                        break
                    time.sleep(0.2)
                if result:
                    sed, led, length = result
                    mid = (sed + led) / 2
                    print(f"  TestLog[2] zone-4: SED={sed} LED={led} "
                          f"MID={mid:.0f} len={length} (+{time.time()-tw:.0f}s)",
                          flush=True)
                    print(f"  >>> PAIR: zone4 circ={circ}  vs opt MID={mid:.0f} "
                          f"({circ - mid:+.0f})  vs opt SED={sed} "
                          f"({circ - sed:+.0f})  [taper {sed}-{led}]",
                          flush=True)
                    if csv:
                        e = est or {}
                        h = hf or {}
                        with open(csv, "a") as f:
                            f.write(f"{n},{time.strftime('%H:%M:%S')},{cam},"
                                    f"{circ},{meth},{e.get('mean','')},"
                                    f"{e.get('p90','')},{e.get('p95','')},"
                                    f"{e.get('binmax_med','')},{e.get('n','')},"
                                    f"{h.get('h_max','')},{h.get('h_p90','')},"
                                    f"{h.get('h_med','')},{h.get('ncell','')},"
                                    f"{h.get('fx','')},{h.get('fy','')},"
                                    f"{sed},{led},{mid:.0f},{length},"
                                    f"{circ-mid:+.0f},{circ-sed:+.0f},{pkl}\n")
                else:
                    print(f"  TestLog[2] didn't update in {args.wait:.0f}s "
                          f"(still {anchor}?) -- line not restarted?", flush=True)
            last = v
            time.sleep(0.2)


if __name__ == "__main__":
    main()
