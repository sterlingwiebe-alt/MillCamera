"""Streaming zone-4 capture, one reset then continuous.

Procedure:
  1. Pause with logs parked, line ready to restart.
  2. Flip Vision_Capture once at the reset moment -> this marks sync: the
     zone-4 parked log is arc #1, which the PLC latches as TestLog[2].
  3. Restart the line and just let it run.

For each log that passes through zone 4 (presence rising->falling edge) the
tool records that log's whole pass (motion fills the cross-section density),
saves the frames, and indexes it. At the end it reads TestLog[0..N-1] and
pairs arc #k <-> TestLog[k+1] (arc#1<->TestLog[2]). Diameter extraction is
done offline from the saved frames.

    python lidar/stream_capture.py --dump lidar/z4captures/stream1 --nlog 18
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
                               frame_zone_arcs, R_LO, R_HI, MIN_ARCS_FRAME)

Z4 = "4_TopOfScrew"
NSLOT = 20


def z4_present(segs):
    return len(frame_zone_arcs(segs, PRESENCE_WINDOWS[Z4])) >= MIN_ARCS_FRAME


def read_testlog(plc, nslot):
    """Read per slot (3 tags at a time) -- a single 60-tag request overflows
    pylogix and returns a short list."""
    out = []
    for i in range(nslot):
        r = plc.Read([f"TestLog[{i}].SED", f"TestLog[{i}].LED",
                      f"TestLog[{i}].Length"])
        v = [x.Value for x in r]
        out.append(tuple(int(x) if x is not None else -1 for x in v))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--dump", required=True)
    ap.add_argument("--nlog", type=int, default=18, help="zone-4 logs to capture")
    ap.add_argument("--settle", type=int, default=2, help="present frames to start")
    ap.add_argument("--release", type=int, default=3, help="empty frames to end")
    ap.add_argument("--maxpass", type=float, default=20.0, help="max s per log pass")
    ap.add_argument("--minframes", type=int, default=6,
                    help="ignore presence episodes shorter than this (blips)")
    ap.add_argument("--duration", type=float, default=3600)
    args = ap.parse_args()

    dump = Path(args.dump)
    dump.mkdir(parents=True, exist_ok=True)

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()

    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"waiting for {args.tag} rising edge (reset sync)...", flush=True)
        last = False
        t0 = time.time()
        while time.time() - t0 < args.duration:
            r = plc.Read(args.tag)
            v = bool(r.Value) if r.Status == "Success" else False
            if v and not last:
                break
            last = v
            time.sleep(0.1)
        plc.Write(args.tag, 0)
        ptr_sync = plc.Read("TestPtr").Value or 0
        # valid if already reset (low) at sync, or if it drops after restart
        reset_seen = ptr_sync <= 2
        last_ptr_check = time.time()
        print(f"[{time.strftime('%H:%M:%S')}] SYNC (TestPtr={ptr_sync}) -- "
              f"capturing arc#1 (parked zone-4 log = TestLog[2]); "
              f"RESTART the line now to reset the pointer.", flush=True)

        captures = []            # list of (k, pkl, pooled_frames_count)
        k = 0
        seen_fn = set()
        # arc#1: the parked log is present now
        in_log = True
        frames = {}
        episode_t = time.time()
        present_run = args.settle
        empty_run = 0

        while time.time() - t0 < args.duration and k < args.nlog:
            fn, segs = buf.latest_complete()
            if not segs or fn in seen_fn:
                time.sleep(0.02)
                continue
            seen_fn.add(fn)
            present = z4_present(segs)

            if not reset_seen and time.time() - last_ptr_check > 2:
                last_ptr_check = time.time()
                pv = plc.Read("TestPtr").Value
                if pv is not None and pv < ptr_sync:
                    reset_seen = True
                    print(f"[{time.strftime('%H:%M:%S')}] reset confirmed "
                          f"(TestPtr {ptr_sync}->{pv})", flush=True)

            if not in_log:
                if present:
                    present_run += 1
                    if present_run >= args.settle:     # new log arrived
                        in_log = True
                        frames = {}
                        episode_t = time.time()
                        empty_run = 0
                else:
                    present_run = 0
            else:
                frames[fn] = segs
                if present:
                    empty_run = 0
                else:
                    empty_run += 1
                over = time.time() - episode_t > args.maxpass
                if empty_run >= args.release or over:
                    if len(frames) < args.minframes:
                        print(f"[{time.strftime('%H:%M:%S')}] blip ignored "
                              f"({len(frames)} frames)", flush=True)
                        in_log = False
                        present_run = 0
                        frames = {}
                        continue
                    # finalize this log
                    k += 1
                    pkl = f"log_{k:02d}_{time.strftime('%H%M%S')}.pkl"
                    with open(dump / pkl, "wb") as f:
                        pickle.dump({"frames": frames, "k": k}, f)
                    est = z4_estimators(frames)
                    circ = (measure(frames)[1] or {}).get("avg")
                    captures.append((k, pkl, len(frames)))
                    print(f"[{time.strftime('%H:%M:%S')}] LOG #{k} captured "
                          f"({len(frames)} frames, circ={circ}, "
                          f"p95={est['p95'] if est else '-'}) -> pairs TestLog[{k+1}]"
                          + ("  [maxpass]" if over else ""), flush=True)
                    in_log = False
                    present_run = 0
                    frames = {}

        # validate + pair with TestLog
        ptr_end = plc.Read("TestPtr").Value or 0
        tl = read_testlog(plc, NSLOT)
        import json
        (dump / "testlog_snapshot.json").write_text(
            json.dumps({"ts": time.strftime("%H:%M:%S"), "ptr_sync": ptr_sync,
                        "ptr_end": ptr_end, "reset_seen": reset_seen,
                        "testlog": tl}))
        print(f"\n[{time.strftime('%H:%M:%S')}] TestPtr end={ptr_end}  "
              f"reset_seen={reset_seen}", flush=True)
        if not reset_seen:
            print("*** WARNING: line reset NOT detected (TestPtr never dropped) "
                  "-- TestLog is STALE. PAIRING INVALID. Re-run with a proper "
                  "line restart after the sync flip. ***", flush=True)
        nz4 = max(ptr_end - 2, 0)        # zone-4 logs latched = slots 2..ptr_end-1
        if len(captures) != nz4:
            print(f"*** COUNT MISMATCH: captured {len(captures)} zone-4 logs vs "
                  f"{nz4} latched (TestPtr-2). Alignment uncertain -- a log was "
                  f"missed or doubled. ***", flush=True)
        else:
            print(f"count OK: {len(captures)} captured == {nz4} latched", flush=True)
        out = dump / "pairs.csv"
        valid = reset_seen and len(captures) == nz4
        with open(out, "w") as f:
            f.write("k,pkl,nframes,opt_sed,opt_led,opt_mid,opt_len,valid\n")
            for k, pkl, nf in captures:
                slot = k + 1
                if slot < min(NSLOT, ptr_end):
                    s, l, ln = tl[slot]
                    mid = (s + l) / 2 if s > 0 and l > 0 else -1
                    f.write(f"{k},{pkl},{nf},{s},{l},{mid:.0f},{ln},{int(valid)}\n")
                    print(f"  arc#{k} <-> TestLog[{slot}]: SED={s} LED={l} "
                          f"MID={mid:.0f} len={ln}", flush=True)
        print(f"saved {out}  ({len(captures)} logs, valid={valid})", flush=True)


if __name__ == "__main__":
    main()
