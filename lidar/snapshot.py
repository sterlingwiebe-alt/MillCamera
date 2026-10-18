"""One-shot LiDAR snapshot on a PLC trigger -- dumps a raw-frame burst to a
pkl. Used to map the fixed zone-4 structure (capture with zone 4 EMPTY) and
for ad-hoc scene grabs. No optimizer pairing.

    python lidar/snapshot.py --tag Vision_Capture --ack --dump <dir> --label empty
"""
from __future__ import annotations

import argparse
import pickle
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from pylogix import PLC
from lidar.live_bridge2 import FrameBuffer
from lidar.probe_zones import PRESENCE_WINDOWS, measure

CAM_TAG = "CamKicker_Servo.ActualPosition"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="Vision_Capture")
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--seconds", type=float, default=1.0)
    ap.add_argument("--label", default="snap")
    ap.add_argument("--dump", required=True)
    ap.add_argument("--ack", action="store_true")
    ap.add_argument("--duration", type=float, default=7200)
    args = ap.parse_args()

    dump = Path(args.dump)
    dump.mkdir(parents=True, exist_ok=True)

    buf = FrameBuffer()
    threading.Thread(target=buf.run, args=(args.port,), daemon=True).start()
    t0 = time.time()
    last = False
    n = 0
    with PLC(ip_address=args.ip, slot=args.slot) as plc:
        print(f"watching '{args.tag}' -- snapshot on rising edge "
              f"(label={args.label})", flush=True)
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
                pr = " ".join(z.split("_")[0] for z in PRESENCE_WINDOWS
                              if pres[z][0])
                fn = f"{args.label}_{n:02d}_{time.strftime('%H%M%S')}.pkl"
                with open(dump / fn, "wb") as f:
                    pickle.dump({"frames": frames, "cam": cam, "pres": pres}, f)
                print(f"\n[{time.strftime('%H:%M:%S')}] SNAP #{n} cam={cam} "
                      f"frames={len(frames)} present:[{pr}] -> {fn}", flush=True)
                if args.ack:
                    plc.Write(args.tag, 0)
            last = v
            time.sleep(0.2)


if __name__ == "__main__":
    main()
