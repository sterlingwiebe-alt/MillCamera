"""Continuously write an incrementing counter to a Logix controller tag.

Commissioning utility: proves sustained EtherNet/IP writes, measures write
latency, and self-heals on connection loss.

    python scripts/plc_heartbeat.py --ip 192.168.1.20 --tag HeartbeatAI
        --interval 0.1 --duration 1800
"""

from __future__ import annotations

import argparse
import statistics
import time

from pylogix import PLC


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", required=True)
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--tag", default="HeartbeatAI")
    ap.add_argument("--interval", type=float, default=0.1)
    ap.add_argument("--duration", type=float, default=1800,
                    help="seconds to run (0 = forever)")
    ap.add_argument("--start", type=int, default=0)
    args = ap.parse_args()

    value = args.start
    writes = 0
    errors = 0
    reconnects = 0
    lat: list[float] = []
    t0 = time.perf_counter()
    next_due = t0
    comm = PLC(ip_address=args.ip, slot=args.slot)
    print(f"heartbeat -> {args.tag} @ {args.ip}/{args.slot} "
          f"every {args.interval * 1000:.0f} ms")
    try:
        while True:
            now = time.perf_counter()
            if args.duration and (now - t0) >= args.duration:
                break
            if now < next_due:
                time.sleep(min(next_due - now, args.interval))
                continue
            next_due += args.interval
            value = (value + 1) & 0x7FFFFFFF
            tw = time.perf_counter()
            try:
                r = comm.Write(args.tag, value)
                ok = r.Status == "Success"
            except Exception:
                ok = False
            if ok:
                lat.append((time.perf_counter() - tw) * 1000)
                writes += 1
            else:
                errors += 1
                try:
                    comm.Close()
                except Exception:
                    pass
                time.sleep(1.0)
                comm = PLC(ip_address=args.ip, slot=args.slot)
                reconnects += 1
                next_due = time.perf_counter()
            if writes and writes % 100 == 0 and len(lat) >= 50:
                recent = lat[-100:]
                print(f"  {writes} writes, value={value}, "
                      f"lat ms p50={statistics.median(recent):.1f} "
                      f"max={max(recent):.1f}, errors={errors}, "
                      f"reconnects={reconnects}", flush=True)
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        try:
            comm.Close()
        except Exception:
            pass
    dt = time.perf_counter() - t0
    if lat:
        print(f"done: {writes} writes in {dt:.0f}s "
              f"({writes / dt:.1f}/s), latency ms: "
              f"p50={statistics.median(lat):.1f} "
              f"p95={sorted(lat)[int(len(lat) * 0.95)]:.1f} "
              f"max={max(lat):.1f}, errors={errors}, reconnects={reconnects}")


if __name__ == "__main__":
    main()
