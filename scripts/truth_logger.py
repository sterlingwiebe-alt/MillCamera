"""Log every change of the Millwide.Logline.data array to CSV.

Builds the ground-truth stream for the LiDAR calibration correlator.

    python scripts/truth_logger.py --ip 192.168.1.20 --duration 1800
"""

from __future__ import annotations

import argparse
import csv
import time
from datetime import datetime
from pathlib import Path

from pylogix import PLC

ROOT = Path(__file__).resolve().parent.parent
TAG = "Millwide.Logline.data[0]"
N = 12


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ip", default="192.168.1.20")
    ap.add_argument("--slot", type=int, default=0)
    ap.add_argument("--interval", type=float, default=0.1)
    ap.add_argument("--duration", type=float, default=1800)
    args = ap.parse_args()

    out = ROOT / "lidar_recordings" / "truth_log.csv"
    out.parent.mkdir(exist_ok=True)
    new_file = not out.exists()
    comm = PLC(ip_address=args.ip, slot=args.slot)
    last = None
    rows = 0
    t0 = time.time()
    with open(out, "a", newline="") as f:
        wr = csv.writer(f)
        if new_file:
            wr.writerow(["iso_time", "epoch"] +
                        [f"data{i}" for i in range(N)])
        while time.time() - t0 < args.duration:
            try:
                r = comm.Read(TAG, N)
                vals = r.Value if r.Status == "Success" else None
            except Exception:
                vals = None
            # data[0] is a rolling heartbeat from the source PLC -- exclude
            # it from change detection (log it as link-liveness evidence)
            if vals is not None and vals[1:] != last:
                wr.writerow([datetime.now().isoformat(timespec="milliseconds"),
                             round(time.time(), 3)] + list(vals))
                f.flush()
                rows += 1
                print(f"{datetime.now().strftime('%H:%M:%S')} change #{rows}: "
                      f"{vals}", flush=True)
                last = vals[1:]
            time.sleep(args.interval)
    comm.Close()
    print(f"done: {rows} changes in {time.time() - t0:.0f}s -> {out}")


if __name__ == "__main__":
    main()
