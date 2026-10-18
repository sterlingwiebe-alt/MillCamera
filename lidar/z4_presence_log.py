r"""Log zone-4 presence transitions from live_state.json (Off->On / On->Off).

Polls the tracker's live_state.json 20x per second and appends one row per
change to <dump>/z4_presence_transitions.csv: time, new state, how long the
previous state lasted, the zone-4 live diameter and the cam angle. Used to
show the user the OFF gap between consecutive logs after the zone-4 presence
band was narrowed (2026-10-15).

    python lidar\z4_presence_log.py --dump lidar\z4captures\interlaced
"""
import argparse, csv, json, time
from datetime import datetime
from pathlib import Path

ZONE = "4_TopOfScrew"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dump", default="lidar/z4captures/interlaced")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args()
    dump = Path(a.dump)
    src, out = dump / "live_state.json", dump / "z4_presence_transitions.csv"
    new = not out.exists()
    f = open(out, "a", newline="")
    w = csv.writer(f)
    if new:
        w.writerow(["iso", "t", "state", "prev_state_s", "z4_live_mm", "cam_deg"])
        f.flush()
    prev, t_change, last_t = None, None, 0.0
    print(f"zone-4 presence transitions -> {out}", flush=True)
    while True:
        try:
            s = json.loads(src.read_text())
        except Exception:
            time.sleep(0.05); continue
        if s.get("t") == last_t:
            time.sleep(0.03); continue
        last_t = s["t"]
        z = (s.get("zones") or {}).get(ZONE) or {}
        cur = "ON" if z.get("present") else "OFF"
        if prev is None:
            prev, t_change = cur, s["t"]
            continue
        if cur != prev:
            dur = s["t"] - t_change
            row = [datetime.fromtimestamp(s["t"]).strftime("%H:%M:%S.%f")[:-3],
                   round(s["t"], 2), cur, round(dur, 2), s.get("z4_live"), s.get("cam_pos")]
            w.writerow(row); f.flush()
            if not a.quiet:
                print(f"{row[0]}  {prev}->{cur}  after {dur:5.2f}s  z4_live {s.get('z4_live')}", flush=True)
            prev, t_change = cur, s["t"]


if __name__ == "__main__":
    main()
