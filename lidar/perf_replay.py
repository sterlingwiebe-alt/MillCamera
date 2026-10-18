"""Offline replay of the FlowTracker over recorded frames: per-frame timing
and an equivalence dump (zone-4 log, presence, published zones, pass state)
for before/after comparisons of the tracker path. No sensor, no PLC.

    python lidar/perf_replay.py --out before.json            # all recordings
    python lidar/perf_replay.py --out after.json --repeat 3
    python lidar/perf_replay.py --compare before.json after.json
    python lidar/perf_replay.py --profile prof.txt --pkl <one.pkl>

It drives the REAL FlowTracker.run() loop with a fake FrameBuffer /
OptimizerTap and a synthetic 20 Hz clock (frame i is at t0 + i * 0.05 s), so
pass bookkeeping is deterministic. live_state.json / flow_passes.csv go to
<repo>/lidar/z4captures/interlaced/perf_replay_dump/ (scratch).
"""
from __future__ import annotations

import argparse
import copy
import cProfile
import glob
import io
import json
import pickle
import pstats
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

import numpy as np  # noqa: E402

REC_DIR = ROOT / "lidar" / "z4captures" / "interlaced"
DEFAULT_PKLS = sorted(glob.glob(str(REC_DIR / "cap_*.pkl")))


class FakeBuf:
    def __init__(self, frames, clock):
        self.frames = frames            # list of (fn, segs)
        self.i = 0
        self.clock = clock
        self.flow = None

    def latest_complete(self):
        if self.i >= len(self.frames):
            self.flow.stop = True
            return None, None
        fn, segs = self.frames[self.i]
        self.clock.t = 1_700_000_000.0 + self.i * 0.05
        self.i += 1
        return fn, segs


class FakeTap:
    cam_pos = None

    def since(self, t):
        return []


class Clock:
    """Shim for flow_track.time: time() is synthetic, the rest delegates."""
    def __init__(self):
        self.t = 1_700_000_000.0

    def time(self):
        return self.t

    def __getattr__(self, name):
        return getattr(time, name)


def load_frames(pkls):
    out = []
    for p in pkls:
        with open(p, "rb") as f:
            d = pickle.load(f)
        fr = d["frames"] if isinstance(d, dict) and "frames" in d else d
        for fn in sorted(fr):
            out.append((int(fn), fr[fn]))
    return out


def run(pkls, dump, profile_path=None):
    import lidar.flow_track as ft
    clock = Clock()
    ft.time = clock                      # synthetic clock inside the tracker
    frames = load_frames(pkls)
    buf = FakeBuf(frames, clock)
    tap = FakeTap()
    recs = []

    class Rec(ft.FlowTracker):
        def _z4_log(self, segs):
            t0 = time.perf_counter()
            best = super()._z4_log(segs)
            dt = time.perf_counter() - t0
            inv = (self._fi % ft.DECK_EVERY == 0)
            r = {"i": len(recs), "fn": buf.frames[buf.i - 1][0], "ms": dt * 1000.0,
                 "inventory": inv,
                 "z4": None if best is None else {
                     "dia_arc_mm": best["dia_arc_mm"], "dia_mm": best["dia_mm"],
                     "center": best["center"], "n_arcs": best["n_arcs"],
                     "length_m": best["length_m"], "height_mm": best["height_mm"]}}
            if inv:
                r["presence"] = copy.deepcopy(getattr(self, "presence", {}))
                r["zones"] = copy.deepcopy(getattr(self, "last_zones", {}))
            recs.append(r)
            return best

    flow = Rec(buf, tap, dump)
    buf.flow = flow
    pr = cProfile.Profile() if profile_path else None
    if pr:
        pr.enable()
    flow.run()
    if pr:
        pr.disable()
        s = io.StringIO()
        ps = pstats.Stats(pr, stream=s).sort_stats("cumulative")
        ps.print_stats(45)
        s.write("\n\n==== by tottime ====\n")
        ps.sort_stats("tottime").print_stats(30)
        Path(profile_path).write_text(s.getvalue(), encoding="utf-8")
    cur = flow._cur
    tail = {"passes": list(flow.passes),
            "cur": None if cur is None else {"n": len(cur["dias"]), "dias": cur["dias"],
                                             "ys": [round(y, 4) for y in cur["ys"]]}}
    return recs, tail


def summarize(recs, label):
    ms = np.array([r["ms"] for r in recs])
    inv = np.array([r["inventory"] for r in recs])

    def st(a):
        return (f"n={len(a)} mean {a.mean():.1f} med {np.median(a):.1f} "
                f"p90 {np.percentile(a, 90):.1f} max {a.max():.1f} ms")
    print(f"[{label}] all frames:       {st(ms)}")
    print(f"[{label}] z4-only frames:   {st(ms[~inv])}")
    print(f"[{label}] inventory frames: {st(ms[inv])}")


def compare(a_path, b_path):
    a = json.load(open(a_path))
    b = json.load(open(b_path))
    ra, rb = a["frames"], b["frames"]
    assert len(ra) == len(rb), (len(ra), len(rb))
    n_diff = 0
    for x, y in zip(ra, rb):
        for key in ("fn", "inventory", "z4"):
            if x[key] != y[key]:
                n_diff += 1
                print(f"frame {x['i']} fn {x['fn']} {key}: {x[key]} != {y[key]}")
        if x["inventory"]:
            for key in ("presence", "zones"):
                if x[key] != y[key]:
                    n_diff += 1
                    print(f"frame {x['i']} {key} differs:\n  {x[key]}\n  {y[key]}")
    if a["tail"] != b["tail"]:
        n_diff += 1
        print("pass tail differs:\n ", a["tail"], "\n ", b["tail"])
    nz4 = sum(1 for r in ra if r["z4"])
    print(f"compared {len(ra)} frames ({nz4} with a zone-4 log, "
          f"{sum(1 for r in ra if r['inventory'])} inventory frames): "
          f"{'IDENTICAL' if n_diff == 0 else str(n_diff) + ' DIFFERENCES'}")
    summarize(ra, "before")
    summarize(rb, "after")
    return n_diff


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="")
    ap.add_argument("--profile", default="")
    ap.add_argument("--pkl", nargs="*", default=DEFAULT_PKLS)
    ap.add_argument("--compare", nargs=2)
    ap.add_argument("--label", default="run")
    ap.add_argument("--repeat", type=int, default=1, help="keep the fastest run's timings")
    args = ap.parse_args()
    if args.compare:
        sys.exit(1 if compare(*args.compare) else 0)
    dump = REC_DIR / "perf_replay_dump"
    dump.mkdir(exist_ok=True)
    for f in dump.glob("*"):
        f.unlink()
    best = None
    for k in range(args.repeat):
        recs, tail = run(args.pkl, dump, args.profile if k == 0 else None)
        summarize(recs, f"{args.label} rep{k}")
        ms = np.array([r["ms"] for r in recs]).mean()
        if best is None or ms < best[0]:
            best = (ms, recs, tail)
    if args.out:
        json.dump({"frames": best[1], "tail": best[2], "pkls": args.pkl},
                  open(args.out, "w"), indent=0)
        print("wrote", args.out)


if __name__ == "__main__":
    main()
