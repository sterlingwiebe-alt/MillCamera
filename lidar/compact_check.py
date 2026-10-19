r"""Verification + timing harness for lidar/fast_compact.py.

Compares the installed scansegmentapi Compact parser with the vectorised
replacement, element for element, on a set of datagrams, and times both.

Payload sources (any combination):
  --raw FILE     datagrams recorded by the capture tool
                 (lidar/fast_compact.py raw dump: interlace_capture.py
                 --raw-dump N writes <dump>/raw_payloads.bin at start-up)
  --pkl FILE     a burst capture (cap_*.pkl) or recording: every stored
                 segment dict is serialised BACK to the Compact wire format
                 (encode_segment) -- real sensor values, real module layout.
                 The encoder is itself checked: the original parser must
                 reproduce the stored segment from the synthesised bytes.
  --random N     N synthetic segments with random layout (1-3 echoes, 1-16
                 layers, 1-120 beams, rssi/theta/properties present or not,
                 scaling factors other than 1) and random 16-bit values,
                 covering the format variants the sensor is not configured
                 to send.

    python lidar/compact_check.py verify --pkl lidar/z4captures/interlaced/cap_01_134722.pkl --random 200
    python lidar/compact_check.py verify --raw lidar/z4captures/interlaced/raw_payloads.bin
    python lidar/compact_check.py bench  --pkl lidar/z4captures/interlaced/cap_01_134722.pkl
"""
from __future__ import annotations

import argparse
import pickle
import struct
import sys
import time
import zlib
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from lidar import fast_compact                      # noqa: E402
import scansegmentapi.compact as compact            # noqa: E402

STX = b"\x02\x02\x02\x02"


# ------------------------------------------------------------------ encoder

def encode_module(meta, recs, next_module_size, reserved=0):
    """Metadata block + beam block of one module. ``recs`` is a structured
    array of shape (beams, layers) in fast_compact's record dtype."""
    L = int(meta["NumberOfLinesInModule"])
    out = struct.pack("<QQIIII", int(meta["SegmentCounter"]), int(meta["FrameNumber"]),
                      int(meta["SenderId"]), L, int(meta["NumberOfBeamsPerScan"]),
                      int(meta["NumberOfEchosPerBeam"]))
    out += struct.pack(f"<{L}Q", *[int(x) for x in meta["TimestampStart"]])
    out += struct.pack(f"<{L}Q", *[int(x) for x in meta["TimestampStop"]])
    out += struct.pack(f"<{L}f", *[float(x) for x in meta["Phi"]])
    out += struct.pack(f"<{L}f", *[float(x) for x in meta["ThetaStart"]])
    out += struct.pack(f"<{L}f", *[float(x) for x in meta["ThetaStop"]])
    out += struct.pack("<f", float(meta["DistanceScalingFactor"]))
    out += struct.pack("<I", int(next_module_size))
    out += struct.pack("<BBBB", int(meta["Availability"]), int(meta["DataContentEchos"]),
                       int(meta["DataContentBeams"]), int(reserved))
    out += recs.tobytes()
    return out


def encode_payload(header, modules):
    """Whole datagram: STX + header + modules + CRC32, as the sensor sends it
    (parse_payload is given the datagram verbatim by FrameBuffer; it ignores
    the trailing CRC because the module sizes bound what it reads)."""
    bodies = []
    for i, (meta, recs) in enumerate(modules):
        nxt = 0
        if i + 1 < len(modules):
            nxt = len(encode_module(*modules[i + 1], 0))
        bodies.append(encode_module(meta, recs, nxt))
    first = len(bodies[0]) if bodies else 0
    out = STX + struct.pack("<IQQII", int(header["CommandId"]), int(header["TelegramCounter"]),
                            int(header["TimestampTransmit"]), int(header["Version"]), first)
    out += b"".join(bodies)
    out += struct.pack("<I", zlib.crc32(out))
    return out


def records_from_segment_data(meta, layers):
    """Rebuild the wire records (beams, layers) from a parsed module's
    SegmentData. Inverts the parser's arithmetic exactly; raises if a value
    does not round-trip (which would mean the pkl was not produced by this
    parser)."""
    L = int(meta["NumberOfLinesInModule"]); B = int(meta["NumberOfBeamsPerScan"])
    E = int(meta["NumberOfEchosPerBeam"])
    dt = fast_compact._record_dtype(E, bool(meta["HasRssi"]), bool(meta["HasProperties"]),
                                    bool(meta["HasTheta"]))
    recs = np.zeros((B, L), dtype=dt)
    scale = float(meta["DistanceScalingFactor"])
    for l, layer in enumerate(layers):
        for e in range(E):
            d = np.rint(np.asarray(layer["Distance"][e], float) / scale)
            if not np.array_equal(d * scale, layer["Distance"][e]):
                raise ValueError("distance does not round-trip")
            recs[f"d{e}"][:, l] = d.astype(np.uint16)
            if meta["HasRssi"]:
                recs[f"r{e}"][:, l] = np.asarray(layer["Rssi"][e]).astype(np.uint16)
        if meta["HasProperties"]:
            recs["p"][:, l] = np.asarray(layer["Properties"]).astype(np.uint8)
        if meta["HasTheta"]:
            t = np.rint(np.asarray(layer["ChannelTheta"], float) * 5215.0 + 16384.0)
            if not np.array_equal((t - 16384.0) / 5215.0, layer["ChannelTheta"]):
                raise ValueError("theta does not round-trip")
            recs["t"][:, l] = t.astype(np.uint16)
    return recs


def encode_segment(seg):
    """Serialise a parsed segment dict (as stored in cap_*.pkl) back to a
    Compact datagram."""
    header = {k: seg[k] for k in ("CommandId", "TelegramCounter", "TimestampTransmit", "Version")}
    modules = [(m, records_from_segment_data(m, m["SegmentData"])) for m in seg["Modules"]]
    return encode_payload(header, modules)


def random_segment(rng, idx):
    """A random-layout segment with random beam values (for the format
    variants the sensor does not send)."""
    n_mod = int(rng.integers(1, 3))
    modules = []
    for k in range(n_mod):
        L = int(rng.integers(1, 17)); B = int(rng.integers(1, 121)); E = int(rng.integers(1, 4))
        has_rssi, has_props, has_theta = (bool(x) for x in rng.integers(0, 2, 3))
        scale = float(np.float32(rng.choice([1.0, 0.5, 0.25, 2.0, 0.001, 1.7, 1e-3 * 3])))
        meta = {
            "SegmentCounter": int(rng.integers(0, 2**40)), "FrameNumber": int(rng.integers(0, 2**40)),
            "SenderId": int(rng.integers(0, 2**32)),
            "NumberOfLinesInModule": L, "NumberOfBeamsPerScan": B, "NumberOfEchosPerBeam": E,
            "TimestampStart": rng.integers(0, 2**63, L, dtype=np.int64),
            "TimestampStop": rng.integers(0, 2**63, L, dtype=np.int64),
            "Phi": rng.uniform(-1, 1, L).astype(np.float32).astype(np.float64),
            "ThetaStart": rng.uniform(-3.2, 3.2, L).astype(np.float32).astype(np.float64),
            "ThetaStop": rng.uniform(-3.2, 3.2, L).astype(np.float32).astype(np.float64),
            "DistanceScalingFactor": scale, "Availability": int(rng.integers(0, 256)),
            "DataContentEchos": 1 | (2 if has_rssi else 0),
            "DataContentBeams": (1 if has_props else 0) | (2 if has_theta else 0),
            "HasRssi": has_rssi, "HasProperties": has_props, "HasTheta": has_theta,
        }
        dt = fast_compact._record_dtype(E, has_rssi, has_props, has_theta)
        raw = rng.integers(0, 256, B * L * dt.itemsize, dtype=np.uint8)
        # make sure the extremes occur
        raw[: min(8, raw.size)] = 0
        raw[-min(8, raw.size):] = 255
        recs = np.frombuffer(raw.tobytes(), dtype=dt).reshape(B, L)
        modules.append((meta, recs))
    header = {"CommandId": 1, "TelegramCounter": idx, "TimestampTransmit": int(rng.integers(0, 2**40)),
              "Version": 4}
    return encode_payload(header, modules)


# ----------------------------------------------------------------- compare

def _bits(a):
    if a.dtype.kind == "f":
        return a.view(np.uint64 if a.dtype.itemsize == 8 else np.uint32)
    return a


def compare(a, b, path="", stats=None, diffs=None):
    """Recursive, exact comparison of two parser outputs. Floats are
    compared by bit pattern (NaN == NaN only if the same NaN). Returns the
    number of leaf elements compared; appends mismatch descriptions to
    ``diffs``."""
    if stats is None:
        stats = {"elements": 0, "arrays": 0, "scalars": 0}
    if diffs is None:
        diffs = []
    if type(a) is not type(b):
        diffs.append(f"{path}: type {type(a).__name__} != {type(b).__name__}")
        return stats, diffs
    if isinstance(a, dict):
        if list(a.keys()) != list(b.keys()):
            diffs.append(f"{path}: keys {list(a.keys())} != {list(b.keys())}")
            return stats, diffs
        for k in a:
            compare(a[k], b[k], f"{path}.{k}", stats, diffs)
    elif isinstance(a, (list, tuple)):
        if len(a) != len(b):
            diffs.append(f"{path}: len {len(a)} != {len(b)}")
            return stats, diffs
        for i, (x, y) in enumerate(zip(a, b)):
            compare(x, y, f"{path}[{i}]", stats, diffs)
    elif isinstance(a, np.ndarray):
        stats["arrays"] += 1
        if a.dtype != b.dtype or a.shape != b.shape:
            diffs.append(f"{path}: dtype/shape {a.dtype}{a.shape} != {b.dtype}{b.shape}")
            return stats, diffs
        for flag in ("C_CONTIGUOUS", "WRITEABLE"):
            if a.flags[flag] != b.flags[flag]:
                diffs.append(f"{path}: flag {flag} {a.flags[flag]} != {b.flags[flag]}")
        ne = int(np.count_nonzero(_bits(a) != _bits(b)))
        stats["elements"] += a.size
        if ne:
            diffs.append(f"{path}: {ne} of {a.size} elements differ (bitwise)")
    else:
        stats["scalars"] += 1
        stats["elements"] += 1
        same = (a == b)
        if isinstance(a, float):
            same = struct.pack("<d", a) == struct.pack("<d", b)
        if not same:
            diffs.append(f"{path}: {a!r} != {b!r}")
    return stats, diffs


# ----------------------------------------------------------------- sources

def load_pkl_segments(path):
    obj = pickle.load(open(path, "rb"))
    frames = obj.get("frames", obj) if isinstance(obj, dict) else obj
    segs = []
    if isinstance(frames, dict):
        for fn in sorted(frames):
            segs.extend(frames[fn])
    else:
        segs.extend(frames)
    return [s for s in segs if isinstance(s, dict) and "Modules" in s]


def gather(args):
    """-> list of (label, payload, reference_segment_or_None)."""
    items = []
    for p in args.raw or []:
        for i, d in enumerate(fast_compact.iter_raw_dump(p)):
            items.append((f"{Path(p).name}[{i}]", d, None))
    for p in args.pkl or []:
        for i, seg in enumerate(load_pkl_segments(p)):
            items.append((f"{Path(p).name}[{i}]", encode_segment(seg), seg))
    if args.random:
        rng = np.random.default_rng(args.seed)
        for i in range(args.random):
            items.append((f"random[{i}]", random_segment(rng, i), None))
    return items


# ---------------------------------------------------------------- commands

def cmd_verify(args):
    items = gather(args)
    if not items:
        sys.exit("no payloads (give --raw, --pkl and/or --random)")
    orig = fast_compact.original_read_beam_data()
    fast = fast_compact._read_beam_data
    stats = {"elements": 0, "arrays": 0, "scalars": 0}
    enc_stats = {"elements": 0, "arrays": 0, "scalars": 0}
    diffs, enc_diffs = [], []
    n_mod = 0
    nbytes = 0
    for label, payload, ref in items:
        nbytes += len(payload)
        compact._read_beam_data = orig
        a = compact.parse_payload(payload)
        compact._read_beam_data = fast
        b = compact.parse_payload(payload)
        n_mod += len(a["Modules"])
        compare(a, b, label, stats, diffs)
        if ref is not None:         # encoder round-trip against the stored segment
            compare(ref, a, "encode:" + label, enc_stats, enc_diffs)
    compact._read_beam_data = orig
    print(f"payloads: {len(items)} ({nbytes / 1e6:.2f} MB), modules: {n_mod}")
    print(f"original vs fast: {stats['elements']:,} elements compared "
          f"({stats['arrays']} arrays, {stats['scalars']} scalars); "
          f"{len(diffs)} mismatches")
    for d in diffs[:30]:
        print("  MISMATCH", d)
    if enc_stats["elements"]:
        print(f"encoder round-trip (stored segment vs original parser on synthesised bytes): "
              f"{enc_stats['elements']:,} elements; {len(enc_diffs)} mismatches")
        for d in enc_diffs[:30]:
            print("  MISMATCH", d)
    ok = not diffs and not enc_diffs
    print("RESULT:", "IDENTICAL" if ok else "DIFFERENT")
    return 0 if ok else 1


def _time_all(payloads, repeats):
    best = None
    for _ in range(repeats):
        t0 = time.perf_counter()
        for p in payloads:
            compact.parse_payload(p)
        dt = time.perf_counter() - t0
        best = dt if best is None else min(best, dt)
    return best


def cmd_bench(args):
    items = gather(args)
    if not items:
        sys.exit("no payloads (give --raw, --pkl and/or --random)")
    payloads = [p for _, p, _ in items]
    frames = set()
    beams = 0
    for p in payloads:
        seg = compact.parse_payload(p)
        for m in seg["Modules"]:
            frames.add(m["FrameNumber"])
            beams += m["NumberOfBeamsPerScan"] * m["NumberOfLinesInModule"]
    nf = max(len(frames), 1)
    orig = fast_compact.original_read_beam_data()
    fast = fast_compact._read_beam_data
    print(f"{len(payloads)} segments, {nf} frames ({len(payloads) / nf:.1f} segments/frame), "
          f"{beams / len(payloads):.0f} beams/segment, best of {args.repeats}")
    out = {}
    for name, fn in (("original", orig), ("fast", fast)):
        compact._read_beam_data = fn
        _time_all(payloads[:20], 1)         # warm up
        t = _time_all(payloads, args.repeats)
        out[name] = t
        print(f"  {name:9s}: {t / len(payloads) * 1e6:8.1f} us/segment   "
              f"{t / nf * 1e3:7.2f} ms/frame   ({t:.3f} s total)")
    compact._read_beam_data = orig
    print(f"  speed-up: {out['original'] / out['fast']:.1f}x")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cmd", choices=["verify", "bench"])
    ap.add_argument("--raw", action="append", help="raw datagram dump (repeatable)")
    ap.add_argument("--pkl", action="append", help="cap_*.pkl / recording (repeatable)")
    ap.add_argument("--random", type=int, default=0, help="N random-layout segments")
    ap.add_argument("--seed", type=int, default=20261019)
    ap.add_argument("--repeats", type=int, default=5)
    args = ap.parse_args()
    sys.exit(cmd_verify(args) if args.cmd == "verify" else cmd_bench(args))


if __name__ == "__main__":
    main()
