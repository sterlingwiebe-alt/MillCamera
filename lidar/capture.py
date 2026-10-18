"""Capture Compact-format scan segments from a multiScan/picoScan over UDP.

Usage (from the project root, camera-app venv):
    python lidar/capture.py --seconds 30 --label first_logs
    python lidar/capture.py --segments 500 --port 2115

Records raw parsed segments to lidar_recordings/<timestamp>_<label>/
    segments_XXXX.pkl   -- batches of parsed segments (pickle)
    structure.txt       -- key tree of the first segment (format discovery)
    meta.json           -- capture stats

Sensor-side prerequisite (web UI): measurement data output = Compact format,
UDP, target = this PC's IP on the sensor link, port 2115.
"""

from __future__ import annotations

import argparse
import json
import pickle
import time
from pathlib import Path

from scansegmentapi.compact import parse_payload
from scansegmentapi.udp_handler import UDPHandler

ROOT = Path(__file__).resolve().parent.parent
OUT_BASE = ROOT / "lidar_recordings"


def describe(obj, indent=0, max_depth=6, lines=None):
    """Render a compact key/type/shape tree of a parsed segment."""
    if lines is None:
        lines = []
    pad = "  " * indent
    if indent >= max_depth:
        lines.append(pad + "...")
        return lines
    if isinstance(obj, dict):
        for k, v in obj.items():
            lines.append(f"{pad}{k}: {type(v).__name__}")
            if isinstance(v, (dict, list)):
                describe(v, indent + 1, max_depth, lines)
            elif hasattr(v, "shape"):
                lines[-1] += f" shape={getattr(v, 'shape', '?')} dtype={getattr(v, 'dtype', '?')}"
    elif isinstance(obj, list):
        lines.append(f"{pad}[list of {len(obj)}]")
        if obj:
            describe(obj[0], indent + 1, max_depth, lines)
    return lines


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2115)
    ap.add_argument("--host", default="0.0.0.0")
    ap.add_argument("--seconds", type=float, default=0,
                    help="capture duration (0 = use --segments)")
    ap.add_argument("--segments", type=int, default=200,
                    help="number of segments when --seconds is 0")
    ap.add_argument("--batch", type=int, default=100,
                    help="segments per receive/save batch")
    ap.add_argument("--label", default="capture")
    args = ap.parse_args()

    stamp = time.strftime("%Y%m%d_%H%M%S")
    out = OUT_BASE / f"{stamp}_{args.label}"
    out.mkdir(parents=True, exist_ok=True)

    transport = UDPHandler(args.host, args.port, 1 << 16)
    # the library's buffer_size is only the per-recv size -- enlarge the
    # kernel receive buffer so full-rate scan segments survive parse pauses
    import socket as _socket
    try:
        transport.client.setsockopt(_socket.SOL_SOCKET, _socket.SO_RCVBUF,
                                    8 * 1024 * 1024)
    except Exception as exc:
        print("warn: could not enlarge socket buffer:", exc)
    print(f"listening on UDP {args.host}:{args.port} ... "
          f"(sensor must target this PC's IP, Compact format)")

    total = 0
    skipped = 0
    parse_errors = 0
    sizes_skipped: dict = {}
    frames = set()
    batch_idx = 0
    batch: list = []
    t0 = time.time()
    first_dumped = False
    first_bad_dumped = False

    def flush_batch():
        nonlocal batch_idx, batch
        if not batch:
            return
        with open(out / f"segments_{batch_idx:04d}.pkl", "wb") as f:
            pickle.dump({"segments": batch}, f)
        batch_idx += 1
        batch = []

    try:
        while True:
            payload = transport.receive_new_scan_segment()
            if isinstance(payload, tuple):   # some versions return (data, addr)
                payload = payload[0]
            if payload is None:
                continue
            # the stream interleaves short non-scan telegrams (IMU etc.) --
            # parse defensively and keep statistics on what we skip
            try:
                seg = parse_payload(payload)
            except Exception:
                parse_errors += 1
                sizes_skipped[len(payload)] = sizes_skipped.get(len(payload), 0) + 1
                if not first_bad_dumped:
                    (out / "first_unparsed_packet.hex").write_text(
                        payload.hex(), encoding="utf-8")
                    first_bad_dumped = True
                continue
            if not isinstance(seg, dict) or not seg:
                skipped += 1
                continue
            if not seg.get("Modules"):
                # valid Compact telegram but no measurement modules --
                # header/keepalive only, not scan data
                skipped += 1
                continue
            if not first_dumped:
                (out / "structure.txt").write_text(
                    "\n".join(describe(seg)), encoding="utf-8")
                first_dumped = True
                print("first segment structure written")
            batch.append(seg)
            total += 1
            frames.add(seg.get("FrameNumber",
                       seg.get("TelegramCounter", total)))
            if len(batch) >= args.batch:
                flush_batch()
                dt = time.time() - t0
                print(f"  batch {batch_idx}: {total} segments, "
                      f"{len(frames)} frames, {total / dt:.0f} seg/s, "
                      f"{parse_errors} non-scan pkts")
            dt = time.time() - t0
            if args.seconds and dt >= args.seconds:
                break
            if not args.seconds and total >= args.segments:
                break
    except KeyboardInterrupt:
        print("stopped by user")
    finally:
        flush_batch()
        try:
            transport.close()
        except Exception:
            pass

    dt = time.time() - t0
    meta = {"segments": total, "frames": len(frames), "duration_s": round(dt, 2),
            "segments_per_s": round(total / dt, 1) if dt else 0,
            "parse_errors": parse_errors, "empty_or_skipped": skipped,
            "skipped_packet_sizes": sizes_skipped,
            "port": args.port, "label": args.label}
    (out / "meta.json").write_text(json.dumps(meta, indent=2), encoding="utf-8")
    print(f"done: {meta} -> {out}")


if __name__ == "__main__":
    main()
