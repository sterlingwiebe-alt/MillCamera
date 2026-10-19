"""Vectorised beam-data parser for the SICK Compact format (scansegmentapi).

The installed ``scansegmentapi.compact._read_beam_data`` unpacks every beam
with ``struct.unpack_from`` and assigns each value element by element: at
20 Hz and ~11.5k beams per frame that was 75-80 % of the capture tool's one
core (perf_notes.md, "First live run (2026-10-17)"). This module replaces it
with one ``numpy.frombuffer`` structured-dtype read per module and the same
arithmetic per element (``uint16 * DistanceScalingFactor``,
``(theta - 16384) / 5215.0``, both in float64 exactly as before), so the
output is bit-identical (lidar/compact_check.py verifies that).

Nothing in the installed package is edited: ``install()`` monkey-patches the
function into the ``scansegmentapi.compact`` module. Call it before the UDP
receiver thread starts (``parse_payload`` reaches ``_read_beam_data`` through
the module globals, so the patch takes effect for every later call).

    from lidar import fast_compact
    fast_compact.install()                               # the fast parser
    fast_compact.install(raw_dump=path, raw_count=240)   # + record the first
                                                         # 240 datagrams

The raw dump (``iter_raw_dump`` reads it back) is the verification set for
lidar/compact_check.py: real sensor datagrams, parsed by the original and
the patched parser and compared element for element.
"""
from __future__ import annotations

import logging
import struct
from pathlib import Path

import numpy as np

logger = logging.getLogger("scansegmentapi")

RAW_MAGIC = b"SICKRAW1"

_DTYPES: dict[tuple, np.dtype] = {}
_ORIGINAL = None            # the package's _read_beam_data, kept for uninstall()


def _record_dtype(num_echos, has_rssi, has_props, has_theta):
    """Packed per-beam record, the same layout as the parser's struct format
    string ``'<' + E*'H' (+ E*'H') (+ 'B') (+ 'H')`` with its interleaved
    distance/rssi order: d0 r0 d1 r1 ... prop theta."""
    key = (num_echos, has_rssi, has_props, has_theta)
    dt = _DTYPES.get(key)
    if dt is None:
        fields = []
        for e in range(num_echos):
            fields.append((f"d{e}", "<u2"))
            if has_rssi:
                fields.append((f"r{e}", "<u2"))
        if has_props:
            fields.append(("p", "u1"))
        if has_theta:
            fields.append(("t", "<u2"))
        dt = np.dtype(fields)           # no align= -> packed, like struct '<'
        _DTYPES[key] = dt
    return dt


def _read_beam_data(data, metadata, offset):
    """Drop-in replacement for scansegmentapi.compact._read_beam_data.

    Returns ``{'SegmentData': [per-layer dict, ...]}`` with, per layer,
    ``'Rssi'`` and ``'Distance'`` (lists of one float64 array per echo),
    ``'ChannelTheta'`` and ``'Properties'`` (float64 arrays), one value per
    beam -- the same containers, dtypes, shapes and values as the original.
    Channels the module does not carry are NaN (the original assigned None
    into a float array, which numpy stores as NaN).
    """
    num_layers = metadata["NumberOfLinesInModule"]
    num_echos = metadata["NumberOfEchosPerBeam"]
    num_beams = metadata["NumberOfBeamsPerScan"]

    if not metadata["HasDistance"]:
        logger.warning(
            f"Failed to read beam data from module. \
                No distance data available. Metadata: {metadata}")
        return None

    if num_layers == 0:
        return {'SegmentData': []}

    has_rssi = bool(metadata["HasRssi"])
    has_props = bool(metadata["HasProperties"])
    has_theta = bool(metadata["HasTheta"])
    dt = _record_dtype(num_echos, has_rssi, has_props, has_theta)
    n = num_beams * num_layers
    recs = None
    if dt.itemsize:
        if len(data) - offset < n * dt.itemsize:
            # same failure mode as struct.unpack_from on a short buffer
            raise struct.error(
                f"unpack_from requires a buffer of at least "
                f"{offset + n * dt.itemsize} bytes for unpacking {n} beam "
                f"records at offset {offset} (actual buffer size is {len(data)})")
        # Beam-major on the wire (beam 0: layer 0..L-1, beam 1: ...), so the
        # record array is (beams, layers); every per-field column is
        # transposed to (layers, beams) and converted once, and each layer's
        # array below is the contiguous row of that float64 matrix.
        recs = np.frombuffer(data, dtype=dt, count=n, offset=offset)
        recs = recs.reshape(num_beams, num_layers)

    def column(name):
        return recs[name].T.astype(np.float64, order="C")

    scale = metadata["DistanceScalingFactor"]
    distance = []
    for e in range(num_echos):
        d = column(f"d{e}")
        d *= scale                      # uint16 -> float64, then * scale (float64 mul)
        distance.append(d)
    if has_rssi:
        rssi = [column(f"r{e}") for e in range(num_echos)]
    else:
        rssi = [_nan_matrix(num_layers, num_beams) for _ in range(num_echos)]
    if has_theta:
        theta = column("t")
        theta -= 16384.0                # angleUINT = floor(angleRAD * 5215 + 16384)
        theta /= 5215.0
    else:
        theta = _nan_matrix(num_layers, num_beams)
    if has_props:
        props = column("p")
    else:
        props = _nan_matrix(num_layers, num_beams)

    result = [{
        'Rssi': [r[layer] for r in rssi],
        'Distance': [d[layer] for d in distance],
        'ChannelTheta': theta[layer],
        'Properties': props[layer],
    } for layer in range(num_layers)]
    return {'SegmentData': result}


def _nan_matrix(num_layers, num_beams):
    m = np.zeros((num_layers, num_beams))
    m[...] = None                       # exactly what the original stores
    return m


# ------------------------------------------------------------ installation

def install(fast=True, raw_dump=None, raw_count=240):
    """Patch the fast parser into scansegmentapi.compact (``fast=False``
    leaves the package's parser in place, for A/B timing). With ``raw_dump``
    (a path) the first ``raw_count`` datagrams given to ``parse_payload``
    are also written verbatim to that file (overwritten on every start)."""
    global _ORIGINAL
    import scansegmentapi.compact as compact
    if _ORIGINAL is None:
        _ORIGINAL = compact._read_beam_data
    compact._read_beam_data = _read_beam_data if fast else _ORIGINAL
    if raw_dump is not None and raw_count > 0:
        if not isinstance(compact.parse_payload, _RawDump):
            compact.parse_payload = _RawDump(compact.parse_payload, raw_dump, raw_count)
    return compact


def uninstall():
    """Restore the package's own parser (tests / A-B timing)."""
    import scansegmentapi.compact as compact
    if _ORIGINAL is not None:
        compact._read_beam_data = _ORIGINAL
    if isinstance(compact.parse_payload, _RawDump):
        compact.parse_payload.close()
        compact.parse_payload = compact.parse_payload.__wrapped__
    return compact


def original_read_beam_data():
    """The package's own _read_beam_data (whether or not install() ran)."""
    if _ORIGINAL is not None:
        return _ORIGINAL
    import scansegmentapi.compact as compact
    return compact._read_beam_data


class _RawDump:
    """parse_payload wrapper that records the first N datagrams verbatim:
    file = RAW_MAGIC, then per datagram <u4 length><bytes>."""

    def __init__(self, parse, path, count):
        self.__wrapped__ = parse
        self.path = Path(path)
        self.left = int(count)
        self.written = 0
        try:
            self._fh = open(self.path, "wb")
            self._fh.write(RAW_MAGIC)
        except Exception as exc:
            logger.warning(f"raw dump {self.path}: {exc}")
            self._fh = None
            self.left = 0

    def __call__(self, payload):
        if self.left > 0:
            try:
                self._fh.write(struct.pack("<I", len(payload)))
                self._fh.write(bytes(payload))
                self.written += 1
                self.left -= 1
                if self.left == 0:
                    self.close()
            except Exception as exc:
                logger.warning(f"raw dump {self.path}: {exc}")
                self.left = 0
                self.close()
        return self.__wrapped__(payload)

    def close(self):
        if self._fh is not None:
            try:
                self._fh.close()
            finally:
                self._fh = None


def iter_raw_dump(path):
    """Yield the datagrams of a raw dump file, in order."""
    with open(path, "rb") as fh:
        if fh.read(len(RAW_MAGIC)) != RAW_MAGIC:
            raise ValueError(f"{path}: not a raw payload dump")
        while True:
            head = fh.read(4)
            if len(head) < 4:
                return
            (n,) = struct.unpack("<I", head)
            payload = fh.read(n)
            if len(payload) < n:
                return
            yield payload
