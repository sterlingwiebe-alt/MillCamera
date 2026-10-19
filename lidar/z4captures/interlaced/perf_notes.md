# FlowTracker per-frame cost on the mill PC (2026-10-15)

Machine: i7-5500U (2 cores / 4 threads, 2.4 GHz), Windows 10, Python 3.12,
numpy 2.5.1, venv `C:\Claude\sterl\.venvs\camera-app`. All timings below were
taken while the live capture tool was running in the background, so they
carry its CPU contention (the max values are scheduling outliers).

Method: `lidar/perf_replay.py` drives the real `FlowTracker.run()` loop over
the 9 recordings `cap_01_*.pkl` (278 frames of 12 segments; 237 frames with
a zone-4 log, 69 inventory frames) with a fake frame buffer and a synthetic
20 Hz clock, times `_z4_log` per frame and dumps every frame's zone-4 result,
presence dict and published zones for comparison.

## Where the time went (before)

cProfile over 63 frames (2 recordings), 14.8 s total, 235 ms per frame under
the profiler:

| part | share | what |
|---|---|---|
| RANSAC in `detect_logs` | ~60 % | 300 iterations per log candidate, each a `rng.choice` (18 us) plus 6 small numpy ops on ~40-130 centers (37 us); ~2.8 candidates per call, 1 call per frame + 1 deck call per 4th frame |
| circle fits (`round_runs` -> svd + `kasa_circle`/lstsq) | ~22 % | ~120 runs per zone pass, ~170 us each, dominated by numpy wrapper overhead not LAPACK |
| `zone_line_points` | ~16 % | the 192 scan lines of a frame converted beam-by-line to xyz again for EVERY zone pass: 1 pass on a zone-4 frame, 9 on an inventory frame (zone-4 band, deck, 7 presence bands) |
| rest | ~2 % | refine (medians/percentiles), publish |

Real (unprofiled) per-frame cost, best of 2 runs:

| frame kind | mean | median | p90 |
|---|---|---|---|
| zone-4 only (3 of 4) | 70 ms | 70 ms | 87 ms |
| inventory (every 4th) | 383 ms | 386 ms | 437 ms |
| all | 148 ms | 74 ms | 394 ms |

At 148 ms per frame the tracker ran at ~7 fps against a 20 Hz stream, which
is the 0.5-2.5 s state lag and the Vision_Transport[0] flicker.

## Changes (results bit-identical, see verification)

`lidar/arcfit.py`
- `seg_points` / `frame_points`: the xyz conversion of a segment is done once
  per segment object (all 16 lines in one vectorised pass, same per-element
  arithmetic as the old loop) and cached (bounded, keyed on object identity
  with a `Modules`-identity check so pooled/copied segments never hit a stale
  entry; locked for the capture tool's second thread). `zone_line_points` is
  now a mask + `reduceat` over the cached points.
- `zone_runs`: the runs of a zone pass over the whole frame in a handful of
  array ops (one `diff`/`norm` for all lines, line ends forced as cuts),
  same runs in the same order as the nested `zone_line_points`/`line_runs`.
- `fit_runs` / `_fit_group`: circle fits batched per run LENGTH, calling
  numpy's own LAPACK gufuncs (`svd_s`, `lstsq`) once per stack. Per-matrix
  results are bit-identical to the single calls (checked); the elementwise
  algebra is the same as `kasa_circle`. Memoised on the run's exact bytes so
  overlapping zone passes share fits.
- `zone_round_runs` (batched) replaces the nested loops in the hot callers;
  `round_runs` keeps the single-run path for the other tools (now memoised).
- `prefit_zones`: all nine zone passes of an inventory frame fitted in ONE
  batch (fewer, larger length groups).

`lidar/segment.py`
- RANSAC evaluated for all 300 hypothesis pairs at once, in chunks of 32
  pairs into preallocated buffers. Same pairs, same arithmetic (batched
  matmul = the same BLAS gemv per pair as `q @ d`; norm = sqrt of summed
  squares), first pair reaching the max inlier count wins as before.
- `_pairs_vectorised`: the 300 `rng.choice(n, 2, replace=False)` draws
  reproduced from the generator's raw 64-bit stream (numpy's Floyd sampling
  with 32-bit Lemire bounded draws + the 2-element shuffle), leaving the
  generator in the same state. Validated against `rng.choice` on 1500
  random states/sizes (pairs and final state identical). A self-test at
  first use falls back to the plain loop if a numpy upgrade ever changes
  the algorithm. 0.15 ms per round instead of 5.6 ms.

`lidar/flow_track.py`
- `_presence_bands()` factored out (same windows/pads as before) and used
  by `_update_presence` and the inventory-frame prefit.
- `_update_presence` uses `zone_round_runs` (same gating, same order).

Not touched: `plc_out.py`, the PLC layout, `probe_zones.py`, `interlace.py`,
`DECK_EVERY`, any threshold or window.

## After

Real per-frame cost, best of 3 runs (`lidar/perf_replay.py --repeat 3`):

| frame kind | before mean | after mean | after median | after p90 | after max |
|---|---|---|---|---|---|
| zone-4 only (3 of 4) | 70 ms | 12.0 ms | 11.7 ms | 15.3 ms | 19.8 ms |
| inventory (every 4th) | 383 ms | 51 ms | 49 ms | 60 ms | 108 ms |
| all | 148 ms | 21.7 ms | 12.8 ms | 52 ms | 108 ms |

A 4-frame cycle costs ~87 ms of CPU for 200 ms of stream, so the tracker
keeps up at 20 Hz; the worst case is one skipped frame after an inventory
frame (the loop always takes the latest complete frame). Published state lag
should drop from 0.5-2.5 s to <= ~60 ms.

Remaining cost of an inventory frame (real, measured in isolation): xyz
conversion 3.6 ms, run collection for 9 passes 3.6 ms, batched fits of
~380 runs in ~27 length groups 12.7 ms, zone-4 detect (fits cached) 4.3 ms,
deck detect 17 ms (= ~8 RANSAC rounds of 300 pairs x ~130 centers, plus the
refine medians), presence gating 3 ms. The deck RANSAC is now the floor:
cutting it further without changing results would need fewer centers or
fewer rounds, i.e. a behaviour change.

## Equivalence check

1. Tracker replay, 278 frames: per-frame zone-4 log (dia_arc_mm, dia_mm,
   center, n_arcs, length, height), the presence dict of every inventory
   frame (present/arcs/arc_dia/y/near per zone), the published zones dict
   and the pass bookkeeping at the end: **IDENTICAL** before vs after
   (exact equality, not just within rounding).
   `python lidar/perf_replay.py --compare before.json after.json`
2. Module level, same 278 frames plus the 64 pooled (x4 interlace,
   `scaled_gates`) cycles: `detect_logs` on the deck and zone-4 regions
   with rejects, the single-path `round_runs` arcs (len, r, center3d) over
   all six presence windows, and `detect_logs` on pooled frames:
   **IDENTICAL** (run against a copy of the original modules).
3. `interlace_capture.py --replay cap_01_052913.pkl` runs and reports as
   before (single 321 mm / pooled 344 mm on that capture).

## First live run (2026-10-17)

Capture tool started 18:56 with this code, deck idle. Published state age
sampled every 5-10 s for 15 min: 0.02-0.20 s (before: 0.5-2.5 s). No
errors. Process CPU still ~94% of one core; py-spy (20 s, 100 Hz, 1993
samples) attributes it:

| thread | share | where |
|---|---|---|
| FrameBuffer.run (UDP receive) | 79 % | `scansegmentapi/compact.py` `_read_beam_data` lines 293-306: `struct.unpack_from` per beam, element-wise assignment of 3 echoes x (distance, rssi), theta, property; ~11.5k beams per 20 Hz frame |
| FlowTracker.run | 21 % | `_fit_group` etc. (idle deck, few arcs) |
| main / PLC out / optimizer tap | < 1 % | |

So the SICK parser is now the dominant cost and the GIL budget is tight:
parser 75 % + tracker 20-45 % (idle to busy deck) exceeds one core.
Options, in order of preference: (1) vectorise `_read_beam_data` with one
`numpy.frombuffer` structured-dtype read per module (same arithmetic per
element: `uint16 * DistanceScalingFactor`, `(theta - 16384) / 5215.0`),
patched in from `lidar/` without editing the installed package, verified
against the original parser on captured raw payloads; (2) move receive +
parse into a separate process (the PC has 4 hardware threads).

## Live view contention (2026-10-17)

With `live_view.py` (640x480 substream decode + resize, 25-35% of a core)
running on the same PC, the capture tool's published state stalled for
~2 s roughly twice a minute (90 s samples at 2 Hz: max 1.9 s, several
over 1 s) while without it the max was 0.66 s. Process priority alone did
not fix it (the two threads of a hyperthreaded core share execution
resources). Pinning the capture tool to logical CPUs 0-1 and the live view
to 2-3 (`scripts/start_mill.ps1`) did: median 0.13 s, p90 0.21 s, max
0.80 s, none over 1 s. The margin is thin: the parser (above) is the lever
if the line-running load brings the stalls back.

## Live view idle decode (2026-10-17)

`live_view.py` decoded + resized the camera substream around the clock,
viewer or not (the browser only receives the MJPEG re-encode; the decode
is server-side, so watching from another PC changed nothing). Now the
MJPEG response counts itself as a camera client for as long as it is being
sent, and the `Camera` thread holds the RTSP capture open only while that
count is > 0, dropping it 5 s after the last client leaves and reopening
on the next one. Until the first decoded frame the stream carries the
zone overlay on a black picture with a "camera starting..." label.

Measured on the dev PC (interpreter process, % of one core; the venv
`python.exe` is a launcher, the child was measured), `--cam cam0_1`:

| state | before | after |
|---|---|---|
| no client, 30 s | 29.9 % | 0.1 % |
| one MJPEG client (urllib, 10 fps), 25 s | 40.7 % | 38.9 % |
| 8 s after the client left, 30 s | (29.9 %) | 0.4 % |

Latency for a client on `127.0.0.1`: first overlay frame 0.2 s, camera
picture 1.2-2.5 s after that (RTSP open + first decoded frame). Reconnect
after the grace period behaves the same. Expect the mill PC idle figure to
drop from the 25-35 % above to ~0 once the MillCamera task is restarted
with this version (not yet done; the supervised instance still runs the
old code).

Gotcha found on the way: a `finally` in the sync MJPEG generator did NOT
run on client disconnect (decode stayed at 25 %): uvicorn 0.52 advertises
ASGI 2.3, so Starlette cancels the sending task and the abandoned
generator is left for the cyclic GC. The count therefore lives in a
`StreamingResponse` subclass whose `__call__` wraps the whole send in
try/finally (and closes the generator explicitly).

Note for measuring with `urllib` on Windows: `localhost` costs a 2 s
IPv6-first connect timeout against a server bound to `0.0.0.0`; use
`127.0.0.1`.

## Notes / options not taken

- Caches: `_FIT_CACHE` (8192 fits, cleared when full), `_PTS_CACHE` (64
  segments), `_FRAME_CACHE` (4 frames). They hold references to segment
  dicts of the last few frames only. Nothing is written into the segment
  dicts, so pickled captures are unchanged.
- If an inventory frame must also be well under 50 ms, the exact-result
  options are gone; the behaviour-changing ones are `DECK_EVERY` (inventory
  cadence), fewer RANSAC iterations, or an exhaustive-pairs RANSAC (which
  would be deterministic and usually better, but ties/misses differ).
- One-off: the first `detect_logs` call runs the pair-draw self-test
  (~60 ms).

## SICK parser vectorised (2026-10-19)

Option (1) from "First live run": `scansegmentapi/compact.py`
`_read_beam_data` (struct.unpack_from per beam, element-wise assignment)
is replaced by `lidar/fast_compact.py`: one `numpy.frombuffer` read per
module with a packed structured dtype in the wire order (d0 r0 d1 r1 ...
prop theta), each field transposed to (layers, beams) and converted to
float64 once, then the same arithmetic per element as the original
(`uint16 * DistanceScalingFactor`, `(theta - 16384) / 5215.0`, float64
both times, so the results are the same IEEE operations on the same
operands). Each layer's arrays are the contiguous rows of those matrices
(same dtype, shape, C-contiguous and writeable as before; they share a
base array instead of owning their memory, which nothing downstream
depends on). Channels a module does not carry become NaN exactly as the
original's `arr[i] = None` did. The package is not edited:
`fast_compact.install()` monkey-patches the function into the module, and
`interlace_capture.py live()` calls it before the receiver thread starts
(`--no-fast-parse` keeps the package's parser for A/B runs). The
metadata reader (`_read_meta_data`, ~1/3 of what is left) is untouched.

Verification (`lidar/compact_check.py verify`): the original and the
patched parser run on the same datagrams and the outputs are compared
recursively, floats by bit pattern, plus container types, key order,
dtypes, shapes and the C_CONTIGUOUS/WRITEABLE flags. Datagram sources:

| source | segments | elements compared | mismatches |
|---|---|---|---|
| `cap_01_134722.pkl` (10-19), every stored segment serialised back to the Compact wire format by `compact_check.encode_segment` | 309 | 2,407,728 (and the encoder round-trip, stored segment vs original parser on the synthesised bytes: 0 mismatches) | 0 |
| `cap_01_041611.pkl` + `cap_01_052913.pkl` (10-15), same way | 754 | 5,875,168 | 0 |
| random layouts: 1-3 echoes, 1-16 layers, 1-120 beams, rssi / theta / properties present or absent, scaling factors 0.001-2, random 16-bit values incl. 0 and 65535 | 800 | ~3.9 M | 0 |
| raw dump file written through the new hook (25 datagrams, read back byte-identical) | 25 | 194,800 | 0 |

12.35 M elements in all, identical. Real sensor datagrams were not yet
available without binding the sensor's port: the supervised tool holds
it. `interlace_capture.py` now records the first 240 datagrams verbatim
to `<dump>/raw_payloads.bin` at every start (`--raw-dump N`, 0 = off,
3.6 MB, overwritten), so after the next restart
`python lidar/compact_check.py verify --raw lidar/z4captures/interlaced/raw_payloads.bin`
repeats the check on the real thing (see below).

Parse time on this PC (`compact_check.py bench`, `cap_01_134722.pkl`:
309 segments, 26 frames, 960 beams and 2 modules per segment, best of 5,
taken with the live tool running at ~100 % of a core on CPUs 0-1):

| | per segment | per frame (11.9 segments) |
|---|---|---|
| original `_read_beam_data` | 5015 us | 59.6 ms |
| `fast_compact` | 391 us | 4.65 ms |
| speed-up | 12.8x | |

cProfile of the patched path: 56 % in `_read_beam_data` (astype and the
per-layer row views), 35 % in the package's `_read_meta_data` (17 small
int.from_bytes / struct reads per module), rest `parse_payload` plumbing.

Live, before (old parser, pid 4328 started 13:57:56, 120 s at 5 Hz,
2026-10-19 14:29): interpreter CPU 99.3 % of a core; published state age
median 0.43 s, p90 1.50 s, max 2.61 s, 115 of 558 samples over 1 s.

Live, after: pending the MillCamera restart (the supervised instance
runs the old code until then); numbers to be added here.
