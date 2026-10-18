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
