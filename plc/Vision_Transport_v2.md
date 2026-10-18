# Vision_Transport layout v2 — LiDAR log deck (approved and live since 2026-10-15 08:28)

Written by the LiDAR tool (`lidar/interlace_capture.py --plc-out`) into the
existing controller-scope tags:

| Tag | Type | Written |
|---|---|---|
| `Vision_Transport` | `DINT[40]` | all 40 elements in ONE write, 5 times per second |
| `Vision_Heartbeat` | `DINT` | increments once per second while the tool runs |

`Vision_Transport[39]` (ResultId) changes last in each write; when it changes,
everything else is already current. Latch on it as before. `[23]` is a
separate counter that changes only when a zone-4 log has finished its pass,
for logic that wants one event per log rather than one per write.

## Elements

| Index | Meaning | Values |
|---|---|---|
| `[0]` | LiDAR state fresh | 1 = state updated within the last second, 0 = stale (do not trust 2..29) |
| `[1]` | Zone count | 7 |
| `[2]` | Presence zone 1, outfeed conveyor | always 0 (no presence logic for zone 1) |
| `[3]` | Presence zone 2, cam waiting | 0 empty, 1 present, 2 cam sweeping (no call) |
| `[4]` | Presence zone 3, cam staging | 0 empty, 1 present, 2 cam sweeping (no call) |
| `[5]` | Presence zone 4, top of screw | 0 empty, 1 present |
| `[6]` | Presence zone 5, mid screw | 0 empty, 1 present (multi-piece station) |
| `[7]` | Presence zone 6, screw deck | 0 empty, 1 present |
| `[8]` | Presence zone 7, infeed deck | 0 empty, 1 present |
| `[9]` | Diameter zone 1 | -1 (none) |
| `[10]` | Diameter zone 2, mm | -1 when empty; window arcs, indicative |
| `[11]` | Diameter zone 3, mm | -1 when empty; indicative, reads high with the log on the cams |
| `[12]` | **Diameter zone 4, mm** | **live median of the pass in progress; -1 when empty. The measurement.** |
| `[13]` | Diameter zone 5, mm | -1 when empty; indicative (reads low, multi-piece) |
| `[14]` | Diameter zone 6, mm | -1 when empty; indicative |
| `[15]` | Diameter zone 7, mm | -1 when empty; indicative |
| `[16]` | Zone-4 pass in progress: frames | 0 when empty; ~20 per second while a log is in the window |
| `[17]` | Zone-4 pass in progress: moving | 1 moving, 0 stopped |
| `[18]` | Last completed zone-4 pass: diameter, mm | final value for the log that just left zone 4 |
| `[19]` | Last completed pass: frames | quality indicator (< 20 = thin) |
| `[20]` | Last completed pass: spread, mm | interquartile range of the per-frame fits (> 15 = wide) |
| `[21]` | Last completed pass: moving | 1 moving, 0 stopped |
| `[22]` | Last completed pass: confidence | 1 clean, 0 low (thin or wide) |
| `[23]` | **Pass counter** | increments when a zone-4 pass completes; latch 18..22 on change |
| `[24]` | Cam angle | degrees x 10, from CamKicker_Servo.ActualPosition as read by the tool; -1 if unread |
| `[25]` | Cam-gate bits | bit 0 = zone 2 blind (lobes sweeping it), bit 1 = zone 3 blind |
| `[26]` | Last optimizer pairing: LiDAR, mm | diagnostic only |
| `[27]` | Last optimizer pairing: SED, mm | diagnostic only |
| `[28]` | Last optimizer pairing: LED, mm | diagnostic only |
| `[29]` | Last optimizer pairing: band error, mm | 0 = inside [SED, MID] |
| `[30]`..`[37]` | reserved | 0 |
| `[38]` | Layout version | 2 |
| `[39]` | **ResultId** | increments per write; changes LAST |

## Notes for the PLC side

- Presence 2 ("cam sweeping") in zones 2 and 3 means the LiDAR cannot judge
  that zone at that cam angle; hold the previous state or treat as unknown.
  Zone 3 is judged at cam angles 50..330, zone 2 at 80..150.
- The zone-4 diameter in `[12]` is the running median and settles within a
  few frames; `[18]` with `[23]` is the clean per-log value to use for
  piece-to-piece gap decisions. Its accuracy against the mill optimizer on
  2026-10-15: inside the SED..MID band on ~96 % of clean passes, mean error
  3 mm, LiDAR reads ~12 mm above SED with ~14 mm scatter.
- Nothing is written anywhere else on the controller. The tool reads
  `Vision_Capture`, `TestPtr`, `TestLog[]` and `CamKicker_Servo.ActualPosition`.
- Heartbeat stopping for > 3 s = tool down; `[0]` = 0 = LiDAR stream down.

## Change log

- v1: template-inspection layout (`plc/Rockwell_Integration.md`), still used by the vision app.
- v2 (approved 2026-10-15, writing since 08:28): this document; writer is `lidar/plc_out.py`,
  started by `lidar/interlace_capture.py --plc-out`. `[24]` and `[25]` are sampled at the same
  instant as the presence states. `Vision_Heartbeat` still to be created on the controller.
