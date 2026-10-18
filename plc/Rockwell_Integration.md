# Rockwell ControlLogix integration (EtherNet/IP)

The vision/LiDAR app writes results to a ControlLogix (L8x-class) over
EtherNet/IP CIP explicit messaging (`pylogix`). **No software is required on
the PC side** — no RSLinx, no FactoryTalk. The PC only needs a network route
to the controller.

## Tags to create in Studio 5000 (controller scope)

| Tag | Data type | External Access |
|---|---|---|
| `Vision_Transport` | `DINT[40]` | **Read/Write** |
| `Vision_Heartbeat` | `DINT` | **Read/Write** |

Both must be **controller-scope** (not program-scope) and have
**External Access = Read/Write** — the two most common reasons a
commissioning day stalls.

## Transport layout — `Vision_Transport` (DINT[40])

Written by the app in ONE write; `[39]` (ResultId) changes LAST, so when it
changes, everything else is already current.

| Index | Meaning |
|---|---|
| `[0]` | Pass (0/1) — overall verdict |
| `[1]` | RoiCount — zones evaluated |
| `[2]..[9]` | RoiPass 1..8 (0/1) |
| `[10]..[17]` | MatchX 1..8 (pixels/units) |
| `[18]..[25]` | MatchY 1..8 |
| `[26]..[33]` | ScoreMilli 1..8 — template score ×1000; threshold zones: in-range % ×10 |
| `[34]` | Processing time, ms ×10 |
| `[35]..[38]` | reserved |
| `[39]` | **ResultId** — increments per inspection |

`Vision_Heartbeat` increments ~1/s while the app runs. Alarm if it stops
changing for > 3 s.

## PLC-side handshake routine (ladder, plain instructions)

Create a `UDT_VisionResult` mirroring the fields you care about, plus tags
`VisionResult` (the UDT), `Vision_LastId` (DINT), `Vision_NewResult` (BOOL).

Rung 1 — detect a new result:
```
NEQ(Vision_Transport[39], Vision_LastId)  ->  OTE(Vision_NewResult)
```

Rung 2 — on new result, capture the data then remember the id:
```
XIC(Vision_NewResult) ->
    MOV(Vision_Transport[0],  VisionResult.Pass)
    MOV(Vision_Transport[1],  VisionResult.RoiCount)
    COP(Vision_Transport[2],  VisionResult.RoiPass[0], 8)
    COP(Vision_Transport[10], VisionResult.MatchX[0],  8)
    COP(Vision_Transport[18], VisionResult.MatchY[0],  8)
    COP(Vision_Transport[26], VisionResult.ScoreMilli[0], 8)
    MOV(Vision_Transport[34], VisionResult.ProcTime_x10)
    MOV(Vision_Transport[39], Vision_LastId)
```
(Declare the UDT members as DINT / DINT arrays so the COPs are
type-clean — no BOOL members, no alignment games.)

Timeout supervision (optional): TON driven while waiting for
`Vision_NewResult` after triggering an inspection, preset per application.

## Emulator testing (no real PLC needed)

The same routine can be proven in Logix Emulate with a test rung that fakes
the app: move test values into `Vision_Transport[0..38]`, then bump `[39]`.

The PC side is tested against a local fake Logix (`cpppo`):
```
python -m cpppo.server.enip --address 127.0.0.1:44819 Vision_Transport=DINT[40] Vision_Heartbeat=DINT
```
**Port 44819, not 44818** — FactoryTalk Linx (RSLinxNG.exe) permanently owns
44818 on any PC with Rockwell software installed. The app backend
(`RockwellPlc`) auto-falls back to unrouted mode for simple servers; against
a real controller it uses normal routed messaging on 44818. Verified
2026-09-16: connect + 10 result writes (avg 6.2 ms) + full layout read-back
against cpppo.

## On-site commissioning checklist

1. PC and controller IPs on the same subnet (or routed); ping the controller.
2. App Settings → PLC backend `Rockwell EtherNet/IP`, controller IP,
   slot (0 for an L8xE embedded port; otherwise the ENxT module's chassis
   routing slot of the controller).
3. Tags created controller-scope, External Access Read/Write.
4. If CIP Security / trusted-slot features are enabled on the controller,
   permit the vision PC.
5. App Settings → "Send test result" → watch `Vision_Transport[39]` change
   in Studio 5000; confirm `Vision_Heartbeat` counting.
6. Run the latency watcher (same methodology as the Beckhoff acceptance
   test) for real trigger-to-PLC numbers.
