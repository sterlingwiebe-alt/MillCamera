# Moving the tooling to another PC

`setup.ps1` installs Python packages and rewrites the venv path. Everything
below is what it cannot do for you: the network, the LiDAR's UDP target,
the firewall, the camera login and how the tools are started. Written from
the move to the mill PC on 2026-10-15 (i7-5500U, Windows 10, Administrator
account, project at `C:\Claude`); adjust the addresses if your network
differs.

## 1. Prerequisites

- Python 3.12 on PATH (`winget install Python.Python.3.12`).
- Git (`winget install Git.Git`): `pip` needs it for the SICK
  ScanSegmentAPI package, and you need it for this repository.
- Clone the repo, then in PowerShell from the project folder:

      Set-ExecutionPolicy -Scope Process Bypass
      .\setup.ps1            # or .\setup.ps1 -Offline with a wheelhouse\

  The venv goes to `%USERPROFILE%\.venvs\camera-app` (never inside
  OneDrive). Recordings (`lidar\z4captures\`) are not in git; the tools
  recreate the folder.

## 2. Network adapters

The devices live on two subnets, so the PC needs one wired adapter with a
static IPv4 address in each. The DEVICE addresses are what the tools know
(`--ip 192.168.1.20` for the PLC, `192.168.0.2` for the camera, both
defaults in the code); the PC's own addresses are not referenced anywhere
and can be any free host in the subnet.

| subnet | devices (fixed) | PC address |
|---|---|---|
| 192.168.0.0/24 ("Camera Net") | IP camera 192.168.0.2 (RTSP :554), multiScan165 LiDAR 192.168.0.1 (web UI) | any free address; this is the one the LiDAR must be told to stream to (section 3). Mill PC: 192.168.0.10 |
| 192.168.1.0/24 ("PLC Net") | ControlLogix 1756-L83E 192.168.1.20, slot 0 | any free address. Mill PC happens to be 192.168.1.240 |

No gateway is needed on either. Windows puts new wired networks on the
Public firewall profile; that is fine, the rules in section 4 cover it.

## 3. LiDAR UDP target (the sensor pushes to the PC)

The multiScan165 does not answer requests; it streams Compact-format UDP
to ONE address configured inside the sensor. After a move that address
still points at the OLD PC, and the new PC sees no frames and no error.

1. Open the sensor's web UI (http://192.168.0.1) in a browser on the new PC.
2. In the measurement data output settings (SOPAS web UI; on the mill PC
   it was under the Application area, look for "data output" / "Compact"),
   set: format Compact, protocol UDP, target IP = the new PC's address on
   the 192.168.0.x adapter, port 2115. Done on the mill PC with
   192.168.0.10 on 2026-10-15 and again 2026-10-17.
3. Leave interlacing x4 on; `lidar/interlace.py` pools 4 consecutive
   frames and expects it.

Check:

    python lidar\capture.py --seconds 4 --label smoke

should report frames arriving at ~20 per second.

## 4. Firewall

- Inbound Allow rule for the stream (exists on the mill PC as
  "LiDAR Compact UDP 2115 (inbound)", profile Any):

      New-NetFirewallRule -DisplayName "LiDAR Compact UDP 2115 (inbound)" -Direction Inbound -Protocol UDP -LocalPort 2115 -Action Allow -Profile Any

- The first time a Python program listens, Windows pops up its own
  "python.exe" prompt. If it is dismissed or answered "Cancel" it creates
  Block rules on Public, which silently drop the LiDAR stream. Check with

      Get-NetFirewallRule -DisplayName python.exe | Select DisplayName, Action, Profile, Enabled

  and set any Block rule to Allow (Windows Defender Firewall -> Advanced
  settings -> Inbound Rules). Run these as Administrator yourself; the
  Claude sandbox refuses firewall changes.

## 5. Camera login (CAM_URL)

The camera tools read the RTSP URL, including the login, from the `CAM_URL`
user environment variable (`lidar/camera_url.py`); it is deliberately not
in the code:

    setx CAM_URL rtsp://USER:PASSWORD@192.168.0.2:554/cam0_0

Open a new shell afterwards (and restart the Claude desktop app if you use
its launch entries).

The camera offers the same picture as three RTSP streams (measured
2026-10-17 by opening each one with OpenCV):

| stream | size | used by |
|---|---|---|
| `cam0_0` | 2048x1536, 30 fps | capture tool stills; `lidar/camera_zones.json` is calibrated for this size |
| `cam0_1` | 640x480, 30 fps | live view (`.claude\launch.json`), chosen because decoding the full stream pinned the mill PC's CPU |
| `cam0_2` | 320x240, 30 fps | unused |

There is no `cam0_3`. The substream only changes what the live view
decodes; measurements and the zone calibration are unaffected.

## 6. PLC

The capture tool writes `Vision_Transport[0..39]` at 5 Hz and
`Vision_Heartbeat` at 1 Hz to 192.168.1.20 (layout: `plc/Vision_Transport_v2.md`,
`lidar/plc_out.py`) and watches the `Vision_Capture` tag. Nothing to
configure on a new PC beyond the PLC Net adapter; the PLC side is the same
project.

## 7. Starting the tools

From the project folder with the venv's python (`setup.ps1` prints the
path):

    python lidar\interlace_capture.py --ack --plc-out      # capture tool + flow tracker + PLC output
    python lidar\live_view.py --port 8200 --cam cam0_1     # live view, http://localhost:8200
    python lidar\light_bank.py --interval 300 --no-lidar   # camera light bank (optional)

### Automatic start (installed on the mill PC 2026-10-17)

`scripts\start_mill.ps1` is a supervisor: it reads `local.env`, waits up to
2 min for the PLC and LiDAR to answer, starts the capture tool and the live
view with their consoles going to `lidar\z4captures\interlaced\
capture_console.log` / `liveview.log` (older runs kept with a timestamp),
and restarts either program if it exits (5 s back-off, doubling to 60 s,
reset after 5 min of healthy running). Its own log is `supervisor.log`.

The scheduled task **MillCamera** runs it at system startup (+30 s), as
the Administrator account whether anyone is logged in or not, with no time
limit, and Task Scheduler restarts the task itself if it ever ends. Install
on a new PC from an elevated PowerShell:

    Copy-Item local.env.example local.env     # then put the real CAM_URL in it
    .\scripts\install_autostart.ps1

Day to day:

    Start-ScheduledTask -TaskName MillCamera                     # start now (also happens at boot)
    powershell -ExecutionPolicy Bypass -File scripts\stop_mill.ps1   # stop everything
    Get-ScheduledTask MillCamera | Select State                  # Running = supervisor alive
    Get-Content lidar\z4captures\interlaced\supervisor.log -Tail 20

So after a reboot there is nothing to do: about a minute after Windows is
up the PLC output and http://<pc>:8200 are back. To run the tools by hand
instead, stop the task first (two capture tools would fight over UDP 2115
and both write the PLC).

Note: `interlace_capture.py` has a `--duration` default of 6 hours, after
which it exits normally; the supervisor passes `--duration 315360000` (10
years) so it runs until stopped. Before that fix (2026-10-19) the tool
restarted itself every 6 h (5 s gap in PLC output, pass counter and
pairing window reset). Because the supervisor restarts the tool from the
files on disk, ANY code change becomes live at the next restart, planned
or not: pull/edit on the mill PC only what is meant to run.

While the LiDAR stream is absent (sensor off), the SICK library prints a
timeout traceback every second to `capture_console.err.log` (1-2 MB per
hour of outage); harmless, the tool keeps waiting.

Signs to look for: `capture_console.log` shows "flow mode ON", "PLC OUT:
writing Vision_Transport[0..39]" and "ARMED on 'Vision_Capture'"; the live
view's LiDAR state is not marked stale; `Vision_Transport[0]` (fresh flag)
stays at 1 while the line runs. "PLC OUT write failed: Connection failure"
means the PLC Net link dropped (the tool keeps retrying).

The supervisor also places the processes: Task Scheduler starts everything
at BelowNormal priority, and on this 2-core/4-thread PC the live view's
video decoding stalled the tracker for ~2 s at a time (state age spikes
to 1.9 s = fresh flag flicker) even after fixing the priority. So the
capture tool runs AboveNormal pinned to logical CPUs 0-1 (one physical
core) and the live view BelowNormal on CPUs 2-3. Measured 2026-10-17 over
90 s each, deck idle: no live view, median 0.12 s / max 0.66 s; live view
without the core split, max 1.9 s with several samples over 1 s; live
view with the split, median 0.13 s / p90 0.21 s / max 0.80 s, none over
1 s. The live view costs 25-35% of a core alone, ~50% with two browsers
connected (9 fps each).

Verified 2026-10-17: starting the task by hand brings both programs up
within 2 s; killing the live view process made the supervisor restart it
(back-off 5 s, 10 s, ... 60 s).

## 8. Performance on a slow PC

The tracker needed 148 ms per 20 Hz frame on the mill PC with the original
code (state lag 0.5-2.5 s, fresh flag flickering). The 2026-10-15
optimisation of `lidar/arcfit.py`, `segment.py` and `flow_track.py`
brought that to ~22 ms per frame on the same PC with bit-identical
results, measured by replaying recordings offline
(`lidar/z4captures/interlaced/perf_notes.md`, harness
`lidar/perf_replay.py`). Keep the full-resolution live view off the
tracker PC regardless.

## 9. Status of the mill PC (2026-10-17)

Verified on this PC:

- Python 3.12 + venv work; the SICK package is in the venv (copied from
  the first PC, not reinstalled).
- LiDAR Compact UDP frames arrive at 192.168.0.10:2115 at 19.6 frames/s
  with no parse errors (`lidar\capture.py --seconds 4`, 2026-10-17, after
  the sensor's target was set to this PC).
- Camera streams open with the `CAM_URL` login at the sizes above.
- PLC (192.168.1.20:44818), camera and LiDAR web UI are reachable.
- Firewall: UDP 2115 Allow rule present; python.exe rules set to Allow.
- The optimised tracker gives identical results to the original on 278
  recorded frames and runs at ~22 ms per frame offline.

- First live run of the optimised tracker, started 2026-10-17 18:56 with
  the deck idle: published state age 0.02-0.20 s (was 0.5-2.5 s), no
  errors in the first 15 minutes. The process still takes ~94% of one
  core: py-spy shows 79% of that in the SICK library's packet parser
  (`scansegmentapi/compact.py` `_read_beam_data`, a per-beam Python loop
  in the UDP receive thread) and 21% in the tracker. That parser is the
  next bottleneck; see `lidar/z4captures/interlaced/perf_notes.md`.

NOT yet verified:

- (Verified 2026-10-19, first production day: ~1100 passes, five zones
  active, state age median 0.10 s / p90 0.19 s / max 0.40 s over 60 s
  samples, none over 0.5 s. The PC keeps up with the optimised tracker.)
- The tracker under load: the 2026-10-17 runs only saw an idle deck.
  With logs moving the tracker needs more of its core and competes with
  the packet parser for the interpreter lock, so lag may creep back until
  the parser is sped up or moved to its own process. Watch the state age
  (`live_state.json` "t" vs now, or the live view's stale marker) during
  the first production shift; samples over 1 s mean the fresh flag
  flickers.
- Long-term stability on this PC (hours of operation, memory, the PLC
  link): the only long live run so far was 2026-10-15 (498 passes) with
  the old code; it ran until the PC was restarted on 2026-10-16 08:50
  (user-initiated restart that ended in an unclean power-off; the tool
  itself did not crash).
- The automatic start has been tested by starting the task by hand, not
  yet by an actual reboot of the mill PC.

Until the first long live run with the current code has been watched, do
not treat this PC as proven.
