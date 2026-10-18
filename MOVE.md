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

The camera offers the same picture as several RTSP streams at different
sizes: `cam0_0` is the full 2048x1536 stream (`lidar/camera_zones.json` is
calibrated for it); `cam0_1` and `cam0_2` are smaller substreams (on the
mill PC they came up as 640x480 and 320x240; check with the camera's web
page or `ffprobe` if it matters). `.claude\launch.json` starts the live
view on `cam0_1` because decoding the full stream pinned the mill PC's
CPU; the capture tool still grabs its stills from `cam0_0`, so the zone
calibration applies to them.

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

On the mill PC the capture tool is started detached so it survives the
terminal closing, with its console going to a log:

    Start-Process -FilePath <venv>\Scripts\python.exe -ArgumentList "lidar\interlace_capture.py","--ack","--plc-out" -WorkingDirectory C:\Claude -RedirectStandardOutput lidar\z4captures\interlaced\capture_console.log -RedirectStandardError lidar\z4captures\interlaced\capture_console.err.log

Healthy signs: the log shows "flow mode ON", "PLC OUT: writing
Vision_Transport[0..39]" and "ARMED on 'Vision_Capture'"; the live view's
LiDAR state is not marked stale; `Vision_Transport[0]` (fresh flag) stays
at 1.

## 8. Performance notes for a slow PC

The tracker needs ~22 ms per 20 Hz frame after the 2026-10-15
optimisation (`lidar/z4captures/interlaced/perf_notes.md`); before it,
148 ms, which is why the mill PC showed a 0.5-2.5 s state lag. Keep the
full-resolution live view off the tracker PC. `lidar/perf_replay.py`
replays recordings offline to time the tracker on any machine.
