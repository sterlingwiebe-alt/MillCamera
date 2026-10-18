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

Two wired adapters, static IPv4, no gateway needed on either:

| adapter (name it like this) | this PC | devices on it |
|---|---|---|
| Camera Net | 192.168.0.10 /24 | IP camera 192.168.0.2 (RTSP :554), multiScan165 LiDAR 192.168.0.1 (web UI) |
| PLC Net | 192.168.1.240 /24 | ControlLogix 1756-L83E 192.168.1.20, slot 0 |

The addresses are hard-coded defaults in the tools (`--ip 192.168.1.20`,
camera `192.168.0.2`), so keep the device side the same and only the PC
addresses need to exist. Windows puts new wired networks on the Public
profile; that is fine, the firewall rules below cover it.

## 3. LiDAR UDP target (the sensor pushes to the PC)

The multiScan165 streams Compact-format UDP to ONE configured address. It
must be this PC:

1. Open the sensor web UI at http://192.168.0.1 from the Camera Net adapter.
2. Application -> Data output: set the target to this PC's Camera Net IP
   (192.168.0.10), port 2115, Compact format. The first PC was
   192.168.0.252; forgetting this step means "no frames" with no error.
3. Leave interlacing x4 on (the pooled analysis in `lidar/interlace.py`
   expects it).

Check:

    python lidar\capture.py --seconds 4 --label smoke

should report ~20 frames/s arriving.

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
its launch entries). Streams: `cam0_0` full 2048x1536, `cam0_1` 640x480,
`cam0_2` 320x240. `.claude\launch.json` starts the live view on `cam0_1`
because decoding the full stream pinned the mill PC's CPU; the capture
tool still grabs full-resolution stills from `cam0_0`.

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
