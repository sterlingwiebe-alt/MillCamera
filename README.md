# Camera App / Log Deck LiDAR tooling

Sawmill log-deck measurement: a SICK multiScan165 LiDAR streams Compact UDP
frames at 20 Hz; `lidar/` measures every log passing zone 4 (diameter,
presence per deck zone) and publishes the state to the PLC and a live web
view. `app/` is the camera inspection app (FastAPI + Basler/RTSP camera +
PLC heartbeat); `plc/` holds the TwinCAT/Rockwell interface definitions.

## Setup (new PC)

    Set-ExecutionPolicy -Scope Process Bypass
    .\setup.ps1                 # venv + pinned packages (requirements-lock.txt)
    setx CAM_URL rtsp://USER:PASSWORD@<camera-ip>:554/cam0_0

Python 3.12 and git are required (the SICK ScanSegmentAPI installs from
GitHub). The camera login is never in the code: every camera tool reads
`CAM_URL` (see `lidar/camera_url.py`); `.claude/launch.json` passes only the
stream name (`cam0_1`, the lighter substream for the live view).

## Running

    python lidar\interlace_capture.py --ack --plc-out   # capture tool + flow tracker + PLC output
    python lidar\live_view.py --port 8200               # live view, http://localhost:8200
    python -m uvicorn app.main:app --port 8000          # camera app

## Layout

| path | what |
|---|---|
| `lidar/flow_track.py` | FlowTracker: per-frame zone-4 measurement, presence, live_state.json |
| `lidar/segment.py`, `arcfit.py`, `probe_zones.py`, `interlace.py` | log segmentation (RANSAC over arc centres), per-line circle fits, presence windows, x4 interlace pooling |
| `lidar/plc_out.py` | Vision_Transport DINT[40] layout v2 writer |
| `lidar/perf_replay.py` | offline replay / timing / equivalence harness |
| `lidar/z4captures/` | recordings and run outputs (not in git, see `.gitignore`); `interlaced/perf_notes.md` is |
| `app/`, `recipes/` | camera inspection app and its recipes |
| `plc/` | PLC function blocks, transport layout |
| `reports/`, `scripts/` | LiDAR-vs-optimizer report and helper scripts |
