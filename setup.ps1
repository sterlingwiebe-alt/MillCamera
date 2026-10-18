<#
.SYNOPSIS
  Set up the Camera App / LiDAR tooling on a new PC.

.DESCRIPTION
  Creates the Python virtual environment OUTSIDE the project folder (never
  inside OneDrive), installs the pinned packages, rewrites the venv path in
  .claude\launch.json, and runs a quick import check.

  Run from the project folder in PowerShell:
      Set-ExecutionPolicy -Scope Process Bypass
      .\setup.ps1                       # venv at %USERPROFILE%\.venvs\camera-app
      .\setup.ps1 -VenvPath D:\venvs\camera-app
      .\setup.ps1 -Loose                # newest compatible versions instead of the lock file
      .\setup.ps1 -Offline              # no internet: install from .\wheelhouse (built on this PC)

  Requires Python 3.12 on PATH (python.org installer or Microsoft Store) and git
  (the SICK ScanSegmentAPI is installed from GitHub).
#>
param(
    [string]$VenvPath = (Join-Path $env:USERPROFILE ".venvs\camera-app"),
    [switch]$Loose,
    [switch]$Offline      # install from .\wheelhouse only (no internet needed)
)
$ErrorActionPreference = "Stop"
$proj = Split-Path -Parent $MyInvocation.MyCommand.Path
Set-Location $proj

Write-Host "== Camera App setup ==" -ForegroundColor Cyan
Write-Host "project : $proj"
Write-Host "venv    : $VenvPath"
if ($proj -like "*OneDrive*") {
    Write-Warning "The project is inside OneDrive. Keep lidar_recordings\ and lidar\z4captures\ OUT of it (tens of GB)."
}
if ($VenvPath -like "*OneDrive*") { throw "Do not put the venv inside OneDrive." }

# --- python
$py = Get-Command python -ErrorAction SilentlyContinue
if (-not $py) { throw "python not found on PATH. Install Python 3.12 first." }
$ver = & python -c "import sys; print('%d.%d' % sys.version_info[:2])"
Write-Host "python  : $ver"
if ($ver -ne "3.12") { Write-Warning "Tested with Python 3.12; you have $ver." }
if (-not (Get-Command git -ErrorAction SilentlyContinue)) { throw "git not found on PATH (needed for the SICK ScanSegmentAPI package)." }

# --- venv
if (-not (Test-Path $VenvPath)) {
    New-Item -ItemType Directory -Force (Split-Path -Parent $VenvPath) | Out-Null
    & python -m venv $VenvPath
}
$vpy = Join-Path $VenvPath "Scripts\python.exe"

# --- packages
if ($Offline) {
    $wh = Join-Path $proj "wheelhouse"
    if (-not (Test-Path $wh)) { throw "wheelhouse\ not found. Build it on a PC with internet: python -m pip wheel -r requirements-lock.txt -w wheelhouse" }
    Write-Host "installing offline from $wh"
    & $vpy -m pip install --no-index --find-links $wh -r requirements-lock.txt
} else {
    & $vpy -m pip install --upgrade pip wheel | Out-Null
    if ($Loose) {
        & $vpy -m pip install -r requirements.txt
    } else {
        & $vpy -m pip install -r requirements-lock.txt
    }
}

# --- launch.json venv path
$launch = Join-Path $proj ".claude\launch.json"
if (Test-Path $launch) {
    $txt = Get-Content $launch -Raw
    $escaped = $vpy.Replace("\", "\\")
    $txt = [regex]::Replace($txt, 'C:\\\\Users\\\\[^"\\]+\\\\\.venvs\\\\camera-app\\\\Scripts\\\\python\.exe', $escaped)
    Set-Content $launch $txt -Encoding utf8
    Write-Host "launch.json venv path -> $vpy"
}

# --- import check
& $vpy -c "import numpy, cv2, pylogix, fastapi, uvicorn, openpyxl, matplotlib; from scansegmentapi.compact import parse_payload; print('imports OK: numpy', numpy.__version__, 'opencv', cv2.__version__)"

Write-Host ""
if (-not $env:CAM_URL) {
    Write-Warning "CAM_URL is not set. The camera tools read the RTSP URL (with login) from it:"
    Write-Host '  setx CAM_URL rtsp://USER:PASSWORD@192.168.0.2:554/cam0_0   (then open a new shell)'
}
Write-Host "Done. Next: follow MOVE.md (network, LiDAR UDP target, firewall), then:" -ForegroundColor Green
Write-Host "  & `"$vpy`" lidar\capture.py --seconds 4 --label smoke      # LiDAR arriving?"
Write-Host "  & `"$vpy`" lidar\interlace_capture.py --ack              # capture tool + flow mode"
Write-Host "  & `"$vpy`" lidar\live_view.py --port 8200                 # http://localhost:8200"
Write-Host "  & `"$vpy`" lidar\light_bank.py --interval 300 --no-lidar  # camera light bank (optional)"
