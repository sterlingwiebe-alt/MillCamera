<#
.SYNOPSIS
  Supervisor for the mill PC: starts the LiDAR capture tool and the web live
  view, and restarts either one if it exits.

.DESCRIPTION
  Run at boot by the "MillCamera" scheduled task (scripts\install_autostart.ps1)
  or by hand from any shell:

      powershell -ExecutionPolicy Bypass -File scripts\start_mill.ps1
      powershell -ExecutionPolicy Bypass -File scripts\start_mill.ps1 -NoLiveView

  Settings come from <project>\local.env (KEY=VALUE lines, git-ignored):
      CAM_URL=rtsp://USER:PASSWORD@192.168.0.2:554/cam0_0   (required for the camera)
      MILL_PYTHON=C:\path\to\venv\Scripts\python.exe        (optional; else .claude\launch.json, else %USERPROFILE%\.venvs\camera-app)
  Logs: lidar\z4captures\interlaced\supervisor.log, capture_console.log,
  liveview.log (previous runs are kept with a timestamp suffix).
  Stop everything with scripts\stop_mill.ps1.
#>
param(
    [string]$Project = (Split-Path -Parent (Split-Path -Parent $MyInvocation.MyCommand.Path)),
    [switch]$NoLiveView
)
$ErrorActionPreference = "Continue"
Set-Location $Project
$dump = Join-Path $Project "lidar\z4captures\interlaced"
New-Item -ItemType Directory -Force $dump | Out-Null
$supLog = Join-Path $dump "supervisor.log"

function Log($msg) {
    $line = "{0:yyyy-MM-dd HH:mm:ss}  {1}" -f (Get-Date), $msg
    Add-Content -Path $supLog -Value $line -Encoding utf8
    Write-Host $line
}

# --- settings: local.env, then the user's environment
$envFile = Join-Path $Project "local.env"
if (Test-Path $envFile) {
    foreach ($l in Get-Content $envFile) {
        if ($l -match '^\s*([A-Za-z_][A-Za-z0-9_]*)\s*=\s*(.*?)\s*$') {
            Set-Item -Path "Env:$($Matches[1])" -Value $Matches[2]
        }
    }
}
if (-not $env:CAM_URL) {
    $u = [Environment]::GetEnvironmentVariable("CAM_URL", "User")
    if ($u) { $env:CAM_URL = $u }
}
if (-not $env:CAM_URL) {
    Log "WARNING: CAM_URL is not set (local.env or user env var); camera stills and the live view will fail"
}

# --- python (venv)
$py = $env:MILL_PYTHON
if (-not $py) {
    $lj = Join-Path $Project ".claude\launch.json"
    if (Test-Path $lj) {
        $m = [regex]::Match((Get-Content $lj -Raw), '"runtimeExecutable"\s*:\s*"([^"]+)"')
        if ($m.Success) { $py = $m.Groups[1].Value.Replace('\\', '\') }
    }
}
if (-not $py -or -not (Test-Path $py)) {
    $py = Join-Path $env:USERPROFILE ".venvs\camera-app\Scripts\python.exe"
}
if (-not (Test-Path $py)) {
    Log "FATAL: python not found ($py); set MILL_PYTHON in local.env"
    exit 1
}
Log "supervisor start: project=$Project python=$py liveview=$(-not $NoLiveView)"

# --- wait (up to 2 min) for the PLC and the LiDAR to answer: the static
# adapters come up a few seconds after boot. Informational only; the tools
# retry on their own.
$deadline = (Get-Date).AddSeconds(120)
$reach = $false
while ((Get-Date) -lt $deadline) {
    $plc = Test-NetConnection 192.168.1.20 -Port 44818 -WarningAction SilentlyContinue -InformationLevel Quiet
    $lidar = Test-NetConnection 192.168.0.1 -Port 80 -WarningAction SilentlyContinue -InformationLevel Quiet
    if ($plc -and $lidar) { $reach = $true; break }
    Start-Sleep -Seconds 5
}
Log "PLC and LiDAR reachable: $reach"

# --- the processes
function Rotate($path) {
    if (Test-Path $path) {
        $stamp = (Get-Item $path).LastWriteTime.ToString("yyyyMMdd_HHmmss")
        Move-Item $path ($path -replace '\.log$', "_$stamp.log") -Force
    }
}
# CPU priority and placement. Task Scheduler starts tasks BELOW NORMAL, and
# on the mill PC (2 cores / 4 threads) the live view's video decoding
# stalled the tracker for ~2 s at a time even at lower priority (state age
# spikes to 1.9 s, 2026-10-17; without the live view: max 0.66 s). The
# tracker feeds the PLC, so it gets AboveNormal and its own physical core
# (logical CPUs 0,1); the live view is cosmetic: BelowNormal on CPUs 2,3.
# On a PC with fewer than 4 logical CPUs no affinity is set.
$ncpu = [Environment]::ProcessorCount
$procs = @(
    @{ name = "capture"; args = @("lidar\interlace_capture.py", "--ack", "--plc-out")
       out = "$dump\capture_console.log"; err = "$dump\capture_console.err.log"
       prio = "AboveNormal"; affinity = $(if ($ncpu -ge 4) { 0x3 } else { 0 })
       proc = $null; fails = 0; last = $null }
)
if (-not $NoLiveView) {
    $procs += @{ name = "liveview"; args = @("lidar\live_view.py", "--port", "8200", "--cam", "cam0_1")
                 out = "$dump\liveview.log"; err = "$dump\liveview.err.log"
                 prio = "BelowNormal"; affinity = $(if ($ncpu -ge 4) { 0xC } else { 0 })
                 proc = $null; fails = 0; last = $null }
}

function Place($proc, $p) {
    try { $proc.PriorityClass = $p.prio } catch {}
    if ($p.affinity) { try { $proc.ProcessorAffinity = [IntPtr]$p.affinity } catch {} }
}

function SetPriority($p) {
    # the venv's python.exe is a launcher that runs the real interpreter as
    # a child: set both (the child exists a moment after the launcher)
    Place $p.proc $p
    Start-Sleep -Seconds 2
    Get-CimInstance Win32_Process -Filter "ParentProcessId=$($p.proc.Id) and Name='python.exe'" | ForEach-Object {
        try { Place (Get-Process -Id $_.ProcessId) $p } catch {}
    }
}

function StartOne($p) {
    Rotate $p.out; Rotate $p.err
    $p.proc = Start-Process -FilePath $py -ArgumentList $p.args -WorkingDirectory $Project `
        -RedirectStandardOutput $p.out -RedirectStandardError $p.err -WindowStyle Hidden -PassThru
    $p.last = Get-Date
    SetPriority $p
    Log ("started {0} pid {1} ({2}, affinity 0x{3:X}): {4}" -f $p.name, $p.proc.Id, $p.prio, $p.affinity, ($p.args -join ' '))
}

foreach ($p in $procs) { StartOne $p }

# --- supervise: restart whatever exits, with a back-off that resets after
# 5 minutes of healthy running
while ($true) {
    Start-Sleep -Seconds 5
    foreach ($p in $procs) {
        if ($p.proc.HasExited) {
            $up = ((Get-Date) - $p.last).TotalSeconds
            if ($up -gt 300) { $p.fails = 0 } else { $p.fails++ }
            $wait = [math]::Min(60, 5 * [math]::Pow(2, [math]::Min([math]::Max($p.fails - 1, 0), 4)))   # 5,10,20,40,60 s
            Log ("{0} exited (code {1}) after {2:N0}s; restarting in {3}s" -f $p.name, $p.proc.ExitCode, $up, $wait)
            Start-Sleep -Seconds $wait
            StartOne $p
        }
    }
}
