<#
.SYNOPSIS
  Stop the supervisor, the capture tool and the live view (whether started by
  the MillCamera scheduled task or by hand).

      powershell -ExecutionPolicy Bypass -File scripts\stop_mill.ps1
#>
$ErrorActionPreference = "SilentlyContinue"
$task = Get-ScheduledTask -TaskName "MillCamera"
if ($task -and $task.State -eq "Running") {
    Stop-ScheduledTask -TaskName "MillCamera"
    Write-Host "scheduled task MillCamera stopped"
}
# supervisor first, so it does not restart what we kill next
foreach ($pat in @("start_mill.ps1", "interlace_capture.py", "live_view.py")) {
    Get-CimInstance Win32_Process | Where-Object { $_.CommandLine -like "*$pat*" } | ForEach-Object {
        Write-Host ("stopping pid {0}: {1}" -f $_.ProcessId, $_.CommandLine)
        Stop-Process -Id $_.ProcessId -Force
    }
}
Write-Host "done"
