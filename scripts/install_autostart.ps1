<#
.SYNOPSIS
  Register the "MillCamera" scheduled task: runs scripts\start_mill.ps1 at
  system startup (no login needed), as the given user, and restarts it if it
  ever ends. Run from an elevated PowerShell in the project folder:

      Set-ExecutionPolicy -Scope Process Bypass
      .\scripts\install_autostart.ps1                 # as the current user
      .\scripts\install_autostart.ps1 -User DESKTOP\SomeUser

  Remove with:  Unregister-ScheduledTask -TaskName MillCamera -Confirm:$false
#>
param(
    [string]$TaskName = "MillCamera",
    [string]$User = "$env:USERDOMAIN\$env:USERNAME",
    [int]$DelaySeconds = 30
)
$ErrorActionPreference = "Stop"
$script = Join-Path (Split-Path -Parent $MyInvocation.MyCommand.Path) "start_mill.ps1"
if (-not (Test-Path $script)) { throw "not found: $script" }

$action = New-ScheduledTaskAction -Execute "powershell.exe" `
    -Argument "-NoProfile -ExecutionPolicy Bypass -WindowStyle Hidden -File `"$script`"" `
    -WorkingDirectory (Split-Path -Parent (Split-Path -Parent $script))
$trigger = New-ScheduledTaskTrigger -AtStartup
$trigger.Delay = "PT${DelaySeconds}S"
# S4U = "run whether user is logged on or not", no password stored
$principal = New-ScheduledTaskPrincipal -UserId $User -LogonType S4U -RunLevel Highest
$settings = New-ScheduledTaskSettingsSet -StartWhenAvailable -MultipleInstances IgnoreNew `
    -AllowStartIfOnBatteries -DontStopIfGoingOnBatteries -DontStopOnIdleEnd `
    -RestartCount 99 -RestartInterval (New-TimeSpan -Minutes 1)
$settings.ExecutionTimeLimit = "PT0S"          # no time limit (runs forever)

Register-ScheduledTask -TaskName $TaskName -Action $action -Trigger $trigger `
    -Principal $principal -Settings $settings -Force | Out-Null
$t = Get-ScheduledTask -TaskName $TaskName
Write-Host ("registered '{0}': state {1}, runs as {2} at startup (+{3}s), script {4}" -f `
    $TaskName, $t.State, $User, $DelaySeconds, $script)
Write-Host "start it now with:  Start-ScheduledTask -TaskName $TaskName"
