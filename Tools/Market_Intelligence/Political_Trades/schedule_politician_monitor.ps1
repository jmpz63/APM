param(
    [int]$Minutes = 15,
    [string]$TaskName = "APM_Pelosi_Trade_Monitor",
    [string]$PythonExe = "C:\\Users\\jmpz6\\AppData\\Local\\Microsoft\\WindowsApps\\python3.11.exe"
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$monitor = Join-Path $scriptDir 'monitor_pelosi_trades.py'
$vbs = Join-Path $scriptDir 'run_hidden.vbs'

if (-not (Test-Path $monitor)) { Write-Error "Monitor script not found: $monitor"; exit 1 }
if (-not (Test-Path $PythonExe)) { $PythonExe = 'py' }

# Create or update a VBScript wrapper to run hidden
# This avoids flashing console windows when the task triggers
$cmd = if ($PythonExe -ieq 'py') { "py -3.11 `"$monitor`"" } else { '"' + $PythonExe + '" ' + '"' + $monitor + '"' }
$vbsContent = @"
Set WshShell = CreateObject("WScript.Shell")
WshShell.Run """$cmd""", 0, False
"@
Set-Content -Path $vbs -Value $vbsContent -Encoding ASCII

# Task runs the VBScript via cscript with no logo, so it's invisible
$taskRun = 'cscript //nologo ' + '"' + $vbs + '"'

$exists = $false
try { schtasks /Query /TN $TaskName | Out-Null; $exists = $true } catch { $exists = $false }
if ($exists) { schtasks /Delete /TN $TaskName /F | Out-Null }

schtasks /Create /SC MINUTE /MO $Minutes /TN $TaskName /TR $taskRun /F | Out-Null

Write-Host "Scheduled task '$TaskName' every $Minutes minute(s)."
Write-Host "Action: $taskRun"
Write-Host "To verify: schtasks /Query /TN $TaskName"
