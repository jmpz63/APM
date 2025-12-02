param(
    [int]$Minutes = 15,
    [string]$TaskName = "APM_Pelosi_Trade_Monitor",
    [string]$PythonExe = "C:\\Users\\jmpz6\\AppData\\Local\\Microsoft\\WindowsApps\\python3.11.exe"
)

$ErrorActionPreference = 'Stop'

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$monitor = Join-Path $scriptDir 'monitor_pelosi_trades.py'

if (-not (Test-Path $monitor)) { Write-Error "Monitor script not found: $monitor"; exit 1 }
if (-not (Test-Path $PythonExe)) { $PythonExe = 'py' }

$taskRun = if ($PythonExe -ieq 'py') { 'py -3.11 ' + '"' + $monitor + '"' } else { '"' + $PythonExe + '" ' + '"' + $monitor + '"' }

$exists = $false
try { schtasks /Query /TN $TaskName | Out-Null; $exists = $true } catch { $exists = $false }
if ($exists) { schtasks /Delete /TN $TaskName /F | Out-Null }

schtasks /Create /SC MINUTE /MO $Minutes /TN $TaskName /TR $taskRun /F | Out-Null

Write-Host "Scheduled task '$TaskName' every $Minutes minute(s)."
Write-Host "Action: $taskRun"
Write-Host "To verify: schtasks /Query /TN $TaskName"
