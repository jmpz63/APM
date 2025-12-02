param(
    [string]$Time = "08:30",
    [string]$TaskName = "APM_BTC_Daily_Capture",
    [string]$PythonExe = "C:\\Users\\jmpz6\\AppData\\Local\\Microsoft\\WindowsApps\\python3.11.exe"
)

$ErrorActionPreference = 'Stop'

# Resolve script path
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$btcScript = Join-Path $scriptDir 'btc_daily_capture.py'

if (-not (Test-Path $btcScript)) {
    Write-Error "BTC capture script not found: $btcScript"
    exit 1
}

if (-not (Test-Path $PythonExe)) {
    # Fallback to 'py -3.11' if python path not found
    $PythonExe = 'py'
}

# Build the task action command
# Wrap paths with quotes for schtasks
$taskRun = if ($PythonExe -ieq 'py') {
    'py -3.11 ' + '"' + $btcScript + '"'
} else {
    '"' + $PythonExe + '" ' + '"' + $btcScript + '"'
}

# If task exists, delete to allow updates
$exists = $false
try {
    schtasks /Query /TN $TaskName | Out-Null
    $exists = $true
} catch {
    $exists = $false
}

if ($exists) {
    schtasks /Delete /TN $TaskName /F | Out-Null
}

# Create the daily task at the specified time (local time)
schtasks /Create /SC DAILY /ST $Time /TN $TaskName /TR $taskRun /F | Out-Null

Write-Host "Scheduled task '$TaskName' to run daily at $Time"
Write-Host "Action: $taskRun"
Write-Host "To verify: schtasks /Query /TN $TaskName"
