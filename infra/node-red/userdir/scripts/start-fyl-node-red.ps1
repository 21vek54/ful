$ErrorActionPreference = 'Stop'

$nodeRedCmd = Join-Path $env:APPDATA 'npm\node-red.cmd'
$logDir = Join-Path $env:USERPROFILE '.node-red\logs'
$stdoutLog = Join-Path $logDir 'node-red-out.log'
$stderrLog = Join-Path $logDir 'node-red-err.log'

New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$alreadyListening = Get-NetTCPConnection -LocalPort 1880 -State Listen -ErrorAction SilentlyContinue
if ($alreadyListening) {
    exit 0
}

$existingProcess = Get-CimInstance Win32_Process -Filter "Name='node.exe'" |
    Where-Object { $_.CommandLine -like '*node-red*' }

if ($existingProcess) {
    exit 0
}

Start-Sleep -Seconds 5

Start-Process `
    -FilePath $nodeRedCmd `
    -WorkingDirectory $env:USERPROFILE `
    -RedirectStandardOutput $stdoutLog `
    -RedirectStandardError $stderrLog `
    -WindowStyle Hidden
