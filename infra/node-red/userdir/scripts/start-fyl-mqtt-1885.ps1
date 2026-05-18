$ErrorActionPreference = 'Stop'

$mosquittoExe = 'C:\Program Files\mosquitto\mosquitto.exe'
$configPath = Join-Path $env:USERPROFILE '.node-red\mosquitto-1885.conf'
$logDir = Join-Path $env:USERPROFILE '.node-red\logs'
$stdoutLog = Join-Path $logDir 'mosquitto-1885-out.log'
$stderrLog = Join-Path $logDir 'mosquitto-1885-err.log'

New-Item -ItemType Directory -Force -Path $logDir | Out-Null

$alreadyListening = Get-NetTCPConnection -LocalPort 1885 -State Listen -ErrorAction SilentlyContinue
if ($alreadyListening) {
    exit 0
}

$existingProcess = Get-CimInstance Win32_Process -Filter "Name='mosquitto.exe'" |
    Where-Object { $_.CommandLine -like '*mosquitto-1885.conf*' }

if ($existingProcess) {
    exit 0
}

Start-Process `
    -FilePath $mosquittoExe `
    -ArgumentList @('-c', $configPath, '-v') `
    -WorkingDirectory $env:USERPROFILE `
    -RedirectStandardOutput $stdoutLog `
    -RedirectStandardError $stderrLog `
    -WindowStyle Hidden
