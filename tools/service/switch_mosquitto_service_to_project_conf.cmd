@echo off
setlocal EnableExtensions

for %%I in ("%~dp0..\..") do set "REPO_ROOT=%%~fI"
set "SRC_CONF=%REPO_ROOT%\infra\mosquitto\conf\mosquitto_1884.conf"

if /I "%~1"=="1883" set "SRC_CONF=%REPO_ROOT%\infra\mosquitto\conf\mosquitto.conf"
if /I "%~1"=="1884" set "SRC_CONF=%REPO_ROOT%\infra\mosquitto\conf\mosquitto_1884.conf"

set "MOSQ_EXE=C:\Program Files\Mosquitto\mosquitto.exe"
set "TARGET_CONF=C:\Program Files\Mosquitto\mosquitto.conf"
set "BACKUP_CONF=C:\Program Files\Mosquitto\mosquitto.conf.fyl.backup"

powershell -NoProfile -Command "$p = New-Object Security.Principal.WindowsPrincipal([Security.Principal.WindowsIdentity]::GetCurrent()); if ($p.IsInRole([Security.Principal.WindowsBuiltInRole]::Administrator)) { exit 0 } else { exit 1 }"
if errorlevel 1 (
  echo ERROR: Administrator rights are required to update C:\Program Files\Mosquitto\mosquitto.conf
  echo Please rerun this script from an elevated terminal.
  exit /b 1
)

if not exist "%SRC_CONF%" (
  echo ERROR: Source config not found:
  echo   %SRC_CONF%
  exit /b 1
)

if not exist "%MOSQ_EXE%" (
  echo ERROR: Mosquitto executable not found:
  echo   %MOSQ_EXE%
  exit /b 1
)

echo Validating source config:
echo   %SRC_CONF%
"%MOSQ_EXE%" -c "%SRC_CONF%" --test-config
if errorlevel 1 (
  echo ERROR: Source config validation failed.
  exit /b 1
)

if exist "%TARGET_CONF%" (
  copy /y "%TARGET_CONF%" "%BACKUP_CONF%" >nul
)

echo Applying config to service default path:
echo   %TARGET_CONF%
copy /y "%SRC_CONF%" "%TARGET_CONF%" >nul
if errorlevel 1 (
  echo ERROR: Failed to copy config to Program Files.
  exit /b 1
)

"%MOSQ_EXE%" -c "%TARGET_CONF%" --test-config
if errorlevel 1 goto :rollback

echo Ensuring service startup mode is default:
sc config mosquitto binPath= "\"%MOSQ_EXE%\" run"
if errorlevel 1 goto :rollback

echo Restarting service...
sc stop mosquitto >nul 2>&1
powershell -NoProfile -Command "$n='mosquitto'; $d=(Get-Date).AddSeconds(60); do { $s=(Get-Service -Name $n).Status; if ($s -eq 'Stopped') { exit 0 }; Start-Sleep -Seconds 1 } while((Get-Date) -lt $d); exit 1"
if errorlevel 1 goto :rollback

sc start mosquitto >nul 2>&1
if errorlevel 1 goto :rollback

powershell -NoProfile -Command "$n='mosquitto'; $d=(Get-Date).AddSeconds(60); do { $s=(Get-Service -Name $n).Status; if ($s -eq 'Running') { exit 0 }; Start-Sleep -Seconds 1 } while((Get-Date) -lt $d); exit 1"
if errorlevel 1 goto :rollback

echo Service switched successfully.
sc qc mosquitto
sc query mosquitto
exit /b 0

:rollback
echo WARNING: Service switch failed. Rolling back previous config.
if exist "%BACKUP_CONF%" (
  copy /y "%BACKUP_CONF%" "%TARGET_CONF%" >nul
)

sc config mosquitto binPath= "\"%MOSQ_EXE%\" run" >nul
sc stop mosquitto >nul 2>&1
powershell -NoProfile -Command "$n='mosquitto'; $d=(Get-Date).AddSeconds(60); do { $s=(Get-Service -Name $n).Status; if ($s -eq 'Stopped') { exit 0 }; Start-Sleep -Seconds 1 } while((Get-Date) -lt $d); exit 1" >nul 2>&1
sc start mosquitto >nul 2>&1
powershell -NoProfile -Command "$n='mosquitto'; $d=(Get-Date).AddSeconds(60); do { $s=(Get-Service -Name $n).Status; if ($s -eq 'Running') { exit 0 }; Start-Sleep -Seconds 1 } while((Get-Date) -lt $d); exit 1" >nul 2>&1

sc qc mosquitto
sc query mosquitto
exit /b 1
