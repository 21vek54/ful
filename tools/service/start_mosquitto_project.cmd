@echo off
setlocal

set "ROOT=%~dp0..\..\"
set "MOSQ_DIR=C:\Program Files\Mosquitto"
set "CONF=%ROOT%infra\mosquitto\conf\mosquitto.conf"

echo Starting Mosquitto with config:
echo   %CONF%

"%MOSQ_DIR%\mosquitto.exe" -c "%CONF%" -v
