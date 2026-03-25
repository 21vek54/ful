@echo off
setlocal

set "ROOT=%~dp0..\..\"

echo Root:
echo   %ROOT%
echo.
echo Node-RED userDir:
echo   %ROOT%infra\node-red\userdir
echo.
echo Mosquitto config:
echo   %ROOT%infra\mosquitto\conf\mosquitto.conf
