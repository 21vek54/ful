@echo off
setlocal

set "ROOT=%~dp0..\..\"
set "USERDIR=%ROOT%infra\node-red\userdir"

echo Starting Node-RED with userDir:
echo   %USERDIR%

node-red --userDir "%USERDIR%"
