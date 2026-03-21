@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0.."
echo [MONITOR] %CD%
pio run -t monitor
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ERROR] Monitor exited with code %ERR%
)
echo.
pause
exit /b %ERR%
