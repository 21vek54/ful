@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0master_control_com12"
echo [FLASH] master_control_com12 (COM12)
pio run -t upload
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ERROR] Upload failed with code %ERR%
) else (
  echo [OK] Upload completed.
)
echo.
pause
exit /b %ERR%
