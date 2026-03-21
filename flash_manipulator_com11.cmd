@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0manipulator_com11"
echo [FLASH] manipulator_com11 (COM11)
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
