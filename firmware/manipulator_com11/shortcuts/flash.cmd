@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0.."
echo [FLASH] %CD%
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
