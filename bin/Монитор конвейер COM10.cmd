@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0..\firmware\conveyor_control_com10"
echo [МОНИТОР] Конвейер COM10
pio run -t monitor
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ОШИБКА] Монитор завершился с кодом %ERR%
)
echo.
pause
exit /b %ERR%
