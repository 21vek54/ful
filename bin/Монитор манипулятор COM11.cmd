@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0..\firmware\manipulator_com11"
echo [МОНИТОР] Манипулятор COM11
pio run -t monitor
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ОШИБКА] Монитор завершился с кодом %ERR%
)
echo.
pause
exit /b %ERR%
