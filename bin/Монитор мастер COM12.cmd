@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0..\firmware\master_control_com12"
echo [МОНИТОР] Мастер COM12
pio run -t monitor
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ОШИБКА] Монитор завершился с кодом %ERR%
)
echo.
pause
exit /b %ERR%
