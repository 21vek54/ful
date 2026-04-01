@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0..\firmware\conveyor_control_com10"
echo [ПРОШИВКА] Конвейер COM10
pio run -t upload
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ОШИБКА] Прошивка завершилась с кодом %ERR%
) else (
  echo [OK] Прошивка завершена.
)
echo.
pause
exit /b %ERR%
