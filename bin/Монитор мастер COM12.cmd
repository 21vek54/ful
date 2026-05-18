@echo off
setlocal
chcp 65001 >nul
set "PROJECT_DIR=%~dp0..\archive\firmware\master_control_com12_legacy_2026-03-25"
cd /d "%PROJECT_DIR%"
if errorlevel 1 (
  echo [ОШИБКА] Не найден production-каталог COM12: "%PROJECT_DIR%"
  pause
  exit /b 1
)
echo [МОНИТОР] Мастер COM12 (production, version_5)
echo [ИСТОЧНИК] %CD%
pio run -t monitor
set ERR=%ERRORLEVEL%
if not "%ERR%"=="0" (
  echo [ОШИБКА] Монитор завершился с кодом %ERR%
)
echo.
pause
exit /b %ERR%
