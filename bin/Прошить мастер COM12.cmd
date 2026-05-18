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
echo [ПРОШИВКА] Мастер COM12 (production, version_5)
echo [ИСТОЧНИК] %CD%
echo [ИНФО] Минимальный firmware/master_control_com12 не является production-прошивкой.
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
