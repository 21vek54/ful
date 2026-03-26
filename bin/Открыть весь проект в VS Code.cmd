@echo off
setlocal
chcp 65001 >nul
cd /d "%~dp0.."
echo [VS CODE] Открываю рабочую область fyl.code-workspace
start "" "C:\Users\Пользователь\AppData\Local\Programs\Microsoft VS Code\Code.exe" "%CD%\fyl.code-workspace"
