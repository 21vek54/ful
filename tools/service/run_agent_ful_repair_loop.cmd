@echo off
setlocal

set "ROOT=%~dp0..\..\"
set "LOOP=%ROOT%tools\service\agent_ful_repair_loop.py"

if "%~1"=="" goto :usage

set "GOAL_FILE=%~f1"
if not exist "%GOAL_FILE%" (
  echo [ERROR] Goal file not found:
  echo   %GOAL_FILE%
  exit /b 2
)

echo Starting FULL repair loop...
echo   goal: %GOAL_FILE%
echo.

if "%~2"=="" (
  python "%LOOP%" --goal-file "%GOAL_FILE%"
) else (
  python "%LOOP%" --goal-file "%GOAL_FILE%" --api-key-file "%~f2"
)
set "RC=%ERRORLEVEL%"

echo.
echo Loop exit code: %RC%
exit /b %RC%

:usage
echo Usage:
echo   %~nx0 ^<goal.txt^> [api_key_file.txt]
echo.
echo Example:
echo   %~nx0 C:\temp\ful_repair_goal.txt
echo   %~nx0 C:\temp\ful_repair_goal.txt C:\temp\openai_api_key_local.txt
exit /b 1
