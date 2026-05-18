@echo off
setlocal

set "ROOT=%~dp0..\..\"
set "LOOP=%ROOT%tools\service\agent_autorun_openai_loop.py"

if "%~1"=="" goto :usage

set "PROMPT_FILE=%~f1"
if not exist "%PROMPT_FILE%" (
  echo [ERROR] Prompt file not found:
  echo   %PROMPT_FILE%
  exit /b 2
)

echo Starting OpenAI auto-run loop...
echo   prompt: %PROMPT_FILE%
echo.

if "%~2"=="" (
  python "%LOOP%" --prompt-file "%PROMPT_FILE%"
) else (
  python "%LOOP%" --prompt-file "%PROMPT_FILE%" --api-key-file "%~f2"
)
set "RC=%ERRORLEVEL%"

echo.
echo Loop exit code: %RC%
exit /b %RC%

:usage
echo Usage:
echo   %~nx0 ^<initial_prompt.txt^> [api_key_file.txt]
echo.
echo Example:
echo   %~nx0 C:\temp\bench_prompt.txt
echo   %~nx0 C:\temp\bench_prompt.txt C:\temp\openai_api_key_local.txt
exit /b 1
