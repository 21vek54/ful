@echo off
setlocal

set "ROOT=%~dp0..\..\"
set "BRIDGE=%ROOT%tools\service\agent_autorun_bridge.py"

if "%~1"=="" goto :usage

set "INPUT=%~f1"
if not exist "%INPUT%" (
  echo [ERROR] Input file not found:
  echo   %INPUT%
  exit /b 2
)

set "RESULT=%~dpn1.autorun_result.json"
set "NEXT_PROMPT=%~dpn1.next_prompt.txt"

if not "%~2"=="" set "RESULT=%~f2"
if not "%~3"=="" set "NEXT_PROMPT=%~f3"

echo Running AUTO_RUN bridge...
echo   input:       %INPUT%
echo   result json: %RESULT%
echo.

python "%BRIDGE%" --input-file "%INPUT%" --result-file "%RESULT%"
set "RC=%ERRORLEVEL%"

if not exist "%RESULT%" (
  echo.
  echo [ERROR] Bridge did not create result JSON:
  echo   %RESULT%
  exit /b 3
)

(
  echo AUTO_RUN_BRIDGE_RESULT_JSON:
  type "%RESULT%"
) > "%NEXT_PROMPT%"

echo.
echo Prepared payload for next prompt:
echo   %NEXT_PROMPT%
echo.
echo ===== NEXT PROMPT PAYLOAD START =====
type "%NEXT_PROMPT%"
echo ===== NEXT PROMPT PAYLOAD END =====
echo Bridge exit code: %RC%

exit /b %RC%

:usage
echo Usage:
echo   %~nx0 ^<agent_response.txt^> [result.json] [next_prompt_payload.txt]
echo.
echo Example:
echo   %~nx0 C:\temp\agent_reply.txt
echo   %~nx0 C:\temp\agent_reply.txt C:\temp\bridge_result.json C:\temp\next_prompt.txt
exit /b 1
