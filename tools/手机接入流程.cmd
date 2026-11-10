@echo off
setlocal
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" goto missing
set "ROBOT_MODE=--phone-flow"
if /i "%~1"=="--check" set "ROBOT_MODE=--check-phone"
"%ROBOT_PY%" "%~dp0..\windows\daily_tools.py" %ROBOT_MODE%
set "ROBOT_RESULT=%ERRORLEVEL%"
if not "%ROBOT_RESULT%"=="0" if /i not "%~1"=="--check" pause
exit /b %ROBOT_RESULT%
:missing
echo Python environment not found: %ROBOT_PY%
if /i not "%~1"=="--check" pause
exit /b 1
