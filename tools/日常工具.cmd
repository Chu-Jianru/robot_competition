@echo off
setlocal
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" goto missing
"%ROBOT_PY%" "%~dp0..\windows\daily_tools.py" %*
if errorlevel 1 pause
exit /b
:missing
echo Python environment not found: %ROBOT_PY%
pause
exit /b 1
