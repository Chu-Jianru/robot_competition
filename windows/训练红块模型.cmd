@echo off
cd /d "%~dp0.."
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" set "ROBOT_PY=python"

"%ROBOT_PY%" training\train_launcher.py %*
set "CODE=%ERRORLEVEL%"
echo.
if not "%CODE%"=="0" echo Training exited with code %CODE%
pause
