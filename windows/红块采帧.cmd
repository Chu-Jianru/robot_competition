@echo off
setlocal
cd /d "%~dp0.."
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" set "ROBOT_PY=python"
set "TARGET=%~1"
if "%TARGET%"=="" set "TARGET=windows\test_data\red_train_20261110"

rem Manual capture: S saves a frame, Q quits. Orientation is detected from the
rem blue/yellow floor, so the phone can be placed any way round.
"%ROBOT_PY%" windows\collect_red_frames.py --scrcpy-dir "E:\scrcpy\scrcpy-win64-v5.0.1" --manual --rotate auto --out "%TARGET%"
if errorlevel 1 pause
