@echo off
setlocal
cd /d "%~dp0.."
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" set "ROBOT_PY=python"

rem Drag a frames folder onto this file to label it. Default: the merged dataset
rem (stream frames + phone photos + older photo batches).
set "TARGET=%~1"
if "%TARGET%"=="" set "TARGET=windows\test_data\red_train_20261110\frames"
if not exist "%TARGET%" goto missing

"%ROBOT_PY%" windows\label_boxes.py "%TARGET%" --prefill
if errorlevel 1 pause
exit /b

:missing
echo Folder not found: %TARGET%
echo.
echo Drag the "frames" folder onto this file, or edit TARGET inside it.
echo Expected default: windows\test_data\red_train_20261110\frames
pause
exit /b 1
