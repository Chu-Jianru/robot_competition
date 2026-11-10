@echo off
cd /d "%~dp0.."
set "PHOTO_PYTHON=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%PHOTO_PYTHON%" set "PHOTO_PYTHON=python"
"%PHOTO_PYTHON%" windows\analyze_field_photos.py --input windows\test_data\field_no_robot_20261103 --annotate
if errorlevel 1 pause
