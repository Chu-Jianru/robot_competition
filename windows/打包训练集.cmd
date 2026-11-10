@echo off
setlocal
cd /d "%~dp0.."
set "SRC=windows\test_data\red_train_20261110\yolo"
set "ZIP=training\red_train_yolo.zip"
if not exist "%SRC%\data.yaml" goto missing

if exist "%ZIP%" del /q "%ZIP%"
powershell -NoProfile -Command "Compress-Archive -Path '%SRC%\*' -DestinationPath '%ZIP%' -Force"
if errorlevel 1 goto failed

echo Archive ready: %cd%\%ZIP%
start "" explorer /select,"%cd%\%ZIP%"
exit /b

:missing
echo Dataset not found: %SRC%\data.yaml
echo Run export first:
echo   python windows\export_yolo.py windows\test_data\red_train_20261110 --val-every 5
pause
exit /b 1

:failed
echo Archive failed. Close any program using %ZIP% and retry.
pause
exit /b 1
