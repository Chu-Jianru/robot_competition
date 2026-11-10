@echo off
setlocal
cd /d "%~dp0.."
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" set "ROBOT_PY=python"
set "FRAMES=windows\test_data\red_train_20261110\frames"
set "MODEL=windows\models\red_block.onnx"

if not exist "%MODEL%" goto missing
if not exist "%FRAMES%\labels.json" goto missing_frames

echo === HSV baseline ===
"%ROBOT_PY%" windows\evaluate_detector.py --frames "%FRAMES%" --detector hsv
echo.
echo === ONNX model ===
"%ROBOT_PY%" windows\evaluate_detector.py --frames "%FRAMES%" --detector onnx --model "%MODEL%"
echo.
echo Compare F1 and ms above. Keep --detector hsv unless ONNX is at least as good
echo and under 100 ms per frame.
pause
exit /b

:missing
echo Model not found: %MODEL%
echo Put best.onnx there (see windows\models\README.md), then run this again.
pause
exit /b 1

:missing_frames
echo Labels not found: %FRAMES%\labels.json
pause
exit /b 1
