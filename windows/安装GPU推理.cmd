@echo off
cd /d "%~dp0.."
set "PYTHONUTF8=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" set "ROBOT_PY=python"
set "TARGET=.deps-gpu"
set "VERSION=1.20.2"
set "PYTHONPATH=%cd%\.deps-gpu;%cd%\.deps"

rem onnxruntime-gpu 1.21+ needs CUDA 13, while PyTorch here ships CUDA 12.6.
rem Pin 1.20.2 (CUDA 12 + cuDNN 9) and skip the download only for that version.
"%ROBOT_PY%" -c "import sys; sys.path.insert(0, r'%cd%\.deps-gpu'); import onnxruntime as o; sys.exit(0 if o.__version__.startswith('1.20') else 1)" >nul 2>&1
if not errorlevel 1 goto check

echo Installing onnxruntime-gpu %VERSION% (CUDA 12 build) into .deps-gpu ...
echo About 200 MB, one time only.
if exist "%TARGET%\onnxruntime" rmdir /s /q "%TARGET%\onnxruntime"
for /d %%D in ("%TARGET%\onnxruntime_gpu-*") do rmdir /s /q "%%D"
"%ROBOT_PY%" -m pip install --target "%TARGET%" "onnxruntime-gpu==%VERSION%"
if errorlevel 1 goto failed

:check
"%ROBOT_PY%" -c "import sys; sys.path.insert(0, r'%cd%\windows'); from detect_onnx import ensure_runtime_paths; ensure_runtime_paths(); import onnxruntime as o; print('version', o.__version__); print('providers', o.get_available_providers())"
echo.
echo If CUDAExecutionProvider is listed, test with:
echo   python windows\detect_onnx.py --model windows\models\red_block.onnx --device gpu --limit 20
pause
exit /b 0

:failed
echo.
echo Install failed. Check network, then run manually:
echo   "%ROBOT_PY%" -m pip install --target .deps-gpu onnxruntime-gpu==%VERSION%
pause
exit /b 1
