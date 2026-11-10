@echo off
setlocal
set "PY=%~dp0esp32-pc-link\pc\.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo Missing Python environment. Run esp32-pc-link\pc\setup.cmd first.
    pause
    exit /b 1
)
"%PY%" "%~dp0debug-workbench\main.py" %*
if errorlevel 1 pause
