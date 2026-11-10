@echo off
setlocal
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo Missing .venv. Run setup.cmd first.
    exit /b 1
)
"%PY%" "%~dp0camera_preview.py" %*
exit /b %ERRORLEVEL%
