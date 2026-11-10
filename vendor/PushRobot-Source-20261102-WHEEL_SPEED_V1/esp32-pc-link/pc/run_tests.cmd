@echo off
setlocal
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo Missing .venv. Run setup.cmd first.
    exit /b 1
)
pushd "%~dp0"
"%PY%" -m unittest discover -s tests -v
set "RC=%ERRORLEVEL%"
popd
exit /b %RC%
