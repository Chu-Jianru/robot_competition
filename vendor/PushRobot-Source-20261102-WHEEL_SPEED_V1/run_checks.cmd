@echo off
setlocal
set "PYTHONUTF8=1"
"%~dp0esp32-pc-link\pc\.venv\Scripts\python.exe" "%~dp0release_tools\run_checks.py" %*
if errorlevel 1 pause
