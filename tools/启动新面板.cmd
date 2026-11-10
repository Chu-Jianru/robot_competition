@echo off
setlocal
set "ROOT=%~dp0.."
set "V=%ROOT%\vendor\PushRobot-Source-20261102-WHEEL_SPEED_V1"
set "PY=%ROOT%\tools\panel-venv\Scripts\python.exe"

if not exist "%PY%" goto nodeps
"%PY%" -c "import serial, cv2, PIL" >nul 2>&1
if errorlevel 1 goto nodeps

set "PYTHONPATH=%V%\debug-workbench;%V%\esp32-pc-link\pc;%V%\motor-bringup"
set "ROBOT_WORKBENCH_DATA_DIR=%ROOT%\tools\panel-data"
set "PYTHONDONTWRITEBYTECODE=1"

if not exist "%ROBOT_WORKBENCH_DATA_DIR%" mkdir "%ROBOT_WORKBENCH_DATA_DIR%"

"%PY%" "%V%\debug-workbench\main.py" --mode chassis %*
exit /b %errorlevel%

:nodeps
echo.
echo Panel environment missing or incomplete.
echo Run once from the project root:
echo   python -m venv tools\panel-venv
echo   tools\panel-venv\Scripts\python.exe -m pip install -r tools\panel-requirements.txt --index-url https://pypi.org/simple
echo.
pause
exit /b 1
