@echo off
setlocal
set "PYTHONUTF8=1"
set "ROBOT_MENU_COLOR=1"
set "ROBOT_PY=E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe"
if not exist "%ROBOT_PY%" goto missing
"%ROBOT_PY%" -u "%~dp0..\windows\daily_tools.py" %*
set "ROBOT_RESULT=%ERRORLEVEL%"
if not "%ROBOT_RESULT%"=="0" if /i not "%~1"=="--check-phone" pause
exit /b %ROBOT_RESULT%
:missing
echo Python environment not found: %ROBOT_PY%
if /i not "%~1"=="--check-phone" pause
exit /b 1
