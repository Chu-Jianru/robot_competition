@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0check-firmware-sync.ps1"
pause
