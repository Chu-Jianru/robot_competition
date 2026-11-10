@echo off
setlocal
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0release_tools\setup_source.ps1"
if errorlevel 1 pause
