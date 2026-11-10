$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot

Write-Host "Checking system Python and tkinter..."
& python -c "import sys; print(sys.executable); import tkinter; print('tkinter OK')"
if ($LASTEXITCODE -ne 0) {
    throw "System Python is unavailable or missing tkinter. The panel needs a Python with tkinter."
}

if (-not (Test-Path ".venv")) {
    Write-Host "Creating virtual environment .venv ..."
    & python -m venv .venv
    if ($LASTEXITCODE -ne 0) { throw "Failed to create venv" }
}

Write-Host "Installing pyserial, opencv-python, Pillow..."
& .\.venv\Scripts\python.exe -m pip install --upgrade pip
& .\.venv\Scripts\python.exe -m pip install -r requirements.txt
if ($LASTEXITCODE -ne 0) { throw "Dependency installation failed" }

& .\.venv\Scripts\python.exe -c "import serial, cv2, tkinter, PIL; print('pyserial', serial.__version__); print('opencv', cv2.__version__); print('Pillow', PIL.__version__); print('tkinter OK')"
Write-Host ""
Write-Host "Done. Start the panel with:"
Write-Host "  ..\..\run_debug.cmd"
Write-Host "Camera preview:"
Write-Host "  .\run_camera.cmd --list"
Write-Host "  .\run_camera.cmd --source 0"
