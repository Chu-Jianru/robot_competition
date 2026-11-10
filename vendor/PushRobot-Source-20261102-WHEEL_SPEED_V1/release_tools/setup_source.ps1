$ErrorActionPreference = 'Stop'
# 可从 release_tools/ 直接运行，也可作为分享包根目录的 setup_source.ps1 运行。
$taskRoot = $PSScriptRoot
if (-not (Test-Path -LiteralPath (Join-Path $taskRoot 'motor-bringup'))) {
    $taskRoot = Split-Path -Parent $PSScriptRoot
}
if (-not (Test-Path -LiteralPath (Join-Path $taskRoot 'motor-bringup'))) {
    throw 'Project root not found. Extract the entire source ZIP first.'
}
Set-Location $taskRoot

# 分享包的 source/ 是独立项目根目录；不复制发送人的虚拟环境。
$taskPython = $null
$taskPythonArgs = @()
if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3 -c 'import sys, tkinter; sys.exit(0 if sys.version_info >= (3,12) else 1)'
    if ($LASTEXITCODE -eq 0) { $taskPython = 'py'; $taskPythonArgs = @('-3') }
}
if (-not $taskPython -and (Get-Command python -ErrorAction SilentlyContinue)) {
    & python -c 'import sys, tkinter; sys.exit(0 if sys.version_info >= (3,12) else 1)'
    if ($LASTEXITCODE -eq 0) { $taskPython = 'python' }
}
if (-not $taskPython) { throw 'Install Python 3.12+ (64-bit, with tkinter), then retry.' }

$taskVenv = Join-Path $taskRoot 'esp32-pc-link\pc\.venv'
if (-not (Test-Path -LiteralPath (Join-Path $taskVenv 'Scripts\python.exe'))) {
    & $taskPython @taskPythonArgs -m venv $taskVenv
    if ($LASTEXITCODE -ne 0) { throw 'Failed to create the virtual environment.' }
}
$taskVenvPython = Join-Path $taskVenv 'Scripts\python.exe'
& $taskVenvPython -m pip install -r (Join-Path $taskRoot 'esp32-pc-link\pc\requirements.txt')
if ($LASTEXITCODE -ne 0) { throw 'Dependency installation failed. Check the network and Python version.' }
& $taskVenvPython -c 'import tkinter, serial, cv2, numpy, PIL; print("Source environment ready.")'
if ($LASTEXITCODE -ne 0) { throw 'Dependency verification failed.' }
Write-Host 'Ready. Run run_debug.cmd to start; run_checks.cmd performs offline tests only.'
