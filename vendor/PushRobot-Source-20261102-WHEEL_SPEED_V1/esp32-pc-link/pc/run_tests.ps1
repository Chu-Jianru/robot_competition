$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $python)) {
    throw "Missing .venv. Run .\setup.ps1 first."
}
Set-Location $PSScriptRoot
& $python -m unittest discover -s tests -v
exit $LASTEXITCODE
