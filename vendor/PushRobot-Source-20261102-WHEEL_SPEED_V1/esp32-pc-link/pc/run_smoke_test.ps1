param(
    [string]$Port = "COM4",
    [switch]$SkipManual
)
$ErrorActionPreference = 'Stop'
$python = Join-Path $PSScriptRoot ".venv\Scripts\python.exe"
if (-not (Test-Path $python)) {
    throw "Missing .venv. Run .\setup.ps1 first."
}
$script = Join-Path $PSScriptRoot "link_smoke_test.py"
$arguments = @($script, "--port", $Port)
if ($SkipManual) { $arguments += "--skip-manual" }
& $python @arguments
exit $LASTEXITCODE
