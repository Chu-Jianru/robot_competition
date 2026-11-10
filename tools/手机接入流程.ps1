[CmdletBinding(DefaultParameterSetName = 'Phone')]
param(
    [Parameter(ParameterSetName = 'Check')]
    [switch]$Check,

    [Parameter(ParameterSetName = 'Daily')]
    [switch]$Daily,

    [Parameter(ParameterSetName = 'List')]
    [switch]$List
)

$robotPython = 'E:\scrcpy\scrcpy-win64-v5.0.1\.venv\Scripts\python.exe'
$robotMenu = Join-Path $PSScriptRoot '..\windows\daily_tools.py'
if (-not (Test-Path -LiteralPath $robotMenu -PathType Leaf)) {
    $robotMenu = 'E:\robot_competition\windows\daily_tools.py'
}

if (-not (Test-Path -LiteralPath $robotPython -PathType Leaf) -or
    -not (Test-Path -LiteralPath $robotMenu -PathType Leaf)) {
    Write-Host 'Robot project or Python environment not found.' -ForegroundColor Red
    exit 1
}

$robotArguments = @(switch ($PSCmdlet.ParameterSetName) {
    'Check' { '--check-phone' }
    'List'  { '--list' }
    'Phone' { '--phone-flow' }
})

$robotPreviousUtf8 = $env:PYTHONUTF8
$robotPreviousColor = $env:ROBOT_MENU_COLOR
try {
    $env:PYTHONUTF8 = '1'
    $env:ROBOT_MENU_COLOR = '1'
    & $robotPython -u $robotMenu @robotArguments
    $robotExitCode = $LASTEXITCODE
} finally {
    $env:PYTHONUTF8 = $robotPreviousUtf8
    $env:ROBOT_MENU_COLOR = $robotPreviousColor
}
exit $robotExitCode
