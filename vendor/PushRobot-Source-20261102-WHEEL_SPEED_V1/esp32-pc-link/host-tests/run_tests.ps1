$ErrorActionPreference = 'Stop'
Set-Location $PSScriptRoot
New-Item -ItemType Directory -Force -Path build | Out-Null
& g++ -std=c++17 -Wall -Wextra -Werror -I../lib/link_core ../lib/link_core/link_core.cpp test_link_core.cpp -o build/test_link_core.exe
if ($LASTEXITCODE -ne 0) { exit $LASTEXITCODE }
& ./build/test_link_core.exe
exit $LASTEXITCODE
