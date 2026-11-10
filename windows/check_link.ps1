<#
.SYNOPSIS
  手机链路体检：USB 枚举、adb 状态、连续 shell、推包、取流，一次跑完给结论。

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\windows\check_link.ps1

.EXAMPLE
  powershell -ExecutionPolicy Bypass -File .\windows\check_link.ps1 -Serial 192.168.1.23:5555
#>
param(
    [string]$ScrcpyDir = 'E:\scrcpy\scrcpy-win64-v5.0.1',
    [string]$Serial = '',
    [int]$EchoCount = 15,
    [int]$PushCount = 3,
    [int]$Frames = 30
)

$ErrorActionPreference = 'Continue'
$adb = Join-Path $ScrcpyDir 'adb.exe'
$python = Join-Path $ScrcpyDir '.venv\Scripts\python.exe'
$bench = Join-Path $PSScriptRoot 'live_bench.py'
if (-not (Test-Path $adb)) { Write-Host "找不到 adb：$adb" -ForegroundColor Red; exit 2 }
if (-not (Test-Path $python)) { $python = 'python' }

function Section($text) {
    Write-Host ""
    Write-Host "=== $text ===" -ForegroundColor Cyan
}

function Get-DeviceLines {
    $lines = @()
    foreach ($line in (& $adb devices 2>$null)) {
        if ($line -match "^\S+\s+(device|offline|unauthorized|authorizing)\s*$") {
            $lines += $line.Trim()
        }
    }
    return $lines
}

# 1. USB 枚举
Section '1. USB 枚举'
$usb = & pnputil /enum-devices /connected /class USB 2>&1 | Select-String '12D1|Unknown USB'
if ($usb) {
    $usb | ForEach-Object { Write-Host ('  ' + $_.Line.Trim()) }
} else {
    Write-Host '  没有看到 12D1（华为）或 Unknown USB 设备' -ForegroundColor Yellow
}
$adbIf = & pnputil /enum-devices /connected /class USBDevice 2>&1 | Select-String 'ADB Interface'
if ($adbIf) { Write-Host '  ADB Interface 已枚举' -ForegroundColor Green }
else { Write-Host '  没有 ADB Interface（调试接口没起来）' -ForegroundColor Yellow }

# 2. adb 状态
Section '2. adb 状态'
$deviceLines = Get-DeviceLines
if (-not $deviceLines) {
    Write-Host '  adb 看不到任何设备' -ForegroundColor Red
} else {
    $deviceLines | ForEach-Object { Write-Host ('  ' + $_) }
}
if (-not $Serial) {
    $online = $deviceLines | Where-Object { $_ -match '\sdevice$' } | Select-Object -First 1
    if ($online) { $Serial = ($online -split '\s+')[0] }
    else {
        $any = $deviceLines | Select-Object -First 1
        if ($any) { $Serial = ($any -split '\s+')[0] }
    }
}
if ($Serial) { Write-Host "  使用设备：$Serial" }

if (-not $Serial) {
    Write-Host ''
    Write-Host '结论：手机没有出现在 adb 里。先看第 1 步：有 Unknown USB 就是线/口/尾插，什么都没有就检查线和 USB 调试。' -ForegroundColor Red
    exit 1
}

$state = ($deviceLines | Where-Object { $_ -match [regex]::Escape($Serial) } | Select-Object -First 1)
if ($state -match 'unauthorized') {
    Write-Host ''
    Write-Host '结论：设备未授权。解锁手机，点“允许 USB 调试”（勾始终允许），再重跑。' -ForegroundColor Red
    exit 1
}
if ($state -match 'offline') {
    Write-Host ''
    Write-Host '结论：设备 offline。手机开发者选项里撤销 USB 调试授权，拔插；仍不行重启手机。' -ForegroundColor Red
    exit 1
}

# 3. 连续 shell
Section "3. 连续 shell（$EchoCount 次）"
$echoFail = 0
$sw = [Diagnostics.Stopwatch]::StartNew()
for ($i = 1; $i -le $EchoCount; $i++) {
    $r = & $adb -s $Serial shell echo ok 2>&1
    if (($r -join ' ') -notmatch 'ok') {
        $echoFail++
        Write-Host ("  第 $i 次失败：" + (($r | Select-Object -First 1))) -ForegroundColor Yellow
    }
}
$sw.Stop()
Write-Host ("  失败 $echoFail / $EchoCount，用时 {0:N1} s" -f $sw.Elapsed.TotalSeconds)
if ($echoFail -eq 0) { Write-Host '  控制通道稳定' -ForegroundColor Green }

# 4. 推包
Section "4. 推包（$PushCount 次，734 KB）"
$src = Join-Path $ScrcpyDir 'scrcpy-server'
$pushFail = 0
for ($i = 1; $i -le $PushCount; $i++) {
    $sw = [Diagnostics.Stopwatch]::StartNew()
    $r = & $adb -s $Serial push $src "/data/local/tmp/linkcheck-$i.jar" 2>&1
    $sw.Stop()
    if ($LASTEXITCODE -ne 0 -or (($r -join ' ') -notmatch '1 file pushed')) {
        $pushFail++
        Write-Host ("  第 $i 次失败（{0:N2} s）：{1}" -f $sw.Elapsed.TotalSeconds, (($r | Select-Object -Last 1))) -ForegroundColor Yellow
    } else {
        Write-Host ("  第 $i 次成功 {0:N2} s" -f $sw.Elapsed.TotalSeconds) -ForegroundColor Green
    }
    & $adb -s $Serial shell "rm -f /data/local/tmp/linkcheck-$i.jar" 2>&1 | Out-Null
}
if ($pushFail -eq 0) { Write-Host '  数据传输稳定' -ForegroundColor Green }

# 5. 取流
Section "5. 取流（$Frames 帧）"
$tempOut = Join-Path $env:TEMP ('linkcheck_' + (Get-Date -Format 'HHmmss'))
$stream = & $python $bench --scrcpy-dir $ScrcpyDir --serial $Serial --frames $Frames --out $tempOut 2>&1
$streamCode = $LASTEXITCODE
$stream | Where-Object { $_ -match '^(首帧|帧 |===|画面|tag|红块|毫米筛|耗时|PC age|判定)' } | ForEach-Object { Write-Host ('  ' + $_) }
if ($streamCode -eq 0) { Write-Host '  取流正常' -ForegroundColor Green }
else { Write-Host '  取流中断（见上面的报错）' -ForegroundColor Yellow }

# 结论
Section '结论'
if ($echoFail -eq 0 -and $pushFail -eq 0 -and $streamCode -eq 0) {
    Write-Host '链路正常：shell、推包、取流都通过，可以跑 capture_hi.py 做台架验收。' -ForegroundColor Green
    exit 0
}
if ($pushFail -gt 0) {
    Write-Host '推包失败 = USB 数据通道不稳：换数据线（不是换口），或换一台电脑对比。' -ForegroundColor Red
} elseif ($echoFail -gt 0) {
    Write-Host '连续 shell 失败 = 控制通道不稳：先撤销 USB 授权/重启手机，再查线。' -ForegroundColor Red
} else {
    Write-Host 'shell 和推包都过，但取流中断：把上面 5 的报错发我；若报“画面停滞”，多半还是 USB 在高码率下掉线。' -ForegroundColor Red
}
exit 1
