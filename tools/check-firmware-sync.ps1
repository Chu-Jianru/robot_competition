# Verify the firmware working copy matches the original zip (only motor_profile.h may differ).
$root = Split-Path -Parent $PSScriptRoot
$zip  = Join-Path $root "archive\deliverables\PushRobot-Source-20261102-WHEEL_SPEED_V1.zip"
$work = Join-Path $root "hardware\firmware\WHEEL_SPEED_V1"
$tmp  = Join-Path $env:TEMP ("fw-sync-" + [guid]::NewGuid().ToString("N"))

if (-not (Test-Path $zip))  { Write-Host "Missing source zip: $zip" -ForegroundColor Red; exit 2 }
if (-not (Test-Path $work)) { Write-Host "Missing working copy: $work" -ForegroundColor Red; exit 2 }

try {
  Expand-Archive -LiteralPath $zip -DestinationPath $tmp -Force
  $srcRoot = (Get-ChildItem $tmp -Directory | Select-Object -First 1).FullName
  $allow = @("motor-bringup\src\motor_profile.h")
  $diffs = New-Object System.Collections.Generic.List[string]

  foreach ($rel in @("motor-bringup", "esp32-pc-link\lib\link_core")) {
    $a = Join-Path $srcRoot $rel
    $b = Join-Path $work $rel
    if (-not (Test-Path $a)) { $diffs.Add("missing in zip: $rel"); continue }
    if (-not (Test-Path $b)) { $diffs.Add("missing in copy: $rel"); continue }

    foreach ($f in Get-ChildItem $a -Recurse -File) {
      $r = $f.FullName.Substring($srcRoot.Length + 1)
      $t = Join-Path $work $r
      if (-not (Test-Path $t)) { $diffs.Add("missing in copy: $r"); continue }
      if ($allow -contains $r) { continue }
      if ((Get-FileHash $f.FullName).Hash -ne (Get-FileHash $t).Hash) { $diffs.Add("differs: $r") }
    }
    foreach ($f in Get-ChildItem $b -Recurse -File) {
      $r = $f.FullName.Substring($work.Length + 1)
      if (-not (Test-Path (Join-Path $srcRoot $r))) { $diffs.Add("extra in copy: $r") }
    }
  }

  if ($diffs.Count -eq 0) {
    Write-Host "OK: working copy matches the source zip (only motor_profile.h differs)" -ForegroundColor Green
    exit 0
  }
  Write-Host ("Found " + $diffs.Count + " difference(s):") -ForegroundColor Yellow
  foreach ($d in $diffs) { Write-Host ("  " + $d) }
  exit 1
}
finally {
  Remove-Item $tmp -Recurse -Force -ErrorAction SilentlyContinue
}
