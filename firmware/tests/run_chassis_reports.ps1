$ErrorActionPreference='Stop'
$firmwareRoot=Split-Path $PSScriptRoot -Parent
$repoRoot=Split-Path $firmwareRoot -Parent
$out=Join-Path $firmwareRoot 'build/chassis-optimization'
$gcc=(Get-Command gcc.exe -ErrorAction Stop).Source
New-Item -ItemType Directory -Force $out | Out-Null
$flags=@('-std=c11','-Wall','-Wextra','-Werror','-DAPP_RUN_MODE=6')
foreach($p in @('src/app','src/service','src/domain','src/device','src/infra','tests')) {$flags+="-I$(Join-Path $firmwareRoot $p)"}
foreach($kind in @('legacy','candidate')) {
    $args=$flags
    $source=Join-Path $firmwareRoot 'src/service/manual_ramp.c'
    if($kind -eq 'legacy') {$args+= '-DUSE_LEGACY_RAMP';$source=Join-Path $PSScriptRoot 'fixtures/manual_ramp_legacy.c'}
    $exe=Join-Path $out "ramp-$kind.exe"
    & $gcc @args (Join-Path $PSScriptRoot 'manual_ramp_trace.c') $source '-lm' '-o' $exe
    if($LASTEXITCODE -ne 0) {throw "Ramp $kind fixture compilation failed"}
    & $exe | Set-Content -Encoding ASCII (Join-Path $out "ramp-$kind.csv")
    if($LASTEXITCODE -ne 0) {throw "Ramp $kind fixture failed"}
}
$clearance=Join-Path $out 'test_clearance_replay.exe'
& $gcc @flags (Join-Path $PSScriptRoot 'test_clearance_replay.c') `
    (Join-Path $firmwareRoot 'src/service/auto_geometry.c') `
    (Join-Path $firmwareRoot 'src/service/straight_test.c') '-lm' '-o' $clearance
if($LASTEXITCODE -ne 0) {throw 'Clearance fixture compilation failed'}
& $clearance (Join-Path $out 'clearance-replay.csv')
if($LASTEXITCODE -ne 0) {throw 'Clearance fixture failed'}
Push-Location $repoRoot
try {
    & py -3 -B tools/manual_ramp_report.py --legacy "$out/ramp-legacy.csv" --candidate "$out/ramp-candidate.csv" --out "$out/ramp-report" > "$out/ramp-report.log"
    if($LASTEXITCODE -ne 0) {throw 'Ramp report failed'}
    & py -3 -B tools/clearance_report.py "$out/clearance-replay.csv" --output "$out/clearance-report.json"
    if($LASTEXITCODE -ne 0) {throw 'Clearance report failed'}
    & py -3 -B tools/chassis_trace_report.py (Join-Path $firmwareRoot 'build/host-tests/chassis-trace-export.log') --source-kind synthetic --output "$out/trace-report"
    if($LASTEXITCODE -ne 0) {throw 'Actual C adapter export report failed'}
    Write-Host 'CHASSIS_REPORTS_OK command curves, synthetic clearance, actual C chunk export roundtrip; no hardware'
} finally {Pop-Location}
