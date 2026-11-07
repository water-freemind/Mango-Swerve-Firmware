param([string]$TraceDirectory = '')
$ErrorActionPreference = 'Stop'
$firmwareRoot = Split-Path $PSScriptRoot -Parent
$out = Join-Path $firmwareRoot 'build/host-tests'
New-Item -ItemType Directory -Force $out | Out-Null
$compiler = (Get-Command gcc.exe -ErrorAction Stop).Source
$exe = Join-Path $out 'test_auto_geometry.exe'
& $compiler -std=c11 -Wall -Wextra -Werror "-I$firmwareRoot/src/service" `
    "$PSScriptRoot/test_auto_geometry.c" "$firmwareRoot/src/service/auto_geometry.c" -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous geometry compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous geometry tests failed' }
$exe = Join-Path $out 'test_auto_motion.exe'
& $compiler -std=c11 -Wall -Wextra -Werror "-I$firmwareRoot/src/service" `
    "$PSScriptRoot/test_auto_motion.c" "$firmwareRoot/src/service/auto_motion.c" `
    "$firmwareRoot/src/service/auto_geometry.c" -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous motion compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous motion tests failed' }
$exe = Join-Path $out 'test_auto_work.exe'
& $compiler -std=c11 -Wall -Wextra -Werror "-I$firmwareRoot/src/service" `
    "$PSScriptRoot/test_auto_work.c" "$firmwareRoot/src/service/auto_work.c" -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous work compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous work tests failed' }
$exe = Join-Path $out 'test_auto_observation.exe'
& $compiler -std=c11 -Wall -Wextra -Werror "-I$firmwareRoot/src/service" `
    "$PSScriptRoot/test_auto_observation.c" "$firmwareRoot/src/service/auto_observation.c" `
    "$firmwareRoot/src/service/auto_route.c" "$firmwareRoot/src/service/auto_geometry.c" -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous observation compilation failed' }
& $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous observation tests failed' }
$exe = Join-Path $out 'test_auto_mission.exe'
& $compiler -std=c11 -Wall -Wextra -Werror "-I$firmwareRoot/src/service" `
    "$PSScriptRoot/test_auto_mission.c" "$firmwareRoot/src/service/auto_mission.c" `
    "$firmwareRoot/src/service/auto_route.c" "$firmwareRoot/src/service/auto_work.c" `
    "$firmwareRoot/src/service/auto_motion.c" "$firmwareRoot/src/service/auto_geometry.c" `
    "$firmwareRoot/src/service/auto_observation.c" -lm -o $exe
if ($LASTEXITCODE -ne 0) { throw 'Autonomous mission compilation failed' }
if (!$TraceDirectory) { $TraceDirectory = Join-Path $firmwareRoot 'build/autonomy-prep/traces' }
New-Item -ItemType Directory -Force $TraceDirectory | Out-Null
& $exe $TraceDirectory
if ($LASTEXITCODE -ne 0) { throw 'Autonomous mission tests failed' }
