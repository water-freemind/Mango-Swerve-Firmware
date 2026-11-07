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
