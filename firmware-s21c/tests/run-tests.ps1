$ErrorActionPreference='Stop'
$root=Split-Path $PSScriptRoot -Parent
New-Item -ItemType Directory -Force "$root/build" | Out-Null
$exe=Join-Path $root 'build/test_stp23l.exe'
& gcc.exe -std=c11 -Wall -Wextra -Werror "-I$root/src" "$PSScriptRoot/test_stp23l.c" "$root/src/stp23l.c" -o $exe
if($LASTEXITCODE -ne 0) {throw 'Parser test compilation failed'}
& $exe
if($LASTEXITCODE -ne 0) {throw 'Parser regression failed'}
$dm=Join-Path (Split-Path $root -Parent) 'firmware'
$exe=Join-Path $root 'build/test_dm_compat.exe'
& gcc.exe -std=c11 -Wall -Wextra -Werror -DS21_TEST_DM "-I$root/src" "-I$dm/src/service" "-I$dm/tests/fakes" `
    "$PSScriptRoot/test_stp23l.c" "$root/src/stp23l.c" "$dm/src/service/laser.c" -o $exe
if($LASTEXITCODE -ne 0) {throw 'DM compatibility compilation failed'}
& $exe
if($LASTEXITCODE -ne 0) {throw 'DM compatibility regression failed'}
$exe=Join-Path $root 'build/test_transport.exe'
& gcc.exe -std=c11 -Wall -Wextra -Werror "-I$PSScriptRoot/fakes" "-I$root/src" `
    "$PSScriptRoot/test_transport.c" "$root/src/s21_transport.c" -o $exe
if($LASTEXITCODE -ne 0) {throw 'Transport compilation failed'}
& $exe
if($LASTEXITCODE -ne 0) {throw 'Transport regression failed'}
$exe=Join-Path $root 'build/test_board.exe'
& gcc.exe -std=c11 -Wall -Wextra -Werror "-I$PSScriptRoot/fakes" "-I$root/src" `
    "$PSScriptRoot/test_board.c" "$root/src/s21_board.c" "$root/src/stp23l.c" -o $exe
if($LASTEXITCODE -ne 0) {throw 'Board adapter compilation failed'}
& $exe
if($LASTEXITCODE -ne 0) {throw 'Board adapter regression failed'}
