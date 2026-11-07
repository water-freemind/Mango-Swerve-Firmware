$ErrorActionPreference = 'Stop'
$firmwareRoot = Split-Path $PSScriptRoot -Parent
$output = Join-Path $firmwareRoot 'build/host-tests'
New-Item -ItemType Directory -Force $output | Out-Null
$compiler = (Get-Command gcc.exe -ErrorAction Stop).Source
$flags = @('-std=c11','-Wall','-Wextra','-Werror','-DTEST_CAN_HAL')
foreach($path in @('src/app','src/service','src/domain','src/device','src/device/bus_motor','src/infra','src/platform','tests/fakes')) {
    $flags += "-I$(Join-Path $firmwareRoot $path)"
}
function Invoke-ChassisTest([string]$name, [string[]]$sources, [int]$mode=6) {
    $exe = Join-Path $output "$name-$mode.exe"
    $args = $flags + @("-DAPP_RUN_MODE=$mode") + @($sources | ForEach-Object {Join-Path $firmwareRoot $_}) + @('-lm','-o',$exe)
    & $compiler @args
    if($LASTEXITCODE -ne 0) { throw "$name compilation failed (mode=$mode)." }
    & $exe
    if($LASTEXITCODE -ne 0) { throw "$name failed (mode=$mode)." }
}
Invoke-ChassisTest 'kine' @('tests/test_steer_wheel_kine.c','src/domain/steer_wheel_kine.c')
Invoke-ChassisTest 'profile' @('tests/test_chassis_profile.c','src/service/chassis_profile.c','src/domain/steer_wheel_kine.c')
Invoke-ChassisTest 'dji-timing' @('tests/test_dji_motor_timing.c','src/device/bus_motor/dji_motor.c')
Invoke-ChassisTest 'dm-numeric' @('tests/test_dm_motor_numeric.c','src/device/bus_motor/dm_motor.c')
Invoke-ChassisTest 'trace' @('tests/test_chassis_trace.c','src/service/chassis_trace.c','src/domain/steer_wheel_kine.c')
foreach($mode in @(0,5,6)) {
    Invoke-ChassisTest 'chassis-control' @('tests/test_chassis_control.c','src/service/chassis.c','src/device/bus_motor/agv_motor.c','src/domain/steer_wheel_kine.c') $mode
}
