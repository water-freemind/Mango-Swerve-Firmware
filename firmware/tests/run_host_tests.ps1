$ErrorActionPreference = 'Stop'

$firmwareRoot = Split-Path $PSScriptRoot -Parent
$outputDir = Join-Path $firmwareRoot 'build\host-tests'
$testExe = Join-Path $outputDir 'test_can3_logic.exe'
$compiler = Get-Command gcc.exe -ErrorAction SilentlyContinue

if ($null -eq $compiler) {
    throw 'gcc.exe was not found in PATH. Install MinGW GCC before running host tests.'
}

New-Item -ItemType Directory -Force $outputDir | Out-Null

$compileArgs = @(
    '-std=c11',
    '-Wall',
    '-Wextra',
    '-Werror',
    "-I$(Join-Path $PSScriptRoot 'fakes')",
    "-I$(Join-Path $firmwareRoot 'src\service')",
    (Join-Path $PSScriptRoot 'test_can3_logic.c'),
    (Join-Path $firmwareRoot 'src\service\can3_router.c'),
    (Join-Path $firmwareRoot 'src\service\laser.c'),
    '-o',
    $testExe
)

& $compiler.Source @compileArgs
if ($LASTEXITCODE -ne 0) {
    throw "Host test compilation failed with exit code $LASTEXITCODE."
}

& $testExe
if ($LASTEXITCODE -ne 0) {
    throw "Host tests failed with exit code $LASTEXITCODE."
}

$commissioningExe = Join-Path $outputDir 'test_commissioning.exe'
$freshnessExe = Join-Path $outputDir 'test_input_freshness.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_RX_TIME_RACE' `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src/service')" `
    "-I$(Join-Path $firmwareRoot 'src/device')" `
    "-I$(Join-Path $firmwareRoot 'src/platform')" `
    "-I$(Join-Path $firmwareRoot 'src/infra')" `
    (Join-Path $PSScriptRoot 'test_input_freshness.c') `
    (Join-Path $firmwareRoot 'src/service/laser.c') '-o' $freshnessExe
if ($LASTEXITCODE -ne 0) { throw 'Input freshness compilation failed.' }
& $freshnessExe
if ($LASTEXITCODE -ne 0) { throw 'Input freshness regression failed.' }
$commissioningSources = @(
    (Join-Path $PSScriptRoot 'test_commissioning.c'),
    (Join-Path $firmwareRoot 'src\platform\stm32_hal_can.c'),
    (Join-Path $firmwareRoot 'src\service\commissioning.c'),
    (Join-Path $firmwareRoot 'src\service\can3_bus.c'),
    (Join-Path $firmwareRoot 'src\service\can3_router.c'),
    (Join-Path $firmwareRoot 'src\service\laser.c')
)
foreach ($swap in @(0, 1)) {
    $commissioningArgs = @(
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-DTEST_CAN_HAL',
        "-DCHASSIS_CAN_BUSES_SWAPPED=$swap",
        "-I$(Join-Path $PSScriptRoot 'fakes')",
        "-I$(Join-Path $firmwareRoot 'src\app')",
        "-I$(Join-Path $firmwareRoot 'src\service')",
        "-I$(Join-Path $firmwareRoot 'src\platform')",
        "-I$(Join-Path $firmwareRoot 'src\infra')"
    ) + $commissioningSources + @('-o', $commissioningExe)
    & $compiler.Source @commissioningArgs
    if ($LASTEXITCODE -ne 0) { throw "Commissioning test compilation failed (swap=$swap)." }
    & $commissioningExe
    if ($LASTEXITCODE -ne 0) { throw "Commissioning test failed (swap=$swap)." }
    & $commissioningExe fail-can1
    if ($LASTEXITCODE -ne 0) { throw "CAN1 failure isolation test failed (swap=$swap)." }
}

$ioc = Get-Content -Raw (Join-Path $firmwareRoot 'robot.ioc')
$queryArgs = $commissioningArgs + @('-DAPP_RUN_MODE=3')
& $compiler.Source @queryArgs
if ($LASTEXITCODE -ne 0) { throw 'CAN2 query test compilation failed.' }
& $commissioningExe
if ($LASTEXITCODE -ne 0) { throw 'CAN2 query test failed.' }
$steeringSources = $commissioningSources + @(
    (Join-Path $firmwareRoot 'src\service\assemble\assemble_chassis.c'),
    (Join-Path $firmwareRoot 'src\service\chassis_profile.c'),
    (Join-Path $firmwareRoot 'src\domain\steer_wheel_kine.c'),
    (Join-Path $firmwareRoot 'src\device\bus_motor\dm_motor.c'),
    (Join-Path $firmwareRoot 'src\device\bus_motor\agv_motor.c')
)
foreach ($swap in @(0, 1)) {
    $steeringArgs = @(
        '-std=c11', '-Wall', '-Wextra', '-Werror', '-DTEST_CAN_HAL', '-DAPP_RUN_MODE=2',
        "-DCHASSIS_CAN_BUSES_SWAPPED=$swap", '-ffunction-sections', '-fdata-sections', '-Wl,--gc-sections',
        "-I$(Join-Path $PSScriptRoot 'fakes')",
        "-I$(Join-Path $firmwareRoot 'src\app')",
        "-I$(Join-Path $firmwareRoot 'src\service')",
        "-I$(Join-Path $firmwareRoot 'src\platform')",
        "-I$(Join-Path $firmwareRoot 'src\infra')",
        "-I$(Join-Path $firmwareRoot 'src\device')",
        "-I$(Join-Path $firmwareRoot 'src\domain')"
    ) + $steeringSources + @('-o', $commissioningExe)
    & $compiler.Source @steeringArgs
    if ($LASTEXITCODE -ne 0) { throw "Steering test compilation failed (swap=$swap)." }
    & $commissioningExe
    if ($LASTEXITCODE -ne 0) { throw "Steering test failed (swap=$swap)." }
}
$fdcan = Get-Content -Raw (Join-Path $firmwareRoot 'Core\Src\fdcan.c')
$expectedIoc = @(
    'FDCAN3.MessageRAMOffset=384',
    'FDCAN3.StdFiltersNbr=8',
    'FDCAN3.RxFifo0ElmtsNbr=16',
    'FDCAN3.TxBuffersNbr=0',
    'FDCAN3.TxFifoQueueElmtsNbr=8'
)
$expectedGenerated = @(
    'hfdcan3.Init.MessageRAMOffset = 384;',
    'hfdcan3.Init.StdFiltersNbr = 8;',
    'hfdcan3.Init.RxFifo0ElmtsNbr = 16;',
    'hfdcan3.Init.TxBuffersNbr = 0;',
    'hfdcan3.Init.TxFifoQueueElmtsNbr = 8;'
)

foreach ($line in $expectedIoc) {
    if (-not $ioc.Contains($line)) {
        throw "robot.ioc is missing expected CAN3 setting: $line"
    }
}
foreach ($line in $expectedGenerated) {
    if (-not $fdcan.Contains($line)) {
        throw "Core/Src/fdcan.c is missing expected CAN3 setting: $line"
    }
}

$can1Words = 8 * 4 + 8 * 4 + 24 * 4 + 8 * 4
$can2Words = $can1Words
$can3Words = 8 + 16 * 4 + 8 * 4
$can3End = 384 + $can3Words
if ($can1Words -ne 192 -or (192 + $can2Words) -ne 384 -or $can3End -gt 2560) {
    throw 'FDCAN Message RAM allocation check failed.'
}

Write-Host "CAN3_CONFIG_TEST_OK (CAN3 words 384..$($can3End - 1), total capacity 2560)"

$remoteExe = Join-Path $outputDir 'test_remote_chassis.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    "-I$(Join-Path $firmwareRoot 'src\device')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\infra')" `
    (Join-Path $PSScriptRoot 'test_remote_chassis.c') `
    (Join-Path $firmwareRoot 'src\service\remote.c') '-o' $remoteExe
if ($LASTEXITCODE -ne 0) { throw 'Chassis remote compilation failed.' }
& $remoteExe
if ($LASTEXITCODE -ne 0) { throw 'Chassis remote test failed.' }

$removedPeripherals = 'UART7|UART8|USART10'
$generatedUart = Get-Content -Raw (Join-Path $firmwareRoot 'Core\Src\usart.c')
$main = Get-Content -Raw (Join-Path $firmwareRoot 'Core\Src\main.c')
if ($ioc -match "(?m)^($removedPeripherals)\." -or
    $ioc -match "(?m)^Mcu\.IP\d+=($removedPeripherals)$" -or
    $generatedUart -match 'huart7|huart8|huart10' -or
    $main -match 'MX_(UART7|UART8|USART10).*Init') {
    throw 'A removed device peripheral remains in CubeMX or generated startup code.'
}
$ipCount = [regex]::Matches($ioc, '(?m)^Mcu\.IP\d+=').Count
$pinCount = [regex]::Matches($ioc, '(?m)^Mcu\.Pin\d+=').Count
if ($ioc -notmatch "(?m)^Mcu\.IPNb=$ipCount\r?$" -or
    $ioc -notmatch "(?m)^Mcu\.PinsNb=$pinCount\r?$") {
    throw 'CubeMX peripheral or pin count is inconsistent.'
}
foreach ($required in @('UART5', 'USART1', 'FDCAN1', 'FDCAN2', 'FDCAN3')) {
    if ($ioc -notmatch "(?m)^Mcu\.IP\d+=$required\r?$") {
        throw "Chassis peripheral $required is missing."
    }
}
$entry = Get-Content -Raw (Join-Path $firmwareRoot 'src\app\entry.h')
if ($entry -match 'arm\.h|pc_comms|pi_comms|asr_comms|thinning') {
    throw 'The chassis application depends on an unrelated device service.'
}
Write-Host 'CHASSIS_SCOPE_TEST_OK'

& (Join-Path $PSScriptRoot 'run_chassis_tests.ps1')

$straightExe = Join-Path $outputDir 'test_straight_test.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    (Join-Path $PSScriptRoot 'test_straight_test.c') `
    (Join-Path $firmwareRoot 'src\service\straight_test.c') '-lm' '-o' $straightExe
if ($LASTEXITCODE -ne 0) { throw 'Straight test compilation failed.' }
& $straightExe
if ($LASTEXITCODE -ne 0) { throw 'Straight test failed.' }
$programExe = Join-Path $outputDir 'test_test_programs.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    (Join-Path $PSScriptRoot 'test_test_programs.c') `
    (Join-Path $firmwareRoot 'src\service\laser.c') `
    (Join-Path $firmwareRoot 'src\service\straight_test.c') '-lm' '-o' $programExe
if ($LASTEXITCODE -ne 0) { throw 'Test programs compilation failed.' }
& $programExe
if ($LASTEXITCODE -ne 0) { throw 'Test programs failed.' }

$zeroExe = Join-Path $outputDir 'test_steering_zero.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CAN_HAL' `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\platform')" `
    "-I$(Join-Path $firmwareRoot 'src\device')" `
    "-I$(Join-Path $firmwareRoot 'src\infra')" `
    (Join-Path $PSScriptRoot 'test_steering_zero.c') `
    (Join-Path $firmwareRoot 'src\service\steering_zero_test.c') '-lm' '-o' $zeroExe
if ($LASTEXITCODE -ne 0) { throw 'Steering zero test compilation failed.' }
& $zeroExe
if ($LASTEXITCODE -ne 0) { throw 'Steering zero test failed.' }

$standaloneExe = Join-Path $outputDir 'test_work_link_standalone.exe'
foreach ($standaloneMode in @(5, 6)) {
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CAN_HAL' "-DAPP_RUN_MODE=$standaloneMode" `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src\app')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\platform')" `
    (Join-Path $PSScriptRoot 'test_work_link_standalone.c') `
    (Join-Path $firmwareRoot 'src\service\work_link.c') '-lm' '-o' $standaloneExe
if ($LASTEXITCODE -ne 0) { throw 'Standalone work link compilation failed.' }
& $standaloneExe
if ($LASTEXITCODE -ne 0) { throw 'Standalone work link test failed.' }
}

$offsetExe = Join-Path $outputDir 'test_chassis_zero_offsets.exe'
foreach ($offsetMode in @(0, 6)) {
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CAN_HAL' "-DAPP_RUN_MODE=$offsetMode" `
    "-I$(Join-Path $firmwareRoot 'src\app')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\device')" `
    "-I$(Join-Path $firmwareRoot 'src\domain')" `
    "-I$(Join-Path $firmwareRoot 'src\infra')" `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src\platform')" `
    (Join-Path $PSScriptRoot 'test_chassis_zero_offsets.c') `
    (Join-Path $firmwareRoot 'src\service\chassis.c') `
    (Join-Path $firmwareRoot 'src\device\bus_motor\agv_motor.c') `
    (Join-Path $firmwareRoot 'src\domain\steer_wheel_kine.c') '-lm' '-o' $offsetExe
if ($LASTEXITCODE -ne 0) { throw 'Chassis zero offset compilation failed.' }
& $offsetExe
if ($LASTEXITCODE -ne 0) { throw 'Chassis zero offset test failed.' }
}

$yawExe = Join-Path $outputDir 'test_yaw_hold.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    "-I$(Join-Path $firmwareRoot 'src\app')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\infra')" `
    (Join-Path $PSScriptRoot 'test_yaw_hold.c') `
    (Join-Path $firmwareRoot 'src\service\chassis_yaw_hold.c') `
    (Join-Path $firmwareRoot 'src\infra\pid.c') '-lm' '-o' $yawExe
if ($LASTEXITCODE -ne 0) { throw 'Yaw hold compilation failed.' }
& $yawExe
if ($LASTEXITCODE -ne 0) { throw 'Yaw hold test failed.' }

$gearsExe = Join-Path $outputDir 'test_manual_speed_gears.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' '-DTEST_CAN_HAL' '-DAPP_RUN_MODE=5' `
    '-ffunction-sections' '-fdata-sections' '-Wl,--gc-sections' `
    "-I$(Join-Path $PSScriptRoot 'fakes')" `
    "-I$(Join-Path $firmwareRoot 'src\app')" `
    "-I$(Join-Path $firmwareRoot 'src\service')" `
    "-I$(Join-Path $firmwareRoot 'src\device')" `
    "-I$(Join-Path $firmwareRoot 'src\domain')" `
    "-I$(Join-Path $firmwareRoot 'src\infra')" `
    "-I$(Join-Path $firmwareRoot 'src\platform')" `
    (Join-Path $PSScriptRoot 'test_manual_speed_gears.c') `
    (Join-Path $firmwareRoot 'src\service\remote.c') `
    (Join-Path $firmwareRoot 'src\service\chassis_yaw_hold.c') `
    (Join-Path $firmwareRoot 'src\infra\pid.c') '-lm' '-o' $gearsExe
if ($LASTEXITCODE -ne 0) { throw 'Manual speed gear compilation failed.' }
& $gearsExe
if ($LASTEXITCODE -ne 0) { throw 'Manual speed gear test failed.' }
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'test_straight_console.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Straight console loopback test failed.' }
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'test_vofa_panel.ps1')
if ($LASTEXITCODE -ne 0) { throw 'VOFA command panel test failed.' }
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'test_vofa_setup.ps1')
if ($LASTEXITCODE -ne 0) { throw 'VOFA saved context setup test failed.' }

$diagnosticIncludes = @('-std=c11', '-Wall', '-Wextra', '-Werror', '-DTEST_CAN_HAL',
    "-I$(Join-Path $PSScriptRoot 'fakes')",
    "-I$(Join-Path $firmwareRoot 'src/app')", "-I$(Join-Path $firmwareRoot 'src/service')",
    "-I$(Join-Path $firmwareRoot 'src/platform')", "-I$(Join-Path $firmwareRoot 'src/infra')",
    "-I$(Join-Path $firmwareRoot 'src/device')", "-I$(Join-Path $firmwareRoot 'src/domain')")
$logTransportExe = Join-Path $outputDir 'test_log_transport.exe'
foreach ($logMode in @(6,5)) {
    & $compiler.Source @diagnosticIncludes "-DAPP_RUN_MODE=$logMode" `
        (Join-Path $PSScriptRoot 'test_log_transport.c') '-o' $logTransportExe
    if ($LASTEXITCODE -ne 0) { throw "Log transport compilation failed (mode=$logMode)." }
    & $logTransportExe
    if ($LASTEXITCODE -ne 0) { throw "Log transport test failed (mode=$logMode)." }
}
$diagnosticExe = Join-Path $outputDir 'test_straight_diagnostics.exe'
$rampExe = Join-Path $outputDir 'test_manual_ramp.exe'
& $compiler.Source @diagnosticIncludes '-DAPP_RUN_MODE=6' `
    (Join-Path $PSScriptRoot 'test_manual_ramp.c') `
    (Join-Path $firmwareRoot 'src/service/manual_ramp.c') '-lm' '-o' $rampExe
if ($LASTEXITCODE -ne 0) { throw 'Manual ramp compilation failed.' }
& $rampExe
if ($LASTEXITCODE -ne 0) { throw 'Manual ramp test failed.' }
$sourceExe = Join-Path $outputDir 'test_drive_source.exe'
& $compiler.Source @diagnosticIncludes '-DAPP_RUN_MODE=6' `
    (Join-Path $PSScriptRoot 'test_drive_source.c') `
    (Join-Path $firmwareRoot 'src/service/drive_source.c') '-lm' '-o' $sourceExe
if ($LASTEXITCODE -ne 0) { throw 'Drive source compilation failed.' }
& $sourceExe
if ($LASTEXITCODE -ne 0) { throw 'Drive source test failed.' }
& $compiler.Source @diagnosticIncludes '-DAPP_RUN_MODE=6' `
    (Join-Path $PSScriptRoot 'test_straight_diagnostics.c') `
    (Join-Path $firmwareRoot 'src/service/full_map_test.c') `
    (Join-Path $firmwareRoot 'src/service/field_pose.c') `
    (Join-Path $firmwareRoot 'src/service/auto_route.c') `
    (Join-Path $firmwareRoot 'src/service/auto_motion.c') `
    (Join-Path $firmwareRoot 'src/service/auto_geometry.c') `
    (Join-Path $firmwareRoot 'src/service/laser_prediction.c') `
    (Join-Path $firmwareRoot 'src/service/chassis_profile.c') `
    (Join-Path $firmwareRoot 'src/service/chassis_trace.c') `
    (Join-Path $firmwareRoot 'src/domain/steer_wheel_kine.c') `
    (Join-Path $firmwareRoot 'src/service/manual_ramp.c') `
    (Join-Path $firmwareRoot 'src/service/straight_test.c') `
    (Join-Path $firmwareRoot 'src/service/drive_source.c') `
    (Join-Path $firmwareRoot 'src/service/chassis_yaw_hold.c') `
    (Join-Path $firmwareRoot 'src/infra/pid.c') '-lm' '-o' $diagnosticExe
if ($LASTEXITCODE -ne 0) { throw 'Straight diagnostics compilation failed.' }
$env:MANGO_TRACE_EXPORT = Join-Path $outputDir 'chassis-trace-export.log'
& $diagnosticExe
if ($LASTEXITCODE -ne 0) { throw 'Straight diagnostics test failed.' }
$predictionExe = Join-Path $outputDir 'test_laser_prediction.exe'
& $compiler.Source '-std=c11' '-Wall' '-Wextra' '-Werror' `
    "-I$(Join-Path $firmwareRoot 'src/service')" `
    (Join-Path $PSScriptRoot 'test_laser_prediction.c') `
    (Join-Path $firmwareRoot 'src/service/laser_prediction.c') `
    (Join-Path $firmwareRoot 'src/service/straight_test.c') '-lm' '-o' $predictionExe
if ($LASTEXITCODE -ne 0) { throw 'Laser prediction compilation failed.' }
& $predictionExe
if ($LASTEXITCODE -ne 0) { throw 'Laser prediction regression failed.' }
& (Join-Path $PSScriptRoot 'run_chassis_reports.ps1')
$biasExe = Join-Path $outputDir 'test_imu_boot_bias.exe'
foreach ($biasMode in @(6, 5)) {
    & $compiler.Source @diagnosticIncludes "-DAPP_RUN_MODE=$biasMode" `
        (Join-Path $PSScriptRoot 'test_imu_boot_bias.c') `
        (Join-Path $firmwareRoot 'src/device/imu/imu.c') `
        (Join-Path $firmwareRoot 'src/device/imu/imu_attitude.c') '-lm' '-o' $biasExe
    if ($LASTEXITCODE -ne 0) { throw "IMU boot policy compilation failed (mode=$biasMode)." }
    & $biasExe
    if ($LASTEXITCODE -ne 0) { throw "IMU boot policy test failed (mode=$biasMode)." }
}

# Pure-C autonomous foundation and offline reports. No serial or probe access.
& powershell.exe -NoProfile -ExecutionPolicy Bypass -File (Join-Path $PSScriptRoot 'run_autonomy_tests.ps1')
if ($LASTEXITCODE -ne 0) { throw 'Autonomous foundation regression failed.' }
$repoRoot = Split-Path $firmwareRoot -Parent
Push-Location $repoRoot
try {
    & py -3 -B -m unittest discover -s tools -p 'test_*.py' -v
    if ($LASTEXITCODE -ne 0) { throw 'Offline route/report Python regression failed.' }
    & py -3 -B tools/autonomy_report.py mission firmware/build/autonomy-prep/traces --output firmware/build/autonomy-prep/reports
    if ($LASTEXITCODE -ne 0) { throw 'Autonomous trace report failed.' }
    & py -3 -B tools/autonomy_report.py ramp tools/fixtures/ramp-synthetic.csv --output firmware/build/autonomy-prep/reports/ramp-synthetic.json
    if ($LASTEXITCODE -ne 0) { throw 'Synthetic ramp report failed.' }
} finally { Pop-Location }
