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
