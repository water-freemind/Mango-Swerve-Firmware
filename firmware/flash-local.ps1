param(
    [string]$OpenOcdRoot = '',
    [string]$Toolchain = '',
    [ValidateSet('auto', 'hid', 'usb_bulk')][string]$DapBackend = 'hid',
    [switch]$ProbeOnly,
    [ValidateSet('Diagnostics', 'Manual', 'ManualStandalone', 'Steering', 'Can2Query', 'ZeroTest', 'StraightTest', 'FullMapWork')][string]$Mode = 'Diagnostics'
)

$ErrorActionPreference = 'Stop'
if (!$OpenOcdRoot) {
    $candidates = @($env:OPENOCD_ROOT, 'E:\EmbeddedTools\xpack-openocd-0.12.0-7',
        (Join-Path $env:USERPROFILE 'EmbeddedTools\xpack-openocd-0.12.0-7'))
    $eideTools = Join-Path $env:USERPROFILE '.eide\tools'
    if (Test-Path -LiteralPath $eideTools) {
        $candidates += Get-ChildItem -LiteralPath $eideTools -Directory -Filter 'openocd*' |
            Sort-Object Name | ForEach-Object { $_.FullName }
    }
    $pathOpenOcd = Get-Command openocd.exe -ErrorAction SilentlyContinue
    if ($pathOpenOcd) { $candidates += Split-Path (Split-Path $pathOpenOcd.Source -Parent) -Parent }
    foreach ($candidate in $candidates) {
        if ($candidate -and [System.IO.Directory]::Exists($candidate) -and
            (Test-Path -LiteralPath (Join-Path $candidate 'bin\openocd.exe'))) {
            $OpenOcdRoot = $candidate
            break
        }
    }
    if (!$OpenOcdRoot) { throw 'OpenOCD not found. Set OPENOCD_ROOT or pass -OpenOcdRoot <installation root>.' }
}
$openocd = Join-Path $OpenOcdRoot 'bin\openocd.exe'
if (!(Test-Path -LiteralPath $openocd)) { throw "OpenOCD missing at $openocd" }
$scripts = $null
foreach ($relative in @('openocd\scripts', 'share\openocd\scripts', 'scripts')) {
    $candidate = Join-Path $OpenOcdRoot $relative
    if ((Test-Path -LiteralPath (Join-Path $candidate 'interface\cmsis-dap.cfg')) -and
        (Test-Path -LiteralPath (Join-Path $candidate 'target\stm32h7x.cfg'))) {
        $scripts = $candidate
        break
    }
}
if (!$scripts) { throw "OpenOCD CMSIS-DAP/STM32H7 scripts missing under $OpenOcdRoot" }
Write-Host "OPENOCD_ROOT $OpenOcdRoot"
Push-Location $PSScriptRoot
try {
    $connection = @('-s', $scripts, '-f', 'interface/cmsis-dap.cfg',
        '-c', "cmsis-dap backend $DapBackend",
        '-c', 'cmsis-dap quirk enable',
        '-c', 'transport select swd', '-f', 'target/stm32h7x.cfg',
        '-c', 'adapter speed 400',
        '-c', 'stm32h7x.cpu0 configure -event reset-init {adapter speed 400}')
    if ($ProbeOnly) {
        & $openocd @connection -c 'init; targets stm32h7x.cpu0; targets; shutdown'
        if ($LASTEXITCODE -ne 0) { throw 'SWD probe failed' }
        return
    }
    & (Join-Path $PSScriptRoot 'build-local.ps1') -Toolchain $Toolchain -Mode $Mode
    New-Item -ItemType Directory -Force 'build/backup' | Out-Null
    $backup = 'build/backup/before-flash-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.bin'
    # Read and verify the original 1 MB flash before programming the selected image.
    & $openocd @connection -c "init; targets stm32h7x.cpu0; reset halt; dump_image $backup 0x08000000 0x100000; verify_image $backup 0x08000000 bin; resume; shutdown"
    if ($LASTEXITCODE -ne 0) { throw 'Backup failed. Check the SWD connection and retry.' }
    if ((Get-Item -LiteralPath $backup).Length -ne 0x100000) { throw 'Incomplete backup' }
    $image = 'build/Debug/robot.hex'
    if ($Mode -eq 'StraightTest') { $image = 'build/StraightTest/robot.hex' }
    if ($Mode -eq 'FullMapWork') { $image = 'build/FullMapWork/robot.hex' }
    if ($Mode -eq 'Manual') { $image = 'build/Manual/robot.hex' }
    if ($Mode -eq 'ManualStandalone') { $image = 'build/ManualStandalone/robot.hex' }
    if ($Mode -eq 'Steering') { $image = 'build/Steering/robot.hex' }
    if ($Mode -eq 'Can2Query') { $image = 'build/Can2Query/robot.hex' }
    if ($Mode -eq 'ZeroTest') { $image = 'build/ZeroTest/robot.hex' }
    & $openocd @connection -c "program $image verify reset exit"
    if ($LASTEXITCODE -ne 0) { throw 'Flash or verification failed' }
    Write-Host "FLASH_OK mode=$Mode backup=$backup"
} finally {
    Pop-Location
}
