param(
    [string]$OpenOcdRoot = 'E:\EmbeddedTools\xpack-openocd-0.12.0-7',
    [string]$Toolchain = 'E:\EmbeddedTools\xpack-arm-none-eabi-gcc-15.2.1-1.1',
    [switch]$ProbeOnly,
    [ValidateSet('Diagnostics', 'Steering', 'ZeroTest')][string]$Mode = 'Diagnostics'
)

$ErrorActionPreference = 'Stop'
$openocd = Join-Path $OpenOcdRoot 'bin\openocd.exe'
$scripts = Join-Path $OpenOcdRoot 'openocd\scripts'
if (!(Test-Path -LiteralPath $openocd)) { throw "OpenOCD missing at $openocd" }
Push-Location $PSScriptRoot
try {
    $connection = @('-s', $scripts, '-f', 'interface/cmsis-dap.cfg',
        '-c', 'transport select swd', '-c', 'adapter speed 1000', '-f', 'target/stm32h7x.cfg')
    if ($ProbeOnly) {
        & $openocd @connection -c 'init; targets; shutdown'
        if ($LASTEXITCODE -ne 0) { throw 'SWD probe failed' }
        return
    }
    & (Join-Path $PSScriptRoot 'build-local.ps1') -Toolchain $Toolchain -Mode $Mode
    New-Item -ItemType Directory -Force 'build/backup' | Out-Null
    $backup = 'build/backup/before-flash-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.bin'
    # Read and verify the original 1 MB flash before programming the diagnostic image.
    & $openocd @connection -c "init; halt; dump_image $backup 0x08000000 0x100000; verify_image $backup 0x08000000 bin; resume; shutdown"
    if ($LASTEXITCODE -ne 0) { throw 'Backup failed. Check the SWD connection and retry.' }
    if ((Get-Item -LiteralPath $backup).Length -ne 0x100000) { throw 'Incomplete backup' }
    $image = 'build/Debug/robot.hex'
    if ($Mode -eq 'Steering') { $image = 'build/Steering/robot.hex' }
    if ($Mode -eq 'ZeroTest') { $image = 'build/ZeroTest/robot.hex' }
    & $openocd @connection -c "program $image verify reset exit"
    if ($LASTEXITCODE -ne 0) { throw 'Flash or verification failed' }
    Write-Host "FLASH_OK mode=$Mode backup=$backup"
} finally {
    Pop-Location
}
