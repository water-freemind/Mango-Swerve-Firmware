param(
    [string]$Toolchain = '',
    [ValidateSet('Diagnostics', 'Manual', 'ManualStandalone', 'Steering', 'Can2Query', 'ZeroTest')][string]$Mode = 'Diagnostics'
)

$ErrorActionPreference = 'Stop'
if (!$Toolchain) {
    $candidates = @($env:ARM_GCC_ROOT,
        'E:\EmbeddedTools\xpack-arm-none-eabi-gcc-15.2.1-1.1',
        (Join-Path $env:USERPROFILE '.eide\tools\gcc_arm'))
    $pathCompiler = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($pathCompiler) { $candidates += Split-Path (Split-Path $pathCompiler.Source -Parent) -Parent }
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath (Join-Path $candidate 'bin\arm-none-eabi-gcc.exe'))) {
            $Toolchain = $candidate
            break
        }
    }
    if (!$Toolchain) { throw 'ARM GCC not found. Set ARM_GCC_ROOT or pass -Toolchain <installation root>.' }
}
$compiler = Join-Path $Toolchain 'bin\arm-none-eabi-gcc.exe'
$objcopy = Join-Path $Toolchain 'bin\arm-none-eabi-objcopy.exe'
$size = Join-Path $Toolchain 'bin\arm-none-eabi-size.exe'
foreach ($tool in @($compiler, $objcopy, $size)) {
    if (!(Test-Path -LiteralPath $tool)) { throw "ARM tool missing at $tool" }
}
Write-Host "ARM_TOOLCHAIN $Toolchain"
$output = 'build/Debug'
if ($Mode -eq 'Manual') { $output = 'build/Manual' }
if ($Mode -eq 'ManualStandalone') { $output = 'build/ManualStandalone' }
if ($Mode -eq 'Steering') { $output = 'build/Steering' }
if ($Mode -eq 'Can2Query') { $output = 'build/Can2Query' }
if ($Mode -eq 'ZeroTest') { $output = 'build/ZeroTest' }

Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force "$output/obj" | Out-Null
    # Match the Debug target in .eide/eide.yml and keep diagnostic mode explicit.
    $runMode = 0
    if ($Mode -eq 'Manual') { $runMode = 1 }
    if ($Mode -eq 'ManualStandalone') { $runMode = 5 }
    if ($Mode -eq 'Steering') { $runMode = 2 }
    if ($Mode -eq 'Can2Query') { $runMode = 3 }
    if ($Mode -eq 'ZeroTest') { $runMode = 4 }
    $flags = @('-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-sp-d16', '-mfloat-abi=hard',
        '-g3', '-Og', '-ffunction-sections', '-fdata-sections', '-Wall',
        '-DDEBUG', '-DUSE_PWR_LDO_SUPPLY', '-DUSE_HAL_DRIVER', '-DSTM32H723xx', "-DAPP_RUN_MODE=$runMode")
    $includes = @('Core/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc',
        'Drivers/STM32H7xx_HAL_Driver/Inc/Legacy', 'Drivers/CMSIS/Device/ST/STM32H7xx/Include',
        'Drivers/CMSIS/Include', 'src/app', 'src/device', 'src/domain', 'src/infra', 'src/platform', 'src/service')
    $flags += $includes | ForEach-Object { "-I$_" }
    $sources = Get-ChildItem Core, Drivers, src -Recurse -File | Where-Object { $_.Extension -in '.c', '.s', '.S' } | Sort-Object FullName
    $objects = @()
    # Invalidate the previous deliverable before rebuilding so a failed build cannot be flashed.
    foreach ($extension in @('elf', 'hex', 'bin')) {
        $oldArtifact = "$output/robot.$extension"
        if (Test-Path -LiteralPath $oldArtifact) { Remove-Item -LiteralPath $oldArtifact }
    }
    foreach ($source in $sources) {
        $relative = $source.FullName.Substring($PSScriptRoot.Length + 1).Replace('\', '/')
        $object = "$output/obj/" + $relative.Replace('/', '_') + '.o'
        Write-Host "CC $relative"
        $languageFlags = @('-std=c11')
        if ($source.Extension -ne '.c') { $languageFlags = @('-x', 'assembler-with-cpp') }
        & $compiler @flags @languageFlags -c $relative -o $object
        if ($LASTEXITCODE -ne 0) { throw "Compilation failed for $relative" }
        $objects += $object
    }
    & $compiler @flags @objects -T STM32H723VGTX_FLASH_USER.ld --specs=nano.specs --specs=nosys.specs '-Wl,--gc-sections' "-Wl,-Map=$output/robot.map" '-Wl,-u,_printf_float' '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o "$output/robot.elf"
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & $objcopy -O ihex "$output/robot.elf" "$output/robot.hex"
    if ($LASTEXITCODE -ne 0) { throw 'HEX export failed' }
    & $objcopy -O binary "$output/robot.elf" "$output/robot.bin"
    if ($LASTEXITCODE -ne 0) { throw 'BIN export failed' }
    & $size "$output/robot.elf"
    if ($LASTEXITCODE -ne 0) { throw 'Size check failed' }
    Write-Host "BUILD_OK mode=$Mode sources=$($sources.Count) output=$output/robot.hex"
} finally {
    Pop-Location
}
