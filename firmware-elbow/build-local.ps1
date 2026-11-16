param(
    [string]$Toolchain = '',
    [ValidateSet('FullMapWork', 'VisionTest', 'LegacyWork', 'ManualEncoder')][string]$Mode = 'FullMapWork'
)

$ErrorActionPreference = 'Stop'
if (!$Toolchain) {
    $candidates = @($env:ARM_GCC_ROOT,
        'E:\EmbeddedTools\xpack-arm-none-eabi-gcc-15.2.1-1.1',
        (Join-Path $env:USERPROFILE '.eide\tools\gcc_arm'))
    $pathCompiler = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($pathCompiler) { $candidates += Split-Path (Split-Path $pathCompiler.Source -Parent) -Parent }
    foreach ($candidate in $candidates) {
        if ($candidate -and (Test-Path -LiteralPath (Join-Path $candidate 'bin/arm-none-eabi-gcc.exe'))) {
            $Toolchain = $candidate
            break
        }
    }
    if (!$Toolchain) { throw 'ARM GCC not found. Set ARM_GCC_ROOT or pass -Toolchain <installation root>.' }
}
$compiler = Join-Path $Toolchain 'bin/arm-none-eabi-gcc.exe'
$objcopy = Join-Path $Toolchain 'bin/arm-none-eabi-objcopy.exe'
$size = Join-Path $Toolchain 'bin/arm-none-eabi-size.exe'
foreach ($tool in @($compiler, $objcopy, $size)) {
    if (!(Test-Path -LiteralPath $tool)) { throw "ARM tool missing at $tool" }
}
Write-Host "ARM_TOOLCHAIN $Toolchain"
$runMode = @{ FullMapWork = 2; VisionTest = 1; LegacyWork = 0; ManualEncoder = 3 }[$Mode]
$output = "build/$Mode"

Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force "$output/obj" | Out-Null
    # A failed build must not leave an older flashable image under this mode's name.
    foreach ($extension in @('elf', 'hex', 'bin')) {
        $oldArtifact = "$output/elbow.$extension"
        if (Test-Path -LiteralPath $oldArtifact) { Remove-Item -LiteralPath $oldArtifact }
    }
    # The existing OLED bitmap tables use valid flat multidimensional initializers.
    $flags = @('-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard',
        '-g3', '-Og', '-ffunction-sections', '-fdata-sections', '-Wall', '-Wno-missing-braces',
        '-DUSE_HAL_DRIVER', '-DSTM32F407xx', "-DELBOW_TEST_MODE=$runMode")
    $includes = @('Core/Inc', 'Drivers/STM32F4xx_HAL_Driver/Inc',
        'Drivers/STM32F4xx_HAL_Driver/Inc/Legacy', 'Drivers/CMSIS/Device/ST/STM32F4xx/Include',
        'Drivers/CMSIS/Include', 'BSP', 'BSP/OLED')
    $flags += $includes | ForEach-Object { "-I$_" }
    $halSources = @(
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c',
        'Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_uart.c'
    )
    $sources = @(Get-ChildItem Core, BSP -Recurse -File |
        Where-Object { $_.Extension -in '.c', '.s', '.S' }) +
        @($halSources | ForEach-Object { Get-Item -LiteralPath $_ })
    $sources = $sources | Sort-Object FullName
    $objects = @()
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
    & $compiler @flags @objects -T STM32F407VETX_FLASH.ld --specs=nano.specs --specs=nosys.specs `
        '-Wl,--gc-sections' '-Wl,-z,noexecstack' '-Wl,-u,_printf_float' "-Wl,-Map=$output/elbow.map" '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o "$output/elbow.elf"
    if ($LASTEXITCODE -ne 0) { throw 'Link failed' }
    & $objcopy -O ihex "$output/elbow.elf" "$output/elbow.hex"
    if ($LASTEXITCODE -ne 0) { throw 'HEX export failed' }
    & $objcopy -O binary "$output/elbow.elf" "$output/elbow.bin"
    if ($LASTEXITCODE -ne 0) { throw 'BIN export failed' }
    & $size "$output/elbow.elf"
    if ($LASTEXITCODE -ne 0) { throw 'Size check failed' }
    Write-Host "BUILD_OK mode=$Mode sources=$($sources.Count) output=$output/elbow.hex"
} finally {
    Pop-Location
}
