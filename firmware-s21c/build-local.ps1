param([string]$Toolchain = '')
$ErrorActionPreference='Stop'
if (!$Toolchain) { $Toolchain=$env:ARM_GCC_ROOT }
if (!$Toolchain) { $Toolchain=Join-Path $env:USERPROFILE '.eide/tools/gcc_arm' }
$cc=Join-Path $Toolchain 'bin/arm-none-eabi-gcc.exe'
$objcopy=Join-Path $Toolchain 'bin/arm-none-eabi-objcopy.exe'
$size=Join-Path $Toolchain 'bin/arm-none-eabi-size.exe'
if (!(Test-Path -LiteralPath $cc)) { throw 'Set ARM_GCC_ROOT or pass -Toolchain.' }
Push-Location $PSScriptRoot
try {
    New-Item -ItemType Directory -Force build/obj | Out-Null
    foreach($ext in @('elf','hex','bin')) {
        $old="build/s21c.$ext"
        if(Test-Path -LiteralPath $old) {Remove-Item -LiteralPath $old}
    }
    $flags=@('-mcpu=cortex-m3','-mthumb','-std=gnu99','-Og','-g3','-ffunction-sections','-fdata-sections',
        '-Wall','-DSTM32F10X_HD','-DUSE_STDPERIPH_DRIVER')
    foreach($inc in @('USER','CORE','STM32F10x_FWLib/inc','SYSTEM/delay','SYSTEM/sys','SYSTEM/usart','HARDWARE','src')) {
        $flags+="-I$inc"
    }
    [xml]$project=Get-Content -Raw USER/UART1.uvprojx
    $sources=@($project.SelectNodes('//FilePath') | ForEach-Object {
        if ($_.InnerText -match '\.c$') { [IO.Path]::GetFullPath((Join-Path "$PSScriptRoot/USER" $_.InnerText)) }
    })
    $sources+=Join-Path $PSScriptRoot 'gcc/startup_stm32f103rc.S'
    $objects=@()
    foreach($source in $sources) {
        $rel=$source.Substring($PSScriptRoot.Length+1).Replace('\','/')
        $obj='build/obj/'+$rel.Replace('/','_')+'.o'
        Write-Host "CC $rel"
        $unitFlags=@()
        if($rel.StartsWith('src/')) {$unitFlags=@('-Wextra','-Werror')}
        & $cc @flags @unitFlags -c $source -o $obj
        if($LASTEXITCODE -ne 0) {throw "Compile failed: $rel"}
        $objects+=$obj
    }
    & $cc -mcpu=cortex-m3 -mthumb @objects -T gcc/stm32f103rc.ld --specs=nano.specs --specs=nosys.specs `
        '-Wl,--gc-sections' '-Wl,-Map=build/s21c.map' '-Wl,--start-group' -lc -lm '-Wl,--end-group' -o build/s21c.elf
    if($LASTEXITCODE -ne 0) {throw 'Link failed'}
    & $objcopy -O ihex build/s21c.elf build/s21c.hex
    if($LASTEXITCODE -ne 0) {throw 'HEX failed'}
    & $objcopy -O binary build/s21c.elf build/s21c.bin
    if($LASTEXITCODE -ne 0) {throw 'BIN failed'}
    & $size build/s21c.elf
    if($LASTEXITCODE -ne 0) {throw 'Size failed'}
    Write-Host "S21C_BUILD_OK sources=$($sources.Count) image=build/s21c.hex (not flashed)"
} finally {Pop-Location}
