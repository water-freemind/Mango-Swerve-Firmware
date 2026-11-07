#requires -Version 5.1
param([string]$VofaDir = 'D:\Vofa +\x64')
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath (Join-Path $VofaDir 'vofa+.exe'))) { throw 'VOFA executable missing in VofaDir.' }
$source = Join-Path $PSScriptRoot 'vofa+.cmds.json'
$null = Get-Content -Raw -LiteralPath $source -Encoding UTF8 | ConvertFrom-Json
$destination = Join-Path $VofaDir 'vofa+.cmds.json'
if (Test-Path -LiteralPath $destination) {
    $backup = Join-Path $VofaDir ('vofa+.cmds.before-mango-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.json')
    Copy-Item -LiteralPath $destination -Destination $backup
    Write-Host "BACKUP $backup"
}
Copy-Item -LiteralPath $source -Destination $destination -Force
$console = [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../straight-console.ps1'))
$legacy = Join-Path $VofaDir 'Mango-VOFA-2m-OPEN-AREA.cmd'
if (Test-Path -LiteralPath $legacy) {
    Move-Item -LiteralPath $legacy -Destination ($legacy + '.retired-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.bak')
}
foreach ($mode in @('Static', '1m-OPEN-AREA')) {
    $motionFlag = ''
    if ($mode -ne 'Static') { $motionFlag = ' -AllowMotion' }
    $launcher = '@echo off' + "`r`n" +
        'title Mango VOFA ' + $mode + "`r`n" +
        'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "' + $console +
        '" -Port COM19 -WavePort 1347 -CommandPort 1348 -UiOnly' + $motionFlag + "`r`n" +
        'pause' + "`r`n"
    [IO.File]::WriteAllText((Join-Path $VofaDir ('Mango-VOFA-' + $mode + '.cmd')), $launcher, [Text.Encoding]::ASCII)
}
if ((Get-FileHash $source).Hash -ne (Get-FileHash $destination).Hash) { throw 'Installed JSON differs from source.' }
Write-Host "VOFA_PANEL_INSTALLED $destination"
Write-Host 'Import this command file in VOFA. Use UDP local=1347, remote=127.0.0.1 port=1348, FireWater.'
Write-Host 'Disconnect VOFA COM19 before opening either launcher. Static blocks ARM/GO; neither launcher auto-starts.'
