#requires -Version 5.1
param(
    [string]$VofaDir = 'D:\Vofa +\x64',
    [switch]$ConfigureContext,
    [string]$ContextDir = (Join-Path $env:LOCALAPPDATA 'vofa+\100\context')
)
$ErrorActionPreference = 'Stop'
if (!(Test-Path -LiteralPath (Join-Path $VofaDir 'vofa+.exe'))) { throw 'VOFA executable missing in VofaDir.' }
if ($ConfigureContext) {
    $executable = [IO.Path]::GetFullPath((Join-Path $VofaDir 'vofa+.exe'))
    $running = Get-CimInstance Win32_Process -Filter "Name = 'vofa+.exe'" | Where-Object { $_.ExecutablePath -eq $executable }
    if ($running) { throw 'Exit VOFA completely before applying its saved connection settings.' }
    $configPath = Join-Path $ContextDir 'vofa+.config.json'
    $config = Get-Content -Raw -Encoding UTF8 -LiteralPath $configPath | ConvertFrom-Json
    if ($config.type -ne 'config' -or $config.vnumber -ne 100) { throw 'Unsupported VOFA settings format.' }
    $pal = $config.ctx.wave_view.ctx.left_panel.ctx.pal.ctx
    if ($null -eq $pal.udp -or $null -eq $pal.link_type_combo) { throw 'VOFA connection settings missing.' }
    $contextBackup = Join-Path $ContextDir ('mango-backup-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
    New-Item -ItemType Directory -Path $contextBackup | Out-Null
    foreach ($name in @('vofa+.config.json','vofa+.cmds.json')) {
        $saved = Join-Path $ContextDir $name
        if (Test-Path -LiteralPath $saved) { Copy-Item -LiteralPath $saved -Destination (Join-Path $contextBackup $name) }
    }
    $pal.protocol_combo = 'FireWater'
    $pal.link_type_combo = 1
    $pal.udp.remote_ip = '127.0.0.1'
    $pal.udp.remote_port = '1348'
    $pal.udp.local_port = '1347'
    [IO.File]::WriteAllText($configPath,($config | ConvertTo-Json -Depth 100),[Text.UTF8Encoding]::new($false))
    Copy-Item -LiteralPath (Join-Path $PSScriptRoot 'vofa+.cmds.json') -Destination (Join-Path $ContextDir 'vofa+.cmds.json') -Force
    Write-Host "VOFA_CONTEXT_CONFIGURED backup=$contextBackup"
}
$source = Join-Path $PSScriptRoot 'vofa+.cmds.json'
$null = Get-Content -Raw -LiteralPath $source -Encoding UTF8 | ConvertFrom-Json
$destination = Join-Path $VofaDir 'vofa+.cmds.json'
if (Test-Path -LiteralPath $destination) {
    $backup = Join-Path $VofaDir ('vofa+.cmds.before-mango-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.json')
    Copy-Item -LiteralPath $destination -Destination $backup
    Write-Host "BACKUP $backup"
}
Copy-Item -LiteralPath $source -Destination $destination -Force
$starter = Join-Path $PSScriptRoot 'start-vofa-panel.ps1'
$legacy = Join-Path $VofaDir 'Mango-VOFA-2m-OPEN-AREA.cmd'
if (Test-Path -LiteralPath $legacy) {
    Move-Item -LiteralPath $legacy -Destination ($legacy + '.retired-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.bak')
}
foreach ($mode in @('Static', '1m-OPEN-AREA')) {
    $startMode = 'Static'
    if ($mode -ne 'Static') { $startMode = 'OneMeter' }
    $launcher = '@echo off' + "`r`n" +
        'title Mango VOFA ' + $mode + "`r`n" +
        'powershell.exe -NoProfile -ExecutionPolicy Bypass -File "' + $starter +
        '" -VofaDir "' + $VofaDir + '" -Mode ' + $startMode + "`r`n" +
        'if errorlevel 1 pause' + "`r`n"
    [IO.File]::WriteAllText((Join-Path $VofaDir ('Mango-VOFA-' + $mode + '.cmd')), $launcher, [Text.Encoding]::ASCII)
}
if ((Get-FileHash $source).Hash -ne (Get-FileHash $destination).Hash) { throw 'Installed JSON differs from source.' }
Write-Host "VOFA_PANEL_INSTALLED $destination"
Write-Host 'Import this command file in VOFA. Use UDP local=1347, remote=127.0.0.1 port=1348, FireWater.'
Write-Host 'Disconnect VOFA COM19 before opening either launcher. Static blocks ARM/GO; neither launcher auto-starts.'
