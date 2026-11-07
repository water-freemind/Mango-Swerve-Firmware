#requires -PSEdition Desktop
$ErrorActionPreference = 'Stop'
$firmware = Split-Path $PSScriptRoot -Parent
$fixture = Join-Path $firmware ('build/host-tests/vofa-setup-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff'))
$context = Join-Path $fixture 'context'
New-Item -ItemType Directory -Path $context -Force | Out-Null
[IO.File]::WriteAllText((Join-Path $fixture 'vofa+.exe'),'test fixture, not executed')
$before = [ordered]@{type='config';vnumber=100;ctx=[ordered]@{
    wave_view=@{ctx=@{left_panel=@{ctx=@{pal=@{ctx=@{
        protocol_combo='RawData';link_type_combo=0;serial=@{port='COM19';baud='921600'};
        udp=@{remote_ip='';remote_port='';local_port=''};keep='unchanged'
    }}}}}};unrelated=@{font='keep';decimal=6;array=@(1,2,3)}
}}
$configPath = Join-Path $context 'vofa+.config.json'
[IO.File]::WriteAllText($configPath,($before | ConvertTo-Json -Depth 30),[Text.UTF8Encoding]::new($false))
[IO.File]::WriteAllText((Join-Path $context 'vofa+.cmds.json'),'old command fixture')
$beforeText = Get-Content -Raw $configPath
& (Join-Path $firmware 'vofa/install-vofa-panel.ps1') -VofaDir $fixture -ContextDir $context -ConfigureContext
$after = Get-Content -Raw $configPath | ConvertFrom-Json
$pal = $after.ctx.wave_view.ctx.left_panel.ctx.pal.ctx
if ($pal.link_type_combo -ne 1 -or $pal.protocol_combo -ne 'FireWater' -or
    $pal.udp.remote_ip -ne '127.0.0.1' -or $pal.udp.remote_port -ne '1348' -or $pal.udp.local_port -ne '1347') {
    throw 'VOFA connection settings were not configured.'
}
$old = $beforeText | ConvertFrom-Json
$oldPal = $old.ctx.wave_view.ctx.left_panel.ctx.pal.ctx
$pal.link_type_combo = $oldPal.link_type_combo
$pal.protocol_combo = $oldPal.protocol_combo
$pal.udp = $oldPal.udp
if (($after | ConvertTo-Json -Depth 30 -Compress) -cne ($old | ConvertTo-Json -Depth 30 -Compress)) {
    throw 'An unrelated VOFA setting changed.'
}
$commands = Join-Path $firmware 'vofa/vofa+.cmds.json'
foreach ($copy in @((Join-Path $fixture 'vofa+.cmds.json'),(Join-Path $context 'vofa+.cmds.json'))) {
    if ((Get-FileHash $commands).Hash -ne (Get-FileHash $copy).Hash) { throw 'Command installation mismatch' }
}
$backup = Get-ChildItem $context -Directory -Filter 'mango-backup-*' | Select-Object -Last 1
if ((Get-Content -Raw (Join-Path $backup.FullName 'vofa+.config.json')) -cne $beforeText) { throw 'Original settings backup changed' }
foreach ($mode in @('Static','1m-OPEN-AREA')) {
    if ((Get-Content -Raw (Join-Path $fixture ('Mango-VOFA-' + $mode + '.cmd'))) -notmatch 'start-vofa-panel.ps1') {
        throw 'Installed shortcut does not start VOFA and its bridge together.'
    }
}
'VOFA_SETUP_TEST_OK (saved UDP context, auto-loaded command file, backups, unrelated settings preserved; VOFA not launched)'
