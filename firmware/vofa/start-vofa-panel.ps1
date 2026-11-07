#requires -PSEdition Desktop
param(
    [string]$VofaDir = 'D:\Vofa +\x64',
    [ValidateSet('Static','OneMeter')][string]$Mode = 'Static'
)
$ErrorActionPreference = 'Stop'
& (Join-Path $PSScriptRoot 'install-vofa-panel.ps1') -VofaDir $VofaDir -ConfigureContext
$occupied = Get-NetUDPEndpoint -LocalPort 1348 -ErrorAction SilentlyContinue
if ($occupied) { throw 'UDP 1348 already in use. Close the existing bridge before starting another session.' }
$logDir = Join-Path $PSScriptRoot '../build/logs'
New-Item -ItemType Directory -Path $logDir -Force | Out-Null
$stamp = Get-Date -Format 'yyyyMMdd-HHmmss-fff'
$arguments = @('-NoProfile','-ExecutionPolicy','Bypass','-File',
    ('"' + [IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../straight-console.ps1')) + '"'),
    '-Port','COM19','-WavePort','1347','-CommandPort','1348','-UiOnly','-ExitWithVofa')
if ($Mode -eq 'OneMeter') { $arguments += '-AllowMotion' }
# User-launched VOFA is interactive; the serial bridge stays hidden and never auto-starts a test.
$null = Start-Process -FilePath (Join-Path $VofaDir 'vofa+.exe') -WorkingDirectory $VofaDir -PassThru
$relay = Start-Process powershell.exe -ArgumentList $arguments -WindowStyle Hidden -PassThru `
    -RedirectStandardOutput (Join-Path $logDir ('vofa-bridge-' + $stamp + '.stdout.log')) `
    -RedirectStandardError (Join-Path $logDir ('vofa-bridge-' + $stamp + '.stderr.log'))
Write-Host "VOFA_BRIDGE_STARTED pid=$($relay.Id) mode=$Mode auto_start=0"
Write-Host 'In VOFA open the UDP connection, then click the named commands. Exit VOFA to close the bridge.'
