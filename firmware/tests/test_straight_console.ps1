#requires -PSEdition Desktop
$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '../straight-console.ps1')
$code = [regex]::Match($source, "(?s)-TypeDefinition @'\r?\n(.*?)\r?\n'@")
if (!$code.Success) { throw 'Console C# source missing' }
Add-Type -TypeDefinition $code.Groups[1].Value
$receiver = New-Object System.Net.Sockets.UdpClient([System.Net.IPEndPoint]::new([System.Net.IPAddress]::Loopback, 0))
$sender = New-Object System.Net.Sockets.UdpClient
try {
    $receiver.Client.ReceiveTimeout = 1000
    $port = $receiver.Client.LocalEndPoint.Port
    foreach ($line in @('TEST_STATUS yaw=1', 'FW: 1,2,3', 'FW: NaN,2,3,4', 'FW: 1,2,3,4,5')) {
        if ([MangoStraightConsole]::ForwardWave($sender, $port, $line)) { throw "Invalid frame accepted: $line" }
    }
    if ([MangoStraightConsole]::ForwardWave($sender, 0, 'FW: 1,2,3,4')) { throw 'Disabled forwarding sent data' }
    if (![MangoStraightConsole]::ForwardWave($sender, $port, "FW: -7.7, -7.6, 0.1, 0.003`r")) { throw 'Valid frame rejected' }
    $peer = [System.Net.IPEndPoint]::new([System.Net.IPAddress]::Any, 0)
    $actual = [System.Text.Encoding]::ASCII.GetString($receiver.Receive([ref]$peer))
    if ($actual -cne "FW: -7.7, -7.6, 0.1, 0.003`n") { throw 'FireWater payload changed' }
    $status = '[INFO] TEST_STATUS phase=0 healthy=0 stopped=1 cmd=0.000000 result=IDLE'
    if (![MangoStraightConsole]::ForwardStatus($sender, $port, ([char]27 + '[94m' + $status + [char]27 + "[0m`r"))) { throw 'Status log rejected' }
    $actual = [System.Text.Encoding]::ASCII.GetString($receiver.Receive([ref]$peer))
    if ($actual -cne ($status + "`n")) { throw 'Status log content changed' }
    foreach ($tag in @('HEALTH','REMOTE','IMU','IMU_RAW','IMU_BIAS','MOTOR','CAN','LASER','TIMING','DRIFT')) {
        $detail = '[INFO] TEST_' + $tag + ' t_ms=1234 value=1'
        if (![MangoStraightConsole]::ForwardStatus($sender, $port, $detail)) { throw "Detail rejected $tag" }
        $actual = [System.Text.Encoding]::ASCII.GetString($receiver.Receive([ref]$peer))
        if ($actual -cne ($detail + "`n")) { throw "Detail payload changed $tag" }
    }
    foreach ($line in @('FW: 1,2,3,4', 'unrelated: 1,2,3,4', '[INFO] TEST_STATUS note:invalid')) {
        if ([MangoStraightConsole]::ForwardStatus($sender, $port, $line)) { throw 'Unexpected log forwarded as status' }
    }
    if ([MangoStraightConsole]::ForwardStatus($sender, 0, $status)) { throw 'Disabled status forwarding sent data' }
    'STRAIGHT_CONSOLE_UDP_TEST_OK (loopback only; serial not opened)'
} finally { $sender.Dispose(); $receiver.Dispose() }
