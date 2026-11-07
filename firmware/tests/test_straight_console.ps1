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
    'STRAIGHT_CONSOLE_UDP_TEST_OK (loopback only; serial not opened)'
} finally { $sender.Dispose(); $receiver.Dispose() }
