#requires -PSEdition Desktop
$ErrorActionPreference = 'Stop'
$source = Get-Content -Raw (Join-Path $PSScriptRoot '../straight-console.ps1')
$code = [regex]::Match($source, "(?s)-TypeDefinition @'\r?\n(.*?)\r?\n'@")
Add-Type -TypeDefinition $code.Groups[1].Value
$tx = [Collections.Generic.List[string]]::new()
$messages = [Collections.Generic.List[string]]::new()
$send = [Action[string]]{ param($s) $tx.Add($s) }
$report = [Action[string]]{ param($s) $messages.Add($s) }
function Assert($ok, $why) { if (!$ok) { throw $why } }
$idle = '[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1 cmd=0.000'
$static = [VofaTestSession]::new($false, $send, $report)
$static.Observe($idle, 0)
$static.Observe('FW: 179,0,0,0', 0)
Assert (!$static.Command("UI ARM`n", 1)) 'Static mode allowed ARM'
Assert (!$static.Command('UI GO', 2)) 'Static mode allowed GO'
Assert ($tx.Count -eq 0) 'Static motion commands reached serial'
Assert ($static.Command('UI MARK ROTATE_BEGIN', 10)) 'Record begin rejected'
$static.Observe('FW: -179,0,0,0', 20)
$static.Observe('FW: -178,0,0,0', 30)
Assert ($static.Command('UI MARK END', 40)) 'Record end rejected'
Assert ($messages[-1].Contains('delta_yaw_deg=3.000000')) 'Angle wrapping corrupted the rotation record'
Assert (!$static.Command("UI ARM`nUI GO", 50)) 'Batch command accepted'
Assert (!$static.Command('GO 1', 60)) 'Raw device command accepted over UDP'
Assert (!$static.Command('UI SNAPSHOT UNKNOWN', 70)) 'Unknown snapshot accepted'

$motion = [VofaTestSession]::new($true, $send, $report)
$tx.Clear()
Assert (!$motion.Command('UI ARM', 0)) 'Missing status did not block ARM'
$motion.Observe($idle, 0)
Assert ($motion.Command('UI ARM', 1)) 'ARM failed'
Assert (!$motion.Command('UI GO', 2)) 'GO before ARM ACK accepted'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=37 reason=ARMED', 10)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=37 healthy=1 stopped=1', 20)
Assert ($motion.Command('UI GO', 30)) 'GO failed'
Assert ($tx[-1] -eq 'GO 37') 'Dynamic token not used'
$count = $tx.Count
$motion.Tick(35)
Assert ($tx.Count -eq $count) 'Heartbeat began before GO confirmation'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=37 reason=RUNNING', 40)
$motion.Tick(40)
Assert ($tx[-1] -eq 'HB 37') 'Heartbeat missing after confirmed GO'
$motion.Observe('[INFO] TEST_STATUS phase=2 token=37 healthy=1 stopped=0', 220)
$motion.Tick(240)
Assert ($tx[-1] -eq 'HB 37') 'Heartbeat did not renew at 200ms'
$motion.Tick(621)
Assert ($tx[-1] -eq 'STOP') 'Stale status did not stop the session'
$count = $tx.Count
$motion.Tick(900)
Assert ($tx.Count -eq $count) 'Heartbeat resumed after timeout'
$motion.Observe($idle, 1000)
Assert ($motion.Command('UI ARM', 1001)) 'Second ARM failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=38 reason=ARMED', 1010)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=38 healthy=1 stopped=1', 1020)
Assert ($motion.Command('UI GO', 1030)) 'Second GO failed'
Assert ($tx[-1] -eq 'GO 38') 'Previous token reused'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=999 reason=RUNNING', 1040)
Assert ($tx[-1] -eq 'STOP') 'Wrong GO ACK did not stop'
$motion.Observe($idle, 2000)
Assert ($motion.Command('UI ARM', 2001)) 'ARM for reset test failed'
$motion.Observe('[INFO] TEST_READY mode=6', 2010)
Assert (!$motion.Command('UI GO', 2020)) 'Reset reused a pre-reset token'
$motion.Observe($idle, 3000)
Assert ($motion.Command('UI ARM', 3001)) 'ARM for banner-loss test failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=41 reason=ARMED', 3010)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=41 healthy=1 stopped=1', 3020)
Assert ($motion.Command('UI GO', 3030)) 'GO for banner-loss test failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=41 reason=RUNNING', 3040)
$motion.Observe($idle, 3050)
$count = $tx.Count
$motion.Tick(3060)
Assert ($tx.Count -eq $count) 'Idle-after-reset kept renewing old heartbeats'

$json = Get-Content -Raw -Encoding UTF8 (Join-Path $PSScriptRoot '../vofa/vofa+.cmds.json') | ConvertFrom-Json
Assert ($json.type -eq 'cmds' -and $json.vnumber -eq 100) 'Not a native VOFA command file'
function Leaves($node) {
    if ($node.is_group) {
        Assert (!$node.loop_on) 'Batch execution enabled on a group'
        foreach ($child in $node.subCmds) { Leaves $child }
    } else { $node }
}
$items = @(Leaves $json.ctx)
Assert ($items.Count -eq 13) 'Expected 13 named panel commands'
$static = [VofaTestSession]::new($false, $send, $report)
$static.Observe($idle, 3000)
$static.Observe('FW: 0,0,0,0', 3000)
$tx.Clear()
foreach ($item in $items) {
    Assert ($item.hex_on -and $item.loop_count -eq 1 -and $item.intro.Length -gt 0) 'Missing single-send mode or description'
    [byte[]]$bytes = @($item.cmd_hex -split ' ' | ForEach-Object { [Convert]::ToByte($_,16) })
    Assert ($bytes[-1] -eq 10) 'Missing actual LF terminator'
    $command = [Text.Encoding]::ASCII.GetString($bytes)
    Assert (!$command.Contains('%') -and !$command.Contains('\n')) 'Unresolved parameter or literal escape'
    $accepted = $static.Command($command, 3010)
    if ($command.Trim() -notin @('UI ARM','UI GO')) { Assert $accepted ('JSON command unsupported - ' + $command) }
}
Assert (@($tx | Where-Object { $_ -ne 'PING' -and $_ -ne 'STOP' }).Count -eq 0) 'Static panel sent a motion command'

# Actual bidirectional sockets through the production receive/forward functions.
$server = [Net.Sockets.UdpClient]::new([Net.IPEndPoint]::new([Net.IPAddress]::Loopback,0))
$client = [Net.Sockets.UdpClient]::new([Net.IPEndPoint]::new([Net.IPAddress]::Loopback,0))
try {
    $client.Client.ReceiveTimeout = 1000
    $tx.Clear()
    $peer = [Net.IPEndPoint]::new([Net.IPAddress]::Any,0)
    $port = $client.Client.LocalEndPoint.Port
    $netReport = [Action[string]]{ param($s) $null = [MangoStraightConsole]::ForwardStatus($server,$port,('[INFO] TEST_PANEL ' + $s)) }
    $network = [VofaTestSession]::new($false,$send,$netReport)
    foreach ($command in @("UI PING`n","UI STOP`n","UI ARM`n","UI SNAPSHOT STATE`n")) {
        $bytes = [Text.Encoding]::ASCII.GetBytes($command)
        $null = $client.Send($bytes,$bytes.Length,$server.Client.LocalEndPoint)
        $timer = [Diagnostics.Stopwatch]::StartNew()
        while ($server.Available -eq 0 -and $timer.ElapsedMilliseconds -lt 1000) { Start-Sleep -Milliseconds 1 }
        [MangoStraightConsole]::ReceivePanelCommands($server,$network,0)
    }
    Assert (($tx -join ',') -eq 'PING,STOP') 'UDP handler forwarded a motion or unknown command'
    $reply = [Text.Encoding]::ASCII.GetString($client.Receive([ref]$peer))
    Assert ($reply -eq "[INFO] TEST_PANEL stop_requested=1`n") 'UDP reply bytes changed'
    Assert ($peer.Port -eq $server.Client.LocalEndPoint.Port) 'Replies did not come from the command socket'
} finally { $server.Dispose(); $client.Dispose() }
'VOFA_PANEL_TEST_OK (13 native commands, static gate, dynamic tokens, ACK, heartbeat, timeout, reset, yaw wrap, bidirectional UDP; no hardware opened)'
