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
$idle = '[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1 target_m=1.00 cmd=0.000'
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
$motion.Observe('[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1', 0)
Assert (!$motion.Command('UI ARM', 1)) 'Legacy firmware without target was allowed to arm'
$motion.Observe('[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1 target_m=2.00', 0)
Assert (!$motion.Command('UI ARM', 1)) 'Two-meter firmware was allowed under the one-meter panel'
$motion.Observe($idle, 0)
Assert ($motion.Command('UI ARM', 1)) 'ARM failed'
Assert (!$motion.Command('UI GO', 2)) 'GO before ARM ACK accepted'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=37 reason=ARMED', 10)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=37 healthy=1 stopped=1 target_m=1.00', 20)
Assert ($motion.Command('UI GO', 30)) 'GO failed'
Assert ($tx[-1] -eq 'GO 37') 'Dynamic token not used'
$count = $tx.Count
$motion.Tick(35)
Assert ($tx.Count -eq $count) 'Heartbeat began before GO confirmation'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=37 reason=RUNNING', 40)
$motion.Tick(40)
Assert ($tx[-1] -eq 'HB 37') 'Heartbeat missing after confirmed GO'
$motion.Observe('[INFO] TEST_STATUS phase=2 token=37 healthy=1 stopped=0 target_m=1.00', 220)
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
$motion.Observe('[INFO] TEST_STATUS phase=1 token=38 healthy=1 stopped=1 target_m=1.00', 1020)
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
$motion.Observe('[INFO] TEST_STATUS phase=1 token=41 healthy=1 stopped=1 target_m=1.00', 3020)
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
Assert ($items.Count -eq 74) 'Expected 74 named panel commands'
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
    if ($command.Trim() -notmatch '^UI (ARM|GO|EDGE|SPEED)\b') { Assert $accepted ('JSON command unsupported - ' + $command) }
}
Assert (@($tx | Where-Object { $_ -ne 'PING' -and $_ -ne 'STOP' }).Count -eq 0) 'Static panel sent a motion command'
$messages.Clear()
Assert ($static.Command('UI SNAPSHOT REMOTE', 4000)) 'Stale snapshot command rejected as unknown'
Assert ($messages[-1].Contains('UART_STATUS_STALE')) 'Stale UART data was presented as a current snapshot'
Assert (@($messages | Where-Object { $_.StartsWith('cached ') }).Count -eq 0) 'Stale cached channels were returned'

# New plans must never run on legacy firmware, or under a different group's GO.
$tx.Clear()
$extended = [VofaTestSession]::new($true,$send,$report)
$extended.Observe($idle,0)
Assert (!$extended.Command('UI ARM ROUND 4',1)) 'Legacy firmware accepted a four-meter round trip'
foreach ($plan in @('ROUND','WALL','APPROACH')) {
    foreach ($distance in $(if($plan -eq 'APPROACH'){@(1)}else{@(1,2,4)})) {
        $extended = [VofaTestSession]::new($true,$send,$report)
        $extended.Observe($idle,0)
        $extended.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00',0)
        $suffix = if($plan -eq 'APPROACH'){$plan}else{"$plan $distance"}
        Assert ($extended.Command("UI ARM $suffix",1)) 'New ARM rejected'
        Assert ($tx[-1] -eq "ARM $suffix") 'Wrong program or distance sent to firmware'
        # A delayed config ACK must not consume the outstanding ARM transaction.
        $extended.Observe('[INFO] TEST_ACK ok=1 phase=0 token=0 reason=SPEED_SET cmd=SPEED',2)
        $extended.Observe("[INFO] TEST_ACK ok=1 phase=1 token=71 reason=ARMED plan=$plan target_m=$distance cmd=ARM",10)
        $extended.Observe("[INFO] TEST_STATUS phase=1 token=71 healthy=1 stopped=1 target_m=$distance",20)
        Assert (!$extended.Command('UI GO',21)) 'A single-forward GO started a different selected plan'
        Assert ($extended.Command("UI GO $suffix",30)) 'Matching program GO rejected'
        $extended.Observe("[INFO] TEST_ACK ok=1 phase=2 token=71 reason=RUNNING plan=$plan target_m=$distance cmd=GO",40)
        $extended.Tick(40)
        Assert ($tx[-1] -eq 'HB 71') 'New plan heartbeat missing'
        $extended.Observe("[INFO] TEST_STATUS phase=2 token=71 healthy=1 stopped=1 target_m=$distance result=TURNAROUND",210)
        $extended.Tick(240)
        Assert ($tx[-1] -eq 'HB 71') 'Dwell stopped heartbeats before the reverse leg'
        Assert (!$extended.Command('UI SPEED 0.05',250)) 'Speed changed during a running plan'
        Assert (!$extended.Command('UI EDGE LEFT',250)) 'Contact reference changed during a running plan'
        Assert ($extended.Command('UI STOP',260)) 'Stop rejected during new plan'
    }
}
$extended = [VofaTestSession]::new($true,$send,$report)
$extended.Observe($idle,0)
$extended.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00',0)
Assert ($extended.Command('UI ARM ROUND 4',1)) 'Mismatch fixture ARM failed'
$extended.Observe('[INFO] TEST_ACK ok=1 phase=1 token=72 reason=ARMED plan=ROUND target_m=2 cmd=ARM',10)
$extended.Observe('[INFO] TEST_STATUS phase=1 token=72 healthy=1 stopped=1 target_m=4',20)
Assert (!$extended.Command('UI GO ROUND 4',30)) 'Wrong-distance ARM ACK was accepted'

# Stationary geometry recording is explicit and does not issue ARM/GO.
$tx.Clear()
$config = [VofaTestSession]::new($false,$send,$report)
$config.Observe($idle,0)
$config.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00',0)
$config.Observe('[INFO] TEST_RANGE fresh=1 age_ms=80 left_mm=109 right_mm=600 rear_mm=800 front_mm=1000',0)
Assert ($config.Command('UI EDGE LEFT',1)) 'Stopped left contact record rejected'
Assert ($tx[-1] -eq 'EDGE ALL 110') 'Contact boundary was not rounded conservatively'
Assert ($config.Command('UI SPEED 0.08',2)) 'Stopped speed preset rejected'
Assert ($tx[-1] -eq 'SPEED 0.08') 'Wrong speed preset sent'
Assert ($config.Command('UI SPEED 0.50',3)) 'Fast speed preset rejected'
Assert ($tx[-1] -eq 'SPEED 0.50') 'Fast speed preset changed'
Assert (!$config.Command('UI SPEED 0.90',3)) 'Unlisted speed accepted'
Assert (!$config.Command('UI ARM ROUND 1',4)) 'Static launcher permitted new motion'
Assert (!$config.Command('UI EDGE LEFT',601)) 'Stale contact sample accepted'

# Display filters never remove raw observations, command ACKs, or explicit queries.
$filters = [VofaTestSession]::new($false,$send,$report)
Assert (!$filters.WaveEnabled) 'Default compact view unexpectedly streams FW text'
Assert ($filters.ShouldForwardStatus($idle,0)) 'Compact view lost initial status'
Assert (!$filters.ShouldForwardStatus($idle,100)) 'Compact status flooded faster than 1 Hz'
Assert ($filters.ShouldForwardStatus($idle,1000)) 'Compact status never refreshed'
$filters.Observe($idle,1000)
$filters.Observe('[INFO] TEST_REMOTE t_ms=1000 SWD=2000 VRB=1500',1000)
Assert ($filters.Command('UI LOG QUIET',1001)) 'Quiet preset rejected'
Assert (!$filters.ShouldForwardStatus($idle,1002)) 'Quiet view forwards periodic status'
Assert ($filters.ShouldForwardStatus('[INFO] TEST_ACK ok=1 phase=0 token=0',1002)) 'Quiet view hid command ACK'
Assert ($filters.Command('UI LOG CAN2 ON',1002)) 'Per-CAN visibility rejected'
Assert ($filters.ShouldForwardStatus('[INFO] TEST_CAN bus=2 rx=1',1003)) 'CAN2 enable failed'
Assert (!$filters.ShouldForwardStatus('[INFO] TEST_CAN bus=1 rx=1',1003)) 'CAN1 leaked through CAN2-only filter'
$messages.Clear()
Assert ($filters.Command('UI SNAPSHOT REMOTE',1004)) 'Quiet mode blocked explicit query'
Assert (@($messages | Where-Object {$_ -match 'cached .*SWD=2000'}).Count -eq 1) 'Muted remote observation disappeared from cache'
Assert ($filters.Command('UI LOG WAVE ON',1005)) 'Wave toggle rejected'
Assert ($filters.WaveEnabled) 'Wave enable failed'
Assert (!$filters.Command('UI LOG UNKNOWN ON',1006)) 'Unknown log filter accepted'
Assert ($filters.Command('UI LOG DEFAULT',1007)) 'Default preset rejected'
Assert (!$filters.WaveEnabled) 'Default preset failed to restore quiet wave state'

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
'VOFA_PANEL_TEST_OK (74 commands, 1/2/4m plans, fast speeds, matching ACK/GO, dwell HB, edge/speed locks, detailed log filters, legacy/static gates, UDP)'
