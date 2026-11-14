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
Assert (([MangoStraightConsole]::ChineseStatus('[INFO] TEST_CONTROL source=MANUAL manual_armed=0 manual_wait=VRB_RELEASED')).Contains('VRB≤1250')) 'Manual VRB wait reason missing'
Assert (([MangoStraightConsole]::ChineseStatus('[INFO] TEST_CONTROL source=MANUAL manual_armed=1')).Contains('手动已使能')) 'Manual enable feedback missing'
Assert (([MangoStraightConsole]::ChineseStatus('[INFO] TEST_MAP_ACK ok=0 reason=WORK_ATTEMPTS_EXHAUSTED cmd=ARM ALL')).Contains('复检仍有目标')) 'Work attempts fault reason missing'
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
Assert ($static.Command('UI MARK ROTATE_BEGIN', 41)) 'Extended wave record begin rejected'
$static.Observe('FW: -177,0,0,0,3800,460,65127,1320,3800,460,3840,1320,11', 42)
Assert ($static.Command('UI MARK END', 43)) 'Extended wave record end rejected'
Assert ($messages[-1].Contains('delta_yaw_deg=1.000000')) 'Laser channels broke yaw rotation record'
Assert (!$static.Command("UI ARM`nUI GO", 50)) 'Batch command accepted'
Assert (!$static.Command('GO 1', 60)) 'Raw device command accepted over UDP'
Assert (!$static.Command('UI SNAPSHOT UNKNOWN', 70)) 'Unknown snapshot accepted'

# Capture commands remain available from the static launcher, without requiring
# an ARM token or transmitting motion. Device-side checks govern capture/export.
$capture = [VofaTestSession]::new($false, $send, $report)
$tx.Clear()
foreach ($action in @('START','STOP','CLEAR','DUMP')) {
    Assert ($capture.Command("UI TRACE $action`n", 80)) ('Static trace command rejected: ' + $action)
    Assert ($tx[-1] -eq "TRACE $action") ('Trace command translated incorrectly: ' + $action)
    Assert ($messages[-1] -match 'motor_command=NONE') 'Capture response claimed a motor command'
}
Assert (($tx -join ',') -eq 'TRACE START,TRACE STOP,TRACE CLEAR,TRACE DUMP') 'Capture emitted unexpected serial commands'
$beforeCaptureRejects = $tx.Count
foreach ($bad in @('UI TRACE','UI TRACE ERASE','UI TRACE START 1','TRACE START',"UI TRACE START`nUI GO",'UI TRACE START; UI GO')) {
    Assert (!$capture.Command($bad, 81)) ('Invalid capture command accepted: ' + $bad)
}
Assert ($tx.Count -eq $beforeCaptureRejects) 'Rejected capture input reached serial'
Assert ($capture.Command('UI LOG QUIET',82)) 'Quiet preset rejected for capture'
foreach ($reply in @(
    '[INFO] TEST_TRACE_META expected_count=2 first=0 next=2 overwritten=0 reason=STOPPED',
    '[INFO] TEST_TRACE seq=0 part=0 total=1 data=0,1,2',
    '[INFO] TEST_TRACE_STATE rejected=INVALID_BUSY_OR_NOT_STOPPED require=SWD_OFF_FRESH_STATIONARY',
    '[INFO] TEST_TRACE_END sent_count=2')) {
    Assert ($capture.ShouldForwardStatus($reply,83)) ('Quiet filter hid explicit capture reply: ' + $reply)
    Assert ($capture.ShouldForwardStatus($reply,83)) 'Same-time capture chunks were throttled'
}
Assert (!$capture.ShouldForwardStatus($idle,83)) 'Capture support accidentally unmuted periodic status'
Assert (!$capture.ShouldForwardStatus('[INFO] TEST_TRACE_UNKNOWN value=1',83)) 'Unknown trace category bypassed quiet filter'

$motion = [VofaTestSession]::new($true, $send, $report)
$tx.Clear()
Assert (!$motion.Command('UI ARM', 0)) 'Missing status did not block ARM'
$motion.Observe('[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1', 0)
Assert (!$motion.Command('UI ARM', 1)) 'Legacy firmware without target was allowed to arm'
$motion.Observe('[INFO] TEST_STATUS phase=0 token=0 healthy=1 stopped=1 target_m=2.00', 0)
Assert (!$motion.Command('UI ARM', 1)) 'Two-meter firmware was allowed under the one-meter panel'
$motion.Observe($idle, 0)
$motion.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',0)
Assert ($motion.Command('UI ARM', 1)) 'ARM failed'
Assert (!$motion.Command('UI GO', 2)) 'GO before ARM ACK accepted'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=37 reason=ARMED plan=FWD target_m=1.00', 10)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=37 healthy=1 stopped=1 target_m=1.00', 20)
Assert ($motion.Command('UI GO', 30)) 'GO failed'
Assert ($tx[-1] -eq 'GO 37') 'Dynamic token not used'
$count = $tx.Count
$motion.Tick(35)
Assert ($tx.Count -eq $count) 'Heartbeat began before GO confirmation'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=37 reason=RUNNING plan=FWD target_m=1.00', 40)
$motion.Tick(40)
Assert ($tx.Count -eq $count) 'Confirmed GO unexpectedly started host heartbeats'
$motion.Observe('[INFO] TEST_STATUS phase=2 token=37 healthy=1 stopped=0 target_m=1.00', 220)
$motion.Tick(240)
Assert ($tx.Count -eq $count) 'Running session sent continuing commands'
$motion.Tick(621)
Assert ($tx.Count -eq $count) 'Stale status interrupted autonomous MCU execution'
$count = $tx.Count
$motion.Tick(900)
Assert ($tx.Count -eq $count) 'Heartbeat resumed after timeout'
$motion.Observe($idle, 1000)
$motion.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',1000)
Assert ($motion.Command('UI ARM', 1001)) 'Second ARM failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=38 reason=ARMED plan=FWD target_m=1.00', 1010)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=38 healthy=1 stopped=1 target_m=1.00', 1020)
Assert ($motion.Command('UI GO', 1030)) 'Second GO failed'
Assert ($tx[-1] -eq 'GO 38') 'Previous token reused'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=999 reason=RUNNING', 1040)
Assert ($tx[-1] -eq 'GO 38') 'Wrong GO ACK sent an automatic stop'
$motion.Observe($idle, 2000)
$motion.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',2000)
Assert ($motion.Command('UI ARM', 2001)) 'ARM for reset test failed'
$motion.Observe('[INFO] TEST_READY mode=6', 2010)
Assert (!$motion.Command('UI GO', 2020)) 'Reset reused a pre-reset token'
$motion.Observe($idle, 3000)
$motion.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',3000)
Assert ($motion.Command('UI ARM', 3001)) 'ARM for banner-loss test failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=41 reason=ARMED plan=FWD target_m=1.00', 3010)
$motion.Observe('[INFO] TEST_STATUS phase=1 token=41 healthy=1 stopped=1 target_m=1.00', 3020)
Assert ($motion.Command('UI GO', 3030)) 'GO for banner-loss test failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=2 token=41 reason=RUNNING plan=FWD target_m=1.00', 3040)
$motion.Observe($idle, 3050)
$count = $tx.Count
$motion.Tick(3060)
Assert ($tx.Count -eq $count) 'Idle-after-reset kept renewing old heartbeats'
$motion.Observe($idle, 3100)
$motion.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',3100)
Assert ($motion.Command('UI ARM',3101)) 'Transport reset setup ARM failed'
$motion.Observe('[INFO] TEST_ACK ok=1 phase=1 token=42 reason=ARMED plan=FWD target_m=1.00',3110)
$motion.TransportReset()
$count = $tx.Count
Assert (!$motion.Command('UI GO',3120)) 'Serial reconnect retained old ARM token'
Assert (!$motion.Command('UI ARM',3121)) 'Serial reconnect retained stale device readiness'
Assert ($tx.Count -eq $count) 'Serial reconnect emitted a motion command'

# A queued DONE/FAULT status can arrive after sending ARM but before its ACK.
# It belongs to the previous attempt and must not erase the pending transaction.
foreach ($previousPhase in @(4,5)) {
    $messages.Clear()
    $retry = [VofaTestSession]::new($true,$send,$report)
    $previous = "[INFO] TEST_STATUS phase=$previousPhase token=7 healthy=1 stopped=1 target_m=1.00"
    $retry.Observe($previous,0)
    $retry.Observe('[INFO] TEST_PLAN proto=2 plan=WALL leg=OUT target_m=1.00 hb_required=0',0)
    Assert ($retry.Command('UI ARM WALL 1',1)) 'Retry ARM rejected'
    $retry.Observe($previous,2)
    $retry.Observe('[INFO] TEST_ACK ok=1 phase=1 token=8 reason=ARMED plan=WALL target_m=1.00 cmd=ARM',10)
    $retry.Observe('[INFO] TEST_STATUS phase=1 token=8 healthy=1 stopped=1 target_m=1.00',20)
    Assert (@($messages | Where-Object {$_ -match '^arm_ready=1'}).Count -eq 1) 'Queued terminal status erased pending ARM'
    Assert ($retry.Command('UI GO WALL 1',30)) 'GO rejected after valid retry ARM ACK'
    Assert ($tx[-1] -eq 'GO 8') 'Retry used the wrong token'
    $retry.Observe('[INFO] TEST_ACK ok=1 phase=2 token=8 reason=RUNNING plan=WALL target_m=1.00 cmd=GO',40)
    $retry.Observe('[INFO] TEST_STATUS phase=5 token=8 healthy=0 stopped=1 target_m=1.00',50)
    $afterFault = $tx.Count
    $retry.Tick(60)
    Assert ($tx.Count -eq $afterFault) 'A confirmed running fault failed to clear heartbeat'
}

# Runtime owner changes invalidate old starts without emitting a stop into
# manual driving. Fresh ARM is required after returning to HOST.
$owner = [VofaTestSession]::new($true,$send,$report)
$owner.Observe($idle,0)
$owner.Observe('[INFO] TEST_PLAN proto=2 plan=FWD target_m=1.00 hb_required=0',0)
$owner.Observe('[INFO] TEST_CONTROL source=HOST',0)
Assert ($owner.Command('UI ARM',1)) 'Host owner ARM failed'
$owner.Observe('[INFO] TEST_ACK ok=1 phase=1 token=81 plan=FWD target_m=1.00',2)
$owner.Observe('[INFO] TEST_CONTROL source=MANUAL',3)
$count=$tx.Count
Assert (!$owner.Command('UI GO',4)) 'Manual owner accepted old host GO'
$owner.Tick(5000)
Assert ($tx.Count -eq $count) 'Host timeout affected manual owner'
$owner.Observe($idle,5000)
$owner.Observe('[INFO] TEST_PLAN proto=2 plan=FWD target_m=1.00 hb_required=0',5000)
$owner.Observe('[INFO] TEST_CONTROL source=HOST',5000)
Assert (!$owner.Command('UI GO',5001)) 'Switch back reused pre-switch ARM'
Assert ($owner.Command('UI ARM',5002)) 'Fresh ARM after switch rejected'
$owner.Observe('[INFO] TEST_ACK ok=1 phase=1 token=82 plan=FWD target_m=1.00',5003)
Assert ($owner.Command('UI GO',5004)) 'GO fixture failed'
$count=$tx.Count
$owner.Tick(5600)
Assert ($tx.Count -eq $count) 'Missing GO confirmation sent automatic STOP'
Assert ($messages[-1] -match 'go_confirmation=TIMEOUT.*motion_state=UNKNOWN.*auto_stop=0') 'Unknown start outcome misreported'
Assert ($owner.Command('UI STOP',5601)) 'Explicit STOP must always be available'
Assert ($tx[-1] -eq 'STOP') 'Explicit STOP was suppressed'

$json = Get-Content -Raw -Encoding UTF8 (Join-Path $PSScriptRoot '../vofa/vofa+.cmds.json') | ConvertFrom-Json
Assert ($json.type -eq 'cmds' -and $json.vnumber -eq 100) 'Not a native VOFA command file'
function Leaves($node) {
    if ($node.is_group) {
        Assert (!$node.loop_on) 'Batch execution enabled on a group'
        foreach ($child in $node.subCmds) { Leaves $child }
    } else { $node }
}
$items = @(Leaves $json.ctx)
Assert ($items.Count -eq 34) 'Expected 34 full-map mode and work commands'
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
    if ($command.Trim() -notmatch '^UI (ARM|GO|EDGE|SPEED|ACCEL|MAP)\b') { Assert $accepted ('JSON command unsupported - ' + $command) }
}
Assert (@($tx | Where-Object { $_ -ne 'PING' -and $_ -ne 'STOP' -and $_ -notmatch '^MAP (STATUS|STOP|WORK STATUS|WORK RESET|ELBOW HOME)$' -and $_ -notmatch '^TRACE (START|STOP|CLEAR|DUMP)$' }).Count -eq 0) 'Static panel sent a motion command'
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
        $extended.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',0)
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
        Assert ($tx[-1] -eq 'GO 71') 'New plan emitted a periodic heartbeat'
        $extended.Observe("[INFO] TEST_STATUS phase=2 token=71 healthy=1 stopped=1 target_m=$distance result=TURNAROUND",210)
        $extended.Tick(240)
        Assert ($tx[-1] -eq 'GO 71') 'Dwell caused an unsolicited host command'
        Assert (!$extended.Command('UI SPEED 0.05',250)) 'Speed changed during a running plan'
        Assert (!$extended.Command('UI EDGE LEFT',250)) 'Contact reference changed during a running plan'
        Assert ($extended.Command('UI STOP',260)) 'Stop rejected during new plan'
    }
}
$extended = [VofaTestSession]::new($true,$send,$report)
$extended.Observe($idle,0)
$extended.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',0)
Assert ($extended.Command('UI ARM ROUND 4',1)) 'Mismatch fixture ARM failed'
$extended.Observe('[INFO] TEST_ACK ok=1 phase=1 token=72 reason=ARMED plan=ROUND target_m=2 cmd=ARM',10)
$extended.Observe('[INFO] TEST_STATUS phase=1 token=72 healthy=1 stopped=1 target_m=4',20)
Assert (!$extended.Command('UI GO ROUND 4',30)) 'Wrong-distance ARM ACK was accepted'

# Stationary geometry recording is explicit and does not issue ARM/GO.
$tx.Clear()
$config = [VofaTestSession]::new($false,$send,$report)
$config.Observe($idle,0)
$config.Observe('[INFO] TEST_PLAN proto=2 plan=FWD leg=OUT target_m=1.00 hb_required=0',0)
$config.Observe('[INFO] TEST_RANGE fresh=1 age_ms=80 left_mm=109 right_mm=600 rear_mm=800 front_mm=1000',0)
Assert ($config.Command('UI EDGE LEFT',1)) 'Stopped left contact record rejected'
Assert ($tx[-1] -eq 'EDGE ALL 110') 'Contact boundary was not rounded conservatively'
Assert ($config.Command('UI SPEED 0.08',2)) 'Stopped speed preset rejected'
Assert ($tx[-1] -eq 'SPEED 0.08') 'Wrong speed preset sent'
Assert ($config.Command('UI SPEED 0.50',3)) 'Fast speed preset rejected'
Assert ($tx[-1] -eq 'SPEED 0.50') 'Fast speed preset changed'
Assert (!$config.Command('UI SPEED 0.90',3)) 'Unlisted speed accepted'
foreach ($preset in @('0.50','1.00','1.50','1.80','2.00')) {
    Assert ($config.Command("UI SPEED $preset",3)) "New speed preset rejected: $preset"
    Assert ($tx[-1] -eq "SPEED $preset") 'Speed preset changed in bridge'
}
$panelFile = Get-Content -Raw -Encoding UTF8 (Join-Path $PSScriptRoot '../vofa/vofa+.cmds.json') | ConvertFrom-Json
$buttons = ($panelFile.ctx.subCmds | Where-Object name -like '02 *').subCmds
$speedButtons = @($buttons | Where-Object cmd_hex -like '55 49 20 4D 41 50 20 53 50 45 45 44 *')
Assert ($speedButtons.Count -eq 5) 'Five speed buttons expected'
$expectedPresets = @('0.50','0.60','0.70','0.80','0.90')
for ($i = 0; $i -lt 5; ++$i) {
    $decoded = [Text.Encoding]::ASCII.GetString([byte[]]($speedButtons[$i].cmd_hex.Split(' ') | ForEach-Object {[Convert]::ToByte($_,16)}))
    Assert ($decoded -eq "UI MAP SPEED $($expectedPresets[$i])`n") 'Speed button bytes do not match label'
}
foreach ($accel in @('0.10','0.20','0.30','0.50')) {
    Assert ($config.Command("UI ACCEL $accel",3)) 'Stopped acceleration setting rejected'
    Assert ($tx[-1] -eq "ACCEL $accel") 'Acceleration value changed in bridge'
}
foreach ($badAccel in @('UI ACCEL 0','UI ACCEL 0.51','UI ACCEL NaN','UI ACCEL -0.1',"UI ACCEL 0.2`nUI GO")) {
    Assert (!$config.Command($badAccel,3)) 'Invalid acceleration accepted'
}
Assert (@($items | Where-Object name -like '*单次*').Count -eq 0) 'Single-run tests remain on the routine panel'
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

# Manual slew telemetry shares the existing health toggle; no extra panel group.
$ramp = '[INFO] TEST_RAMP t_ms=1008 target=1.5,0,0 output=0.2,0,0 reversing=0'
Assert ($filters.ShouldForwardStatus($ramp,1008)) 'Default view hid manual ramp'
Assert ($filters.Command('UI LOG QUIET',1009)) 'Quiet preset rejected for ramp'
Assert (!$filters.ShouldForwardStatus($ramp,1010)) 'Quiet view leaked manual ramp'
$filters.Observe($ramp,1010)
$messages.Clear()
Assert ($filters.Command('UI SNAPSHOT STATE',1010)) 'Quiet view blocked ramp snapshot'
Assert (@($messages | Where-Object {$_ -match 'cached .*TEST_RAMP'}).Count -eq 1) 'Muted ramp missing from state snapshot'
Assert ($filters.Command('UI LOG HEALTH ON',1011)) 'Health enable rejected'
Assert ($filters.ShouldForwardStatus($ramp,1012)) 'Health enable did not include ramp'
Assert ($filters.Command('UI LOG HEALTH OFF',1013)) 'Health disable rejected'
Assert (!$filters.ShouldForwardStatus($ramp,1014)) 'Health disable leaked ramp'
Assert ($filters.Command('UI LOG ALL ON',1015)) 'All preset rejected for ramp'
Assert ($filters.ShouldForwardStatus($ramp,1016)) 'All view hid manual ramp'

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
$mapPanel = [VofaTestSession]::new($true,$send,$report)
$mapPanel.Observe($idle + ' control=HOST',0)
Assert (!$mapPanel.Command('UI MAP ARM ALL',1)) 'Legacy firmware accepted map ARM'
$mapPanel.Observe('[INFO] TEST_MAP_STATUS proto=1 phase=READY token=0',10)
Assert ($mapPanel.Command('UI MAP ORIGIN',11)) 'Map origin rejected'
Assert (!$mapPanel.Command('UI MAP ARM NEXT',11)) 'Removed segmented command accepted'
Assert ($mapPanel.Command('UI MAP SPEED 0.60',11)) 'Map fast preset rejected'
Assert (!$mapPanel.Command('UI MAP SPEED 0.10',11)) 'Old map speed accepted'
Assert ($mapPanel.Command('UI MAP MODE NAV',11)) 'Navigation mode rejected'
Assert ($tx[-1] -eq 'MAP MODE NAV') 'Navigation selection changed in bridge'
Assert ($mapPanel.Command('UI MAP MODE WORK',11)) 'Work mode rejected'
Assert ($tx[-1] -eq 'MAP MODE WORK') 'Work selection changed in bridge'
Assert (!$mapPanel.Command('UI MAP MODE OTHER',11)) 'Unknown map mode accepted'
Assert ($mapPanel.Command('UI MAP ARM ALL',12)) 'Map ARM rejected'
Assert (!$mapPanel.Command('UI MAP GO',13)) 'Map GO before ACK accepted'
$mapPanel.Observe('[INFO] TEST_MAP_ACK ok=1 phase=ARMED token=91 segment=0 reason=ARMED cmd=ARM NEXT',14)
Assert (!$mapPanel.Command('UI MAP GO',15)) 'Mismatched map selection ACK armed a run'
$mapPanel.Observe('[INFO] TEST_MAP_ACK ok=1 phase=ARMED token=92 segment=0 reason=ARMED cmd=ARM ALL',16)
Assert ($mapPanel.Command('UI MAP GO',17)) 'Matching map GO rejected'
Assert ($tx[-1] -eq 'MAP GO 92') 'Wrong map token'
$mapPanel.Observe('[INFO] TEST_MAP_STATUS proto=1 phase=RUNNING token=92',20)
Assert (!$mapPanel.Command('UI MAP MODE NAV',21)) 'Mode changed during a running map'
Assert (!$mapPanel.Command('UI ARM WALL 4',21)) 'Straight session interfered with full map'
Assert ($mapPanel.Command('UI MAP PAUSE',22)) 'Map pause rejected'
Assert ($mapPanel.Command('UI STOP',23)) 'Global stop rejected during map'
$mapPanel.TransportReset()
Assert (!$mapPanel.Command('UI MAP GO',24)) 'Map token survived transport reset'
Assert ($mapPanel.Command('UI LOG QUIET',25)) 'Quiet preset rejected'
Assert ($mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_SNAPSHOT valid=1 x=0.2 y=-0.5',26)) 'Explicit pose query hidden by quiet preset'
Assert ($mapPanel.Command('UI LOG MAP ON',27)) 'Map logs could not be enabled'
Assert ($mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_AISLE active=1 frozen=0',28)) 'Aisle log hidden by map toggle'
Assert ($mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_TREE hold=1 elapsed_ms=500',28)) 'Tree dwell log hidden'
Assert ($mapPanel.Command('UI LOG MAP OFF',29)) 'Map logs could not be disabled'
Assert (!$mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_AISLE active=1 frozen=0',30)) 'Aisle log ignored map toggle'
Assert (!$mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_TREE hold=1',30)) 'Tree log ignored map toggle'
Assert ($mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_TREE_EVENT hold=1',30)) 'Tree event hidden'
Assert ($mapPanel.ShouldForwardStatus('[INFO] TEST_MAP_CONTROL_FAULT process_code=8',30)) 'Control fault hidden'
'VOFA_PANEL_TEST_OK (34 commands, map modes and handshake, acceleration/speeds, TRACE, UDP)'

$workPanel = [VofaTestSession]::new($true,$send,$report)
$workPanel.Observe($idle + ' control=HOST',0)
$workPanel.Observe('[INFO] TEST_MAP_STATUS proto=1 phase=READY token=0 segment=0',1)
foreach ($preset in @('0.15','0.20','0.25','0.30','0.35')) {
    Assert ($workPanel.Command("UI MAP APPROACH $preset",2)) 'Approach preset rejected'
    Assert ($tx[-1] -eq "MAP APPROACH $preset") 'Approach command changed in bridge'
}
Assert (!$workPanel.Command('UI MAP APPROACH 0.10',2)) 'Unsupported approach speed accepted'
Assert ($workPanel.Command('UI MAP WORK STATUS',3)) 'Work status query rejected'
Assert ($tx[-1] -eq 'MAP WORK STATUS') 'Work query changed in bridge'
Assert ($workPanel.Command('UI MAP WORK RESET',4)) 'Work reset rejected'
Assert ($workPanel.ShouldForwardStatus('[INFO] TEST_WORK enabled=1 phase=2 task=1 station=12',5)) 'Work status hidden'
'VOFA_WORK_QUERY_AND_APPROACH_PRESETS_OK'

$notReadyMap = [VofaTestSession]::new($true,$send,$report)
$notReadyMap.Observe('[INFO] TEST_STATUS phase=0 token=0 healthy=0 stopped=1 target_m=1.00 control=HOST blocked=RC_SWITCH',0)
$notReadyMap.Observe('[INFO] TEST_MAP_STATUS proto=1 phase=READY token=0',0)
$tx.Clear()
Assert (!$notReadyMap.Command('UI MAP ARM ALL',1)) 'Unsafe map preparation accepted'
Assert ($messages[-1].Contains('rejected=RC_SWITCH')) 'Missing concrete preparation rejection'
Assert ($tx.Count -eq 0) 'Rejected preparation reached the board'
Assert ($notReadyMap.Command('UI MAP ELBOW HOME',2)) 'Stopped manual-home confirmation rejected'
Assert ($tx[-1] -eq 'MAP ELBOW HOME') 'Home confirmation command changed'
