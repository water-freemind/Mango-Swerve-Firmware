#requires -PSEdition Desktop
#requires -Version 5.1
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$BaudRate = 921600,
    [ValidateRange(0,65535)][int]$WavePort = 0,
    [ValidateRange(0,65535)][int]$CommandPort = 0,
    [switch]$AllowMotion,
    [switch]$UiOnly,
    [switch]$ExitWithVofa,
    [ValidateRange(0,3600)][int]$RunSeconds = 0
)
$ErrorActionPreference = 'Stop'
if ([Console]::IsInputRedirected -and !$UiOnly) { throw 'Use an interactive terminal, or -UiOnly for the VOFA UDP panel.' }
if ($CommandPort -ne 0 -and ($WavePort -eq 0 -or $CommandPort -eq $WavePort)) {
    throw 'CommandPort requires a different, nonzero WavePort.'
}
New-Item -ItemType Directory -Force (Join-Path $PSScriptRoot 'build/logs') | Out-Null
$logPath = Join-Path $PSScriptRoot ('build/logs/straight-console-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.log')
Add-Type -TypeDefinition @'
using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Globalization;
using System.Net;
using System.Net.Sockets;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
using System.Collections.Generic;
public static class MangoStraightConsole {
    public static bool ForwardWave(UdpClient udp, int port, string line) {
        if (port == 0 || !line.StartsWith("FW:", StringComparison.Ordinal)) return false;
        string[] fields = line.Substring(3).Trim().Split(',');
        if (fields.Length != 4 && fields.Length != 13) return false;
        foreach (string field in fields) {
            double value;
            if (!Double.TryParse(field, NumberStyles.Float, CultureInfo.InvariantCulture, out value) ||
                Double.IsNaN(value) || Double.IsInfinity(value)) return false;
        }
        byte[] bytes = Encoding.ASCII.GetBytes(line.TrimEnd('\r') + "\n");
        udp.Send(bytes, bytes.Length, new IPEndPoint(IPAddress.Loopback, port));
        return true;
    }
    public static bool ForwardStatus(UdpClient udp, int port, string line) {
        if (port == 0) return false;
        string plain = Regex.Replace(line, @"\x1b\[[0-9;]*m", "").TrimEnd('\r');
        // FireWater treats nonnumeric lines without a colon as ordinary text.
        // Keep a narrow status whitelist and do not create numeric-looking log frames.
        if (!Regex.IsMatch(plain, @"^\[(INFO|WARN|ERROR)\] TEST_(STATUS|ACK|READY|INIT_FAILED|HEALTH|CONTROL|RAMP|REMOTE|IMU|IMU_RAW|IMU_BIAS|MOTOR|CAN|LASER|LASER_VALID|WALL|PAUSE|PAUSE_RANGE|FAULT|FAULT_RANGE|TIMING|DRIFT|PLAN|RANGE|BRAKE|PANEL|TRACE|TRACE_META|TRACE_STATE|TRACE_END)\b") ||
            plain.IndexOf(':') >= 0) return false;
        byte[] bytes = Encoding.ASCII.GetBytes(plain + "\n");
        udp.Send(bytes, bytes.Length, new IPEndPoint(IPAddress.Loopback, port));
        return true;
    }
    public static void ReceivePanelCommands(UdpClient udp, VofaTestSession panel, long now) {
        // Bound work per loop; arbitrary serial text and token values are not accepted.
        for (int i = 0; i < 8 && udp.Available != 0; ++i) {
            IPEndPoint peer = new IPEndPoint(IPAddress.Any, 0);
            byte[] packet = udp.Receive(ref peer);
            if (IPAddress.IsLoopback(peer.Address) && packet.Length <= 128)
                panel.Command(Encoding.ASCII.GetString(packet), now);
        }
    }
    public static void Run(string port, int baud, string logPath, int wavePort, int commandPort, bool allowMotion, bool uiOnly, int runSeconds, bool exitWithVofa) {
        using (SerialPort serial = new SerialPort(port, baud, Parity.None, 8, StopBits.One))
        using (UdpClient wave = commandPort == 0 ? new UdpClient() :
            new UdpClient(new IPEndPoint(IPAddress.Loopback, commandPort)))
        using (StreamWriter log = new StreamWriter(logPath, false, Encoding.UTF8)) {
            // Windows reports ICMP port-unreachable as a socket reset when
            // VOFA disconnects. Keep the UDP listener alive for its reconnect.
            wave.Client.IOControl((IOControlCode)unchecked((int)0x9800000C), new byte[4], null);
            serial.Handshake = Handshake.None;
            serial.DtrEnable = false; serial.RtsEnable = false;
            serial.WriteTimeout = 100; serial.ReadTimeout = 100;
            serial.NewLine = "\n";
            log.AutoFlush = true;
            uint token = 0;
            StringBuilder input = new StringBuilder();
            StringBuilder pending = new StringBuilder();
            Stopwatch clock = Stopwatch.StartNew();
            long nextAppCheck = 0;
            Action<string> panelLog = delegate(string text) {
                string message = "[INFO] TEST_PANEL " + text;
                Console.WriteLine(message); log.WriteLine(message);
                try { ForwardStatus(wave, wavePort, message); } catch (SocketException) { }
            };
            long nextSerialTry = 0, lastProtocol = 0;
            VofaTestSession panel = null;
            Action<string> disconnectSerial = delegate(string reason) {
                panel.TransportReset(); pending.Clear();
                try { serial.Close(); } catch (IOException) { }
                nextSerialTry = clock.ElapsedMilliseconds + 1000;
                panelLog("uart_online=0 retry_ms=1000 reason=" + reason);
            };
            panel = new VofaTestSession(allowMotion, delegate(string command) {
                if (!serial.IsOpen) { panelLog("rejected=UART_OFFLINE command=" + command); return; }
                try { serial.WriteLine(command); log.WriteLine("HOST_TX " + command); }
                catch (Exception ex) {
                    if (!(ex is IOException || ex is InvalidOperationException || ex is UnauthorizedAccessException || ex is TimeoutException)) throw;
                    disconnectSerial(ex.GetType().Name);
                }
            }, panelLog);
            if (commandPort != 0) panelLog("ready=1 command_port=" + commandPort + " wave_port=" + wavePort +
                " motion_allowed=" + (allowMotion ? "1" : "0") + " auto_start=0");
            if (commandPort != 0) panelLog("log_preset=DEFAULT STATUS_and_PLAN_max_hz=1 wave=0 ACK_and_snapshots=ALWAYS background_log=FULL");
            Console.WriteLine("No auto-start. Commands: PING, ARM, GO <token>, STOP, QUIT. No host heartbeat or automatic stop on exit. Escape = STOP; SWD=1000 stops independently.");
            try {
                while (true) {
                    if (runSeconds != 0 && clock.ElapsedMilliseconds >= runSeconds * 1000L) break;
                    if (exitWithVofa && clock.ElapsedMilliseconds >= nextAppCheck) {
                        Process[] apps = Process.GetProcessesByName("vofa+");
                        bool closed = apps.Length == 0;
                        foreach (Process app in apps) app.Dispose();
                        if (closed) break;
                        nextAppCheck = clock.ElapsedMilliseconds + 500;
                    }
                    string rx = "";
                    try {
                        if (!serial.IsOpen && clock.ElapsedMilliseconds >= nextSerialTry) {
                            serial.BaudRate = baud;
                            serial.Open(); serial.DiscardInBuffer();
                            pending.Clear(); panel.TransportReset(); lastProtocol = clock.ElapsedMilliseconds;
                            panelLog("uart_open=1 baud=" + baud + " waiting_for_protocol=1");
                        }
                        if (serial.IsOpen) rx = serial.ReadExisting();
                    } catch (Exception ex) {
                        if (!(ex is IOException || ex is InvalidOperationException || ex is UnauthorizedAccessException || ex is TimeoutException)) throw;
                        disconnectSerial(ex.GetType().Name);
                    }
                    if (rx.Length != 0) {
                        Console.Write(rx); log.Write(rx); pending.Append(rx);
                        string text = pending.ToString(); int end;
                        while ((end = text.IndexOf('\n')) >= 0) {
                            string line = text.Substring(0, end); text = text.Substring(end + 1);
                            if (Regex.IsMatch(line, @"TEST_STATUS phase=\d+ token=\d+ healthy=[01] stopped=[01]"))
                                lastProtocol = clock.ElapsedMilliseconds;
                            panel.Observe(line, clock.ElapsedMilliseconds);
                            try {
                                if (line.StartsWith("FW:", StringComparison.Ordinal)) {
                                    if (panel.WaveEnabled) ForwardWave(wave, wavePort, line);
                                } else if (panel.ShouldForwardStatus(line, clock.ElapsedMilliseconds)) ForwardStatus(wave, wavePort, line);
                            }
                            catch (SocketException ex) { log.WriteLine("WAVE_FORWARD_ERROR " + ex.Message); }
                            Match m = Regex.Match(line, @"TEST_ACK ok=1 phase=1 token=(\d+)");
                            if (m.Success) UInt32.TryParse(m.Groups[1].Value, out token);
                        }
                        pending.Clear(); pending.Append(text.Length > 2048 ? "" : text);
                    }
                    if (serial.IsOpen && clock.ElapsedMilliseconds - lastProtocol > 5000)
                        disconnectSerial("NO_VALID_STATUS_5S");
                    if (commandPort != 0) {
                        ReceivePanelCommands(wave, panel, clock.ElapsedMilliseconds);
                        panel.Tick(clock.ElapsedMilliseconds);
                    }
                    if (!uiOnly && Console.KeyAvailable) {
                        ConsoleKeyInfo key = Console.ReadKey(true);
                        if (key.Key == ConsoleKey.Escape) { serial.WriteLine("STOP"); input.Clear(); }
                        else if (key.Key == ConsoleKey.Enter) {
                            string command = input.ToString().Trim(); input.Clear(); Console.WriteLine();
                            if (command.Equals("QUIT", StringComparison.OrdinalIgnoreCase)) break;
                            if (command.Equals("GO", StringComparison.OrdinalIgnoreCase)) command = "GO " + token;
                            if (command.Length != 0) { serial.WriteLine(command); log.WriteLine("HOST_TX " + command); }
                        } else if (key.Key == ConsoleKey.Backspace) {
                            if (input.Length > 0) { input.Length--; Console.Write("\b \b"); }
                        } else if (!Char.IsControl(key.KeyChar)) { input.Append(key.KeyChar); Console.Write(key.KeyChar); }
                    }
                    Thread.Sleep(5);
                }
            } finally {
                // Motion belongs to the MCU/RC after GO. Closing a viewer must
                // neither abort that run nor stop independent manual driving.
                serial.Close();
            }
        }
    }
}
public sealed class VofaTestSession {
    private readonly bool allowMotion;
    private readonly Action<string> tx, report;
    private readonly Dictionary<string, string> cache = new Dictionary<string, string>();
    private long lastStatus = -10000, armSent, armAck, goSent;
    private bool healthy, stopped, awaitingArm, armed, awaitingGo, running, targetMatches;
    private bool firmwareV2, firmwareOneShot, rangeFresh;
    private string controlMode = "UNKNOWN";
    private long lastPlan = -10000, lastRange = -10000;
    private double firmwareTarget;
    private string selectedPlan = "FWD", observedPlan = "FWD";
    private int selectedTarget = 1, phaseValue;
    private readonly Dictionary<string, int> ranges = new Dictionary<string, int>();
    private static readonly string[] logCategories = {"STATUS","HEALTH","REMOTE","MOTOR","CAN1","CAN2","CAN3",
        "IMU","IMU_RAW","IMU_BIAS","DRIFT","LASER","RANGE","PLAN","BRAKE","TIMING","WAVE"};
    private readonly Dictionary<string, bool> logEnabled = new Dictionary<string, bool>();
    private readonly Dictionary<string, long> lastForward = new Dictionary<string, long>();
    private bool compactLogs;
    private uint activeToken;
    private bool haveYaw, marking;
    private double yaw, markedChange;
    private long markTime, lastYaw = -10000;
    private string markKind;
    public VofaTestSession(bool motion, Action<string> send, Action<string> emit) {
        allowMotion = motion; tx = send; report = emit;
        LogPreset("DEFAULT");
    }
    public bool WaveEnabled { get { return logEnabled["WAVE"]; } }
    private void LogPreset(string preset) {
        foreach (string category in logCategories) logEnabled[category] = preset == "ALL";
        compactLogs = preset == "DEFAULT";
        if (compactLogs) logEnabled["STATUS"] = logEnabled["HEALTH"] = logEnabled["PLAN"] = true;
        lastForward.Clear();
    }
    public bool ShouldForwardStatus(string raw, long now) {
        string line = Regex.Replace(raw, @"\x1b\[[0-9;]*m", "");
        Match tag = Regex.Match(line, @"^\[(INFO|WARN|ERROR)\] TEST_([A-Z_]+)\b");
        if (!tag.Success) return false;
        string category = tag.Groups[2].Value;
        if (category == "TRACE" || category == "TRACE_META" || category == "TRACE_STATE" || category == "TRACE_END") return true;
        if (category == "CONTROL" || category == "RAMP" || category == "WALL") category = "HEALTH";
        if (category == "PAUSE" || category == "PAUSE_RANGE") return true;
        if (category == "LASER_VALID") category = "LASER";
        if (tag.Groups[1].Value != "INFO" || category == "ACK" || category == "READY" || category == "INIT_FAILED") return true;
        if (category == "CAN") {
            Match bus = Regex.Match(line, @"\bbus=([123])\b");
            if (!bus.Success) return false;
            category += bus.Groups[1].Value;
        }
        bool enabled;
        if (!logEnabled.TryGetValue(category, out enabled) || !enabled) return false;
        long last;
        if (compactLogs && (category == "STATUS" || category == "PLAN") &&
            lastForward.TryGetValue(category, out last) && now - last < 1000) return false;
        lastForward[category] = now;
        return true;
    }
    private void Clear() { awaitingArm = armed = awaitingGo = running = false; activeToken = 0; }
    public void TransportReset() {
        Clear(); lastStatus = lastPlan = lastRange = -10000;
        firmwareV2 = firmwareOneShot = rangeFresh = haveYaw = marking = false;
        controlMode = "UNKNOWN"; cache.Clear(); ranges.Clear();
    }
    private bool SelectionMatches(string line) {
        if (!firmwareV2 && selectedPlan == "FWD" && selectedTarget == 1) return true;
        Match plan = Regex.Match(line, @"\bplan=(FWD|ROUND|WALL|APPROACH)\b");
        Match distance = Regex.Match(line, @"\btarget_m=([0-9.]+)\b");
        double target;
        return plan.Success && plan.Groups[1].Value == selectedPlan && distance.Success &&
            Double.TryParse(distance.Groups[1].Value, NumberStyles.Float, CultureInfo.InvariantCulture, out target) &&
            Math.Abs(target - selectedTarget) < 0.005;
    }
    public void Observe(string raw, long now) {
        string line = Regex.Replace(raw, @"\x1b\[[0-9;]*m", "").TrimEnd('\r');
        Match tag = Regex.Match(line, @"^\[(?:INFO|WARN|ERROR)\] (TEST_[A-Z_]+)\b");
        if (tag.Success) {
            string key = tag.Groups[1].Value;
            Match id = Regex.Match(line, @"\b(?:id|bus)=(\d+)");
            if (id.Success) key += "_" + id.Groups[1].Value;
            cache[key] = line;
        }
        if (line.Contains("TEST_READY")) {
            TransportReset();
        }
        Match planStatus = Regex.Match(line, @"TEST_PLAN proto=2 plan=(FWD|ROUND|WALL|APPROACH)\b");
        if (planStatus.Success) {
            firmwareV2 = true; lastPlan = now; observedPlan = planStatus.Groups[1].Value;
            firmwareOneShot = Regex.IsMatch(line, @"\bhb_required=0\b");
        }
        Match owner = Regex.Match(line, @"(?:TEST_CONTROL[^\r\n]*\bsource=|\bcontrol=)(HOST|MANUAL|STOP)\b");
        if (owner.Success) {
            controlMode = owner.Groups[1].Value;
            if (controlMode != "HOST") Clear();
        }
        if (line.Contains("TEST_RANGE ")) {
            lastRange = now; rangeFresh = Regex.IsMatch(line, @"\bfresh=1\b");
            Match age = Regex.Match(line, @"\bage_ms=(\d+)");
            uint ageMs;
            rangeFresh = rangeFresh && age.Success && UInt32.TryParse(age.Groups[1].Value, out ageMs) && ageMs <= 300;
            foreach (string direction in new string[] {"LEFT", "RIGHT", "REAR", "FRONT"}) {
                Match mm = Regex.Match(line, @"\b" + direction.ToLowerInvariant() + @"_mm=(\d+)");
                int value;
                if (mm.Success && Int32.TryParse(mm.Groups[1].Value, out value)) ranges[direction] = value;
                else rangeFresh = false;
            }
        }
        Match status = Regex.Match(line, @"TEST_STATUS phase=(\d+) token=(\d+) healthy=(\d+) stopped=(\d+)");
        if (status.Success) {
            lastStatus = now;
            healthy = status.Groups[3].Value == "1"; stopped = status.Groups[4].Value == "1";
            Match distance = Regex.Match(line, @"\btarget_m=([0-9.]+)\b");
            double target;
            targetMatches = distance.Success && Double.TryParse(distance.Groups[1].Value,
                NumberStyles.Float, CultureInfo.InvariantCulture, out target) && Math.Abs(target - 1.0) < 0.005;
            firmwareTarget = distance.Success && Double.TryParse(distance.Groups[1].Value,
                NumberStyles.Float, CultureInfo.InvariantCulture, out target) ? target : Double.NaN;
            string phase = status.Groups[1].Value;
            phaseValue = Int32.Parse(phase, CultureInfo.InvariantCulture);
            // A terminal status queued before ARM can arrive before its ACK.
            // Resolve the pending ARM through its ACK/timeout, not that old status.
            if (!awaitingArm && (phase == "0" || phase == "3" || phase == "4" || phase == "5")) Clear();
            // A matching status can confirm GO if only its ACK was lost.
            if (awaitingGo && phase == "2" && status.Groups[2].Value == activeToken.ToString(CultureInfo.InvariantCulture) &&
                Math.Abs(firmwareTarget - selectedTarget) < 0.005 && (selectedPlan == "FWD" || observedPlan == selectedPlan)) {
                awaitingGo = false; running = true;
            }
        }
        Match ack = Regex.Match(line, @"TEST_ACK ok=(\d+) phase=(\d+) token=(\d+)");
        bool sessionAck = ack.Success && !Regex.IsMatch(line, @"\bcmd=(SPEED|ACCEL|EDGE)\b");
        if (sessionAck && awaitingArm) {
            awaitingArm = false;
            if (ack.Groups[1].Value == "1" && ack.Groups[2].Value == "1" && now - armSent <= 1000 &&
                UInt32.TryParse(ack.Groups[3].Value, out activeToken) && SelectionMatches(line)) {
                armed = true; armAck = now; report("arm_ready=1 next=CLICK_GO expires_ms=4000 plan=" + selectedPlan + " target_m=" + selectedTarget);
            } else { Clear(); report("rejected=ARM_ACK_MISMATCH_OR_NOT_READY"); }
        } else if (sessionAck && awaitingGo) {
            awaitingGo = false;
            if (ack.Groups[1].Value == "1" && ack.Groups[2].Value == "2" &&
                ack.Groups[3].Value == activeToken.ToString(CultureInfo.InvariantCulture) && now - goSent <= 500 && SelectionMatches(line)) {
                running = true;
            } else Clear();
        }
        if (line.StartsWith("FW:", StringComparison.Ordinal)) {
            double value;
            string[] fields = line.Substring(3).Split(',');
            if ((fields.Length == 4 || fields.Length == 13) && Double.TryParse(fields[0], NumberStyles.Float, CultureInfo.InvariantCulture, out value) &&
                !Double.IsNaN(value) && !Double.IsInfinity(value)) {
                if (haveYaw && marking) {
                    double delta = value - yaw;
                    while (delta > 180) delta -= 360;
                    while (delta < -180) delta += 360;
                    markedChange += delta;
                }
                yaw = value; haveYaw = true; lastYaw = now;
            }
        }
    }
    public bool Command(string packet, long now) {
        string command = packet.Trim();
        if (command.IndexOf('\n') >= 0 || command.IndexOf('\r') >= 0) { report("rejected=BATCH_PACKET"); return false; }
        if (command == "UI STOP") { tx("STOP"); Clear(); report("stop_requested=1"); return true; }
        if (Regex.IsMatch(command, @"^UI TRACE (START|STOP|CLEAR|DUMP)$")) {
            tx(command.Substring(3)); report("capture_command=" + command.Substring(9) + " motor_command=NONE device_checks_dump_stop=1"); return true;
        }
        if (command == "UI LOG QUIET" || command == "UI LOG DEFAULT" || command == "UI LOG ALL ON") {
            string preset = command == "UI LOG QUIET" ? "QUIET" : command == "UI LOG DEFAULT" ? "DEFAULT" : "ALL";
            LogPreset(preset);
            report("log_preset=" + preset + " ACK_and_snapshots=ALWAYS background_log=FULL wave=" + (WaveEnabled ? "1" : "0"));
            return true;
        }
        Match visibility = Regex.Match(command, @"^UI LOG ([A-Z0-9_]+) (ON|OFF)$");
        if (visibility.Success) {
            string category = visibility.Groups[1].Value;
            if (!logEnabled.ContainsKey(category)) { report("rejected=UNKNOWN_LOG_CATEGORY"); return false; }
            logEnabled[category] = visibility.Groups[2].Value == "ON";
            lastForward.Remove(category);
            report("log_category=" + category + " enabled=" + (logEnabled[category] ? "1" : "0") + " background_log=FULL");
            return true;
        }
        if (command.StartsWith("UI SNAPSHOT ", StringComparison.Ordinal)) {
            string category = command.Substring(12);
            string pattern;
            switch (category) {
                case "STATE": pattern = @"^TEST_(STATUS|HEALTH|CONTROL|RAMP|WALL|TIMING|PLAN)$"; break;
                case "IMU": pattern = @"^TEST_(IMU|IMU_RAW|IMU_BIAS|DRIFT)$"; break;
                case "LASER": pattern = @"^TEST_(LASER|LASER_VALID|RANGE|PLAN|BRAKE)$"; break;
                case "REMOTE": pattern = @"^TEST_REMOTE$"; break;
                case "MOTORS": pattern = @"^TEST_MOTOR_"; break;
                case "CAN": pattern = @"^TEST_CAN_"; break;
                default: report("rejected=UNKNOWN_SNAPSHOT"); return false;
            }
            if (now - lastStatus > 500) {
                report("snapshot=" + category + " ready=0 reason=UART_STATUS_STALE status_age_ms=" + (now - lastStatus));
                return true;
            }
            report("snapshot=" + category + " status_age_ms=" + (now - lastStatus) + " source=CACHED_UART");
            foreach (KeyValuePair<string, string> item in cache) if (Regex.IsMatch(item.Key, pattern)) report("cached " + item.Value);
            return true;
        }
        if (command.StartsWith("UI MARK ", StringComparison.Ordinal)) {
            string kind = command.Substring(8);
            if (kind != "STATIC_BEGIN" && kind != "ROTATE_BEGIN" && kind != "END") return false;
            if (!haveYaw || now - lastStatus > 500 || now - lastYaw > 500) { report("rejected=NO_FRESH_YAW"); return false; }
            if (kind == "END") {
                if (!marking) { report("rejected=NO_ACTIVE_RECORD"); return false; }
                report("record=" + markKind + " elapsed_ms=" + (now - markTime) +
                    " delta_yaw_deg=" + markedChange.ToString("F6", CultureInfo.InvariantCulture) + " motor_command=NONE");
                marking = false;
            } else {
                marking = true; markTime = now; markedChange = 0; markKind = kind;
                report("record=" + kind + " yaw_deg=" + yaw.ToString("F6", CultureInfo.InvariantCulture) + " motor_command=NONE");
            }
            return true;
        }
        if (command == "UI PING") {
            if (awaitingArm || awaitingGo) return false; // ACK has no command ID; keep it unambiguous.
            tx("PING"); return true;
        }
        Match setting = Regex.Match(command, @"^UI (SPEED|ACCEL) ([0-9]+(?:\.[0-9]+)?)$");
        if (setting.Success) {
            string parameter = setting.Groups[1].Value;
            string value = setting.Groups[2].Value;
            double acceleration;
            if (parameter == "SPEED" && !Regex.IsMatch(value, @"^(0\.(?:05|08|10|20|30|40|50|80)|1\.(?:00|20|50|80)|2\.00)$")) return false;
            if (parameter == "ACCEL" && (!Double.TryParse(value, NumberStyles.Float, CultureInfo.InvariantCulture, out acceleration) ||
                acceleration < 0.05 || acceleration > 0.50)) return false;
            if (!firmwareV2 || now - lastPlan > 1000 || now - lastStatus > 500 || !stopped ||
                phaseValue == 1 || phaseValue == 2 || phaseValue == 3 || awaitingArm || armed || awaitingGo || running) {
                report("rejected=" + parameter + "_NEEDS_IDLE_PROTOCOL_2"); return false;
            }
            tx(parameter + " " + value); report(parameter.ToLowerInvariant() + "_requested=" + value + " confirm_device_ACK=1");
            return true;
        }
        Match edge = Regex.Match(command, @"^UI EDGE (FRONT|RIGHT|REAR|LEFT)$");
        if (edge.Success) {
            string direction = edge.Groups[1].Value;
            if (awaitingArm || armed || awaitingGo || running || phaseValue == 1 || phaseValue == 2 || phaseValue == 3 ||
                !stopped || now - lastStatus > 500 || now - lastRange > 500 || !rangeFresh || !ranges.ContainsKey(direction)) {
                report("rejected=EDGE_NEEDS_STOPPED_AND_FRESH_RANGE"); return false;
            }
            int mm = ranges[direction];
            if (mm < 70 || mm > 1000) { report("rejected=EDGE_OUTSIDE_70_TO_1000_MM"); return false; }
            report("edge_direction=" + direction + " contact_raw_mm=" + mm + " body_pose=USER_CONFIRMED range_age_ms=" + (now - lastRange));
            if (direction == "LEFT") {
                int common = ((mm + 4) / 5) * 5;
                tx("EDGE ALL " + common); report("common_edge_requested_mm=" + common + " confirm_device_ACK=1 expires_on_reboot=1");
            }
            return true;
        }
        Match motion = Regex.Match(command, @"^UI (ARM|GO)(?: (ROUND|WALL) ([124])| (APPROACH))?$");
        if (!motion.Success) { report("rejected=UNKNOWN_COMMAND"); return false; }
        bool isArm = motion.Groups[1].Value == "ARM";
        string requestedPlan = motion.Groups[2].Success ? motion.Groups[2].Value : motion.Groups[4].Success ? "APPROACH" : "FWD";
        int requestedTarget = motion.Groups[3].Success ? Int32.Parse(motion.Groups[3].Value, CultureInfo.InvariantCulture) : 1;
        if (!allowMotion) { report("rejected=STATIC_MODE_USE_MOTION_LAUNCHER_IN_OPEN_AREA"); return false; }
        if (!firmwareOneShot || now - lastPlan > 1000) {
            report("rejected=FIRMWARE_ONE_SHOT_REQUIRED"); return false;
        }
        if (controlMode == "MANUAL" || controlMode == "STOP") {
            report("rejected=SELECT_SWC_1500_HOST mode=" + controlMode); return false;
        }
        if (requestedPlan != "FWD" && (!firmwareV2 || now - lastPlan > 1000)) {
            report("rejected=FIRMWARE_PROTOCOL_2_REQUIRED"); return false;
        }
        if (!firmwareV2 && !targetMatches) { report("rejected=FIRMWARE_TARGET_NOT_1M"); return false; }
        if (now - lastStatus > 500 || !healthy || !stopped || awaitingArm || awaitingGo || running) {
            report("rejected=NOT_READY_OR_BUSY"); return false;
        }
        if (isArm) {
            Clear(); selectedPlan = requestedPlan; selectedTarget = requestedTarget;
            awaitingArm = true; armSent = now;
            tx(selectedPlan == "FWD" ? "ARM" : "ARM " + selectedPlan +
                (selectedPlan == "APPROACH" ? "" : " " + selectedTarget));
            return true;
        }
        if (requestedPlan != selectedPlan || requestedTarget != selectedTarget || Math.Abs(firmwareTarget - selectedTarget) >= 0.005 || Double.IsNaN(firmwareTarget)) {
            report("rejected=PLAN_OR_DISTANCE_MISMATCH"); return false;
        }
        if (!armed || now - armAck > 4000) { report("rejected=CLICK_ARM_FIRST"); return false; }
        armed = false; awaitingGo = true; goSent = now;
        tx("GO " + activeToken.ToString(CultureInfo.InvariantCulture));
        return true;
    }
    public void Tick(long now) {
        if (awaitingArm && now - armSent > 1000) { Clear(); report("rejected=ARM_ACK_TIMEOUT"); }
        if (awaitingGo && now - goSent > 500) {
            Clear();
            report("go_confirmation=TIMEOUT motion_state=UNKNOWN auto_stop=0 use_RC_or_STOP_to_stop=1");
        }
    }
}
'@
[MangoStraightConsole]::Run($Port, $BaudRate, $logPath, $WavePort, $CommandPort, $AllowMotion.IsPresent, $UiOnly.IsPresent, $RunSeconds, $ExitWithVofa.IsPresent)
