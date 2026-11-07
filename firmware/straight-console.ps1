#requires -PSEdition Desktop
#requires -Version 5.1
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$BaudRate = 921600,
    [ValidateRange(0,65535)][int]$WavePort = 0,
    [ValidateRange(0,65535)][int]$CommandPort = 0,
    [switch]$AllowMotion,
    [switch]$UiOnly,
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
        if (fields.Length != 4) return false;
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
        if (!Regex.IsMatch(plain, @"^\[(INFO|WARN|ERROR)\] TEST_(STATUS|ACK|READY|INIT_FAILED|HEALTH|REMOTE|IMU|IMU_RAW|IMU_BIAS|MOTOR|CAN|LASER|TIMING|DRIFT|PANEL)\b") ||
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
    public static void Run(string port, int baud, string logPath, int wavePort, int commandPort, bool allowMotion, bool uiOnly, int runSeconds) {
        using (SerialPort serial = new SerialPort(port, baud, Parity.None, 8, StopBits.One))
        using (UdpClient wave = commandPort == 0 ? new UdpClient() :
            new UdpClient(new IPEndPoint(IPAddress.Loopback, commandPort)))
        using (StreamWriter log = new StreamWriter(logPath, false, Encoding.UTF8)) {
            serial.Handshake = Handshake.None;
            serial.DtrEnable = false; serial.RtsEnable = false;
            serial.WriteTimeout = 100; serial.ReadTimeout = 100;
            serial.NewLine = "\n";
            serial.Open(); log.AutoFlush = true;
            uint token = 0; string heartbeat = null;
            StringBuilder input = new StringBuilder();
            StringBuilder pending = new StringBuilder();
            Stopwatch clock = Stopwatch.StartNew(); long nextHb = 0;
            Action<string> panelLog = delegate(string text) {
                string message = "[INFO] TEST_PANEL " + text;
                Console.WriteLine(message); log.WriteLine(message);
                try { ForwardStatus(wave, wavePort, message); } catch (SocketException) { }
            };
            VofaTestSession panel = new VofaTestSession(allowMotion,
                delegate(string command) { serial.WriteLine(command); log.WriteLine("HOST_TX " + command); }, panelLog);
            if (commandPort != 0) panelLog("ready=1 command_port=" + commandPort + " wave_port=" + wavePort +
                " motion_allowed=" + (allowMotion ? "1" : "0") + " auto_start=0");
            Console.WriteLine("No auto-start. Commands: PING, ARM, GO <token>, STOP, QUIT. HB every 200ms after GO. Escape = STOP.");
            try {
                while (true) {
                    if (runSeconds != 0 && clock.ElapsedMilliseconds >= runSeconds * 1000L) break;
                    string rx = serial.ReadExisting();
                    if (rx.Length != 0) {
                        Console.Write(rx); log.Write(rx); pending.Append(rx);
                        string text = pending.ToString(); int end;
                        while ((end = text.IndexOf('\n')) >= 0) {
                            string line = text.Substring(0, end); text = text.Substring(end + 1);
                            panel.Observe(line, clock.ElapsedMilliseconds);
                            try {
                                if (!ForwardWave(wave, wavePort, line)) ForwardStatus(wave, wavePort, line);
                            }
                            catch (SocketException ex) { log.WriteLine("WAVE_FORWARD_ERROR " + ex.Message); }
                            Match m = Regex.Match(line, @"TEST_ACK ok=1 phase=1 token=(\d+)");
                            if (m.Success) UInt32.TryParse(m.Groups[1].Value, out token);
                            m = Regex.Match(line, @"TEST_STATUS phase=(\d+)");
                            if (m.Success && (m.Groups[1].Value == "3" ||
                                m.Groups[1].Value == "4" || m.Groups[1].Value == "5")) heartbeat = null;
                        }
                        pending.Clear(); pending.Append(text.Length > 2048 ? "" : text);
                    }
                    if (commandPort != 0) {
                        ReceivePanelCommands(wave, panel, clock.ElapsedMilliseconds);
                        panel.Tick(clock.ElapsedMilliseconds);
                    }
                    if (!uiOnly && Console.KeyAvailable) {
                        ConsoleKeyInfo key = Console.ReadKey(true);
                        if (key.Key == ConsoleKey.Escape) { serial.WriteLine("STOP"); heartbeat = null; input.Clear(); }
                        else if (key.Key == ConsoleKey.Enter) {
                            string command = input.ToString().Trim(); input.Clear(); Console.WriteLine();
                            if (command.Equals("QUIT", StringComparison.OrdinalIgnoreCase)) break;
                            if (command.Equals("GO", StringComparison.OrdinalIgnoreCase)) command = "GO " + token;
                            if (command == "STOP") heartbeat = null;
                            if (Regex.IsMatch(command, @"^GO [0-9]+$")) {
                                heartbeat = "HB " + command.Substring(3); nextHb = clock.ElapsedMilliseconds;
                            }
                            if (command.Length != 0) { serial.WriteLine(command); log.WriteLine("HOST_TX " + command); }
                        } else if (key.Key == ConsoleKey.Backspace) {
                            if (input.Length > 0) { input.Length--; Console.Write("\b \b"); }
                        } else if (!Char.IsControl(key.KeyChar)) { input.Append(key.KeyChar); Console.Write(key.KeyChar); }
                    }
                    if (heartbeat != null && clock.ElapsedMilliseconds >= nextHb) {
                        serial.WriteLine(heartbeat); nextHb = clock.ElapsedMilliseconds + 200;
                    }
                    Thread.Sleep(5);
                }
            } finally {
                try { serial.WriteLine("STOP"); } catch { }
                serial.Close();
            }
        }
    }
}
public sealed class VofaTestSession {
    private readonly bool allowMotion;
    private readonly Action<string> tx, report;
    private readonly Dictionary<string, string> cache = new Dictionary<string, string>();
    private long lastStatus = -10000, armSent, armAck, goSent, nextHb;
    private bool healthy, stopped, awaitingArm, armed, awaitingGo, running, targetMatches;
    private uint activeToken;
    private bool haveYaw, marking;
    private double yaw, markedChange;
    private long markTime, lastYaw = -10000;
    private string markKind;
    public VofaTestSession(bool motion, Action<string> send, Action<string> emit) {
        allowMotion = motion; tx = send; report = emit;
    }
    private void Clear() { awaitingArm = armed = awaitingGo = running = false; activeToken = 0; }
    public void Observe(string raw, long now) {
        string line = Regex.Replace(raw, @"\x1b\[[0-9;]*m", "").TrimEnd('\r');
        Match tag = Regex.Match(line, @"^\[(?:INFO|WARN|ERROR)\] (TEST_[A-Z_]+)\b");
        if (tag.Success) {
            string key = tag.Groups[1].Value;
            Match id = Regex.Match(line, @"\b(?:id|bus)=(\d+)");
            if (id.Success) key += "_" + id.Groups[1].Value;
            cache[key] = line;
        }
        if (line.Contains("TEST_READY")) { Clear(); lastStatus = -10000; haveYaw = marking = false; cache.Clear(); }
        Match status = Regex.Match(line, @"TEST_STATUS phase=(\d+) token=(\d+) healthy=(\d+) stopped=(\d+)");
        if (status.Success) {
            lastStatus = now;
            healthy = status.Groups[3].Value == "1"; stopped = status.Groups[4].Value == "1";
            Match distance = Regex.Match(line, @"\btarget_m=([0-9.]+)\b");
            double target;
            targetMatches = distance.Success && Double.TryParse(distance.Groups[1].Value,
                NumberStyles.Float, CultureInfo.InvariantCulture, out target) && Math.Abs(target - 1.0) < 0.005;
            string phase = status.Groups[1].Value;
            if (phase == "3" || phase == "4" || phase == "5" || (phase == "0" && !awaitingArm)) Clear();
            // A matching status can confirm GO if only its ACK was lost.
            if (awaitingGo && phase == "2" && status.Groups[2].Value == activeToken.ToString(CultureInfo.InvariantCulture)) {
                awaitingGo = false; running = true; nextHb = now;
            }
        }
        Match ack = Regex.Match(line, @"TEST_ACK ok=(\d+) phase=(\d+) token=(\d+)");
        if (ack.Success && awaitingArm) {
            awaitingArm = false;
            if (ack.Groups[1].Value == "1" && ack.Groups[2].Value == "1" && now - armSent <= 1000 &&
                UInt32.TryParse(ack.Groups[3].Value, out activeToken)) {
                armed = true; armAck = now; report("arm_ready=1 next=CLICK_GO expires_ms=4000");
            } else Clear();
        } else if (ack.Success && awaitingGo) {
            awaitingGo = false;
            if (ack.Groups[1].Value == "1" && ack.Groups[2].Value == "2" &&
                ack.Groups[3].Value == activeToken.ToString(CultureInfo.InvariantCulture) && now - goSent <= 500) {
                running = true; nextHb = now;
            } else { tx("STOP"); Clear(); }
        }
        if (line.StartsWith("FW:", StringComparison.Ordinal)) {
            double value;
            string[] fields = line.Substring(3).Split(',');
            if (fields.Length == 4 && Double.TryParse(fields[0], NumberStyles.Float, CultureInfo.InvariantCulture, out value) &&
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
        if (command.StartsWith("UI SNAPSHOT ", StringComparison.Ordinal)) {
            string category = command.Substring(12);
            string pattern;
            switch (category) {
                case "STATE": pattern = @"^TEST_(STATUS|HEALTH|TIMING)$"; break;
                case "IMU": pattern = @"^TEST_(IMU|IMU_RAW|IMU_BIAS|DRIFT)$"; break;
                case "LASER": pattern = @"^TEST_LASER$"; break;
                case "REMOTE": pattern = @"^TEST_REMOTE$"; break;
                case "MOTORS": pattern = @"^TEST_MOTOR_"; break;
                case "CAN": pattern = @"^TEST_CAN_"; break;
                default: report("rejected=UNKNOWN_SNAPSHOT"); return false;
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
        if (command != "UI ARM" && command != "UI GO") { report("rejected=UNKNOWN_COMMAND"); return false; }
        if (!allowMotion) { report("rejected=STATIC_MODE_USE_MOTION_LAUNCHER_IN_OPEN_AREA"); return false; }
        if (!targetMatches) { report("rejected=FIRMWARE_TARGET_NOT_1M"); return false; }
        if (now - lastStatus > 500 || !healthy || !stopped || awaitingArm || awaitingGo || running) {
            report("rejected=NOT_READY_OR_BUSY"); return false;
        }
        if (command == "UI ARM") { Clear(); awaitingArm = true; armSent = now; tx("ARM"); return true; }
        if (!armed || now - armAck > 4000) { report("rejected=CLICK_ARM_FIRST"); return false; }
        armed = false; awaitingGo = true; goSent = now;
        tx("GO " + activeToken.ToString(CultureInfo.InvariantCulture));
        return true;
    }
    public void Tick(long now) {
        if (awaitingArm && now - armSent > 1000) { Clear(); report("rejected=ARM_ACK_TIMEOUT"); }
        if ((awaitingGo && now - goSent > 500) || (running && now - lastStatus > 400)) {
            tx("STOP"); Clear(); report("stop_requested=STATUS_OR_GO_ACK_TIMEOUT");
        }
        if (running && now >= nextHb) { tx("HB " + activeToken.ToString(CultureInfo.InvariantCulture)); nextHb = now + 200; }
    }
}
'@
[MangoStraightConsole]::Run($Port, $BaudRate, $logPath, $WavePort, $CommandPort, $AllowMotion.IsPresent, $UiOnly.IsPresent, $RunSeconds)
