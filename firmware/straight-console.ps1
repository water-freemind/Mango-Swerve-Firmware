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
    private static string Field(string line,string name) {
        Match m=Regex.Match(line,@"(?:^|\s)"+Regex.Escape(name)+@"=([^\s]+)");
        return m.Success?m.Groups[1].Value:"?";
    }
    public static string ChineseStatus(string raw) {
        string line=Regex.Replace(raw,@"\x1b\[[0-9;]*m","").TrimEnd('\r');
        var reasons=new Dictionary<string,string> {
            {"ORIGIN_OUTSIDE_ENTRY_ZONE","起点不在入口区域；请回到入口并让前向激光对准远端围栏（约4.56至5.36米）"},
            {"ORIGIN_FRONT_NOT_READY","前向激光尚未就绪或数据过期，请检查前方测距"},
            {"ORIGIN_NEEDS_STOPPED_FEEDBACK","请先停车，等待底盘确认停稳后再登记起点"},
            {"ORIGIN_FRONT_ANCHORED","入口起点登记成功"},
            {"ARM_NOT_READY","尚未满足准备条件；请先登记起点，检查SWC、SWD及停稳状态"},
            {"ARMED","全图已准备，请点击开始"},{"RUNNING","任务运行中"},
            {"WORK_PEER_NOT_READY","疏果端未就绪，请先确认四路手动收回零位，再检查视觉在线"},
            {"WORK_ATTEMPTS_EXHAUSTED","当前树同一区域已作业两次，复检仍有目标，保持停车"},
            {"WORK_TREE_TIMEOUT","当前树作业超时，保持停车"},
            {"WORK_VISION_OFFLINE","疏果视觉连续离线，保持停车"},
            {"WORK_ALIGNMENT_LOST","疏果复检发现树位偏离对中区间，保持停车"},
            {"WORK_CAN_TIMEOUT","疏果端CAN通信超时，保持停车"},
            {"WORK_ENCODER_OR_FEEDBACK","肘击编码器或底盘停稳反馈异常，保持停车"},
            {"WORK_STOP_FEEDBACK","底盘作业停稳反馈异常，保持停车"},
            {"WORK_ENCODER_DIRECTION","肘击编码器方向异常，检查故障通道与行程读数"},
            {"WORK_ENCODER_LIMIT","肘击编码器超出安全行程，保持停车"},
            {"WORK_ENCODER_STALL","肘击动作期间编码器连续300毫秒无进展，保持停车"},
            {"WORK_ENCODER_TIMEOUT","肘击动作超过6秒，保持停车"},
            {"WORK_PEER_FAULT","疏果端故障，请查看作业结果码"},
            {"VRB_RELEASED","VRB尚未压到使能位置，请保持VRB≤1250"},
            {"SWD_REARM","请松杆回中，将SWD拨到1000再拨回2000"},
            {"STICKS_NOT_CENTERED","请先松杆回中，再手动接管"},
            {"WHEELS_MOVING","正在等待车辆停稳后手动接管"},
            {"REMOTE_PERMIT","请检查SWA=1000、SWD=2000、VRA>1200"},
            {"HARDWARE","底盘电机或IMU反馈尚未就绪"},
            {"RANGE_PAUSE","测距保护暂停，正在等待有效数据和停稳恢复"},
            {"NEAR_RECOVERY_WAIT","近距恢复：保持停止，等待安全退让条件"},
            {"NEAR_RECOVERY_ALIGN","近距恢复：零驱动对齐退让方向"},
            {"NEAR_RECOVERY_MOVE","近距恢复：限速远离障碍"},
            {"NEAR_RECOVERY_CLEAR","近距解除，等待停稳与独立测距确认"},
            {"NEAR_RECOVERY_TIMEOUT","近距纠偏超时，任务已停止"},
            {"NEAR_RECOVERY_TRAVEL_LIMIT","近距退让已达位移上限，任务已停止"},
            {"RANGE_PAUSE_TIMEOUT","测距保护未能在时限内恢复，任务已停止"},
            {"POSE_OBSERVATION_PAUSE","定位观测不足，暂停等待有效观测"},
            {"ENTRY_NORTH_PAUSE","即将到G7，等待远端围栏定位确认"},
            {"RESUMED","保护条件恢复，继续行驶"},{"ROUTE_COMPLETE","全图路线已完成"},
            {"MAP_ENVELOPE","估计车体超出地图通行包络，任务已停止"},
            {"MOTION_FAULT","运动控制检测到偏差或过点异常，任务已停止"},
            {"MODE_NAV_SELECTED","已选择单独跑图模式"},{"MODE_WORK_SELECTED","已选择联合疏果模式"},
            {"MODE_NEEDS_IDLE_STOPPED","请先结束当前任务并停稳，再切换模式"},
            {"SPEED_SET","行驶速度已设置"},{"APPROACH_SPEED_SET","接近速度已设置"},
            {"NEED_ORIGIN","请回入口对准后登记起点"},
            {"CONTROL_MODE_CHANGED","控制来源已切换，原任务结束"},
            {"RC_OFFLINE","遥控未在线，请检查遥控器和接收机"},
            {"RC_SWITCH","遥控开关未放行，请检查SWC=1500、SWD=2000并保持摇杆回中"},
            {"HOST_NOT_SELECTED","当前未选择上位机控制，SWC应在中位1500"},
            {"NONE","正常"},{"IDLE","空闲"},
            {"CLICK_MAP_ARM_FIRST_OR_NOT_READY","请先点击准备全图并等待准备成功，再点击开始"},
            {"MAP_ARM_ACK","车端拒绝准备，请查看上一条全图拒绝原因"},
            {"FIRMWARE_FULL_MAP_REQUIRED_OR_STALE","未收到新鲜全图状态，请检查底盘串口桥接"},
            {"UART_OFFLINE","底盘串口未连接，正在重试"},
            {"NO_VALID_STATUS_5S","连续5秒没有有效底盘状态，正在重连串口"},
            {"SELECT_SWC_1500_HOST","请选择上位机来源：SWC中位1500"}
        };
        string reason=Field(line,"reason"),description;
        if(line.Contains("TEST_MAP_ACK ") && reason!="?") {
            int command=line.IndexOf("cmd=",StringComparison.Ordinal);
            string cmd=command>=0?line.Substring(command+4):"?";
            if(!reasons.TryGetValue(reason,out description)) description=reason;
            return "[全图] "+(Field(line,"ok")=="1"?"成功":"拒绝")+"："+cmd+"；"+description+" ["+reason+"]";
        }
        if(line.Contains("TEST_MAP_EVENT ") && reason!="?") {
            if(!reasons.TryGetValue(reason,out description)) description=reason;
            return "[全图事件] 段="+Field(line,"segment")+"；"+description+" ["+reason+"]";
        }
        if(line.Contains("TEST_MAP_STATUS ") && Field(line,"phase")!="?") {
            var phase=new Dictionary<string,string>{{"IDLE","空闲"},{"READY","已登记起点"},{"ARMED","已准备"},{"RUNNING","运行中"},{"PAUSED","暂停"},{"DONE","完成"},{"FAULT","停止"}};
            string state;if(!phase.TryGetValue(Field(line,"phase"),out state)) state=Field(line,"phase");
            if(!reasons.TryGetValue(reason,out description)) description=reason;
            return "[全图状态] "+state+"；模式="+Field(line,"mode")+"；段="+Field(line,"segment")+
                "；位置="+Field(line,"x")+","+Field(line,"y")+"；速度档="+Field(line,"speed")+
                "；接近档="+Field(line,"approach")+"；"+description;
        }
        if(line.Contains("TEST_STATUS ") && Field(line,"stopped")!="?") {
            string blocked=Field(line,"blocked");
            if(!reasons.TryGetValue(blocked,out description)) description=blocked;
            return "[底盘状态] 来源="+Field(line,"control")+"；停稳="+(Field(line,"stopped")=="1"?"是":"否")+
                "；许可="+(Field(line,"healthy")=="1"?"正常":"未满足")+"；"+description;
        }
        if(line.Contains("TEST_HEALTH ") && Field(line,"first")!="?") {
            string first=Field(line,"first");if(!reasons.TryGetValue(first,out description)) description=first;
            return "[健康检查] "+description+"；阻塞位="+Field(line,"blocked")+
                "；IMU="+Field(line,"imu_ok")+"；串口故障="+Field(line,"uart_fault");
        }
        if(line.Contains("TEST_REMOTE ") && Field(line,"online")!="?")
            return "[遥控] "+(Field(line,"online")=="1"?"在线":"离线")+"；SWC="+Field(line,"SWC")+
                "；SWD="+Field(line,"SWD")+"；VRB="+Field(line,"VRB")+"；摇杆回中="+Field(line,"centered");
        if(line.Contains("TEST_MAP_POSE ") && Field(line,"motion_phase")!="?")
            return "[运动指令] 前后="+Field(line,"vx")+"；横向="+Field(line,"vy")+"；旋转="+Field(line,"wz")+
                "；阶段="+Field(line,"motion_phase")+"；预测位="+Field(line,"predicted");
        if(line.Contains("TEST_CONTROL ") && Field(line,"source")=="MANUAL") {
            if(Field(line,"manual_armed")=="1") return "[遥控] 手动已使能";
            string wait=Field(line,"manual_wait");
            if(reasons.TryGetValue(wait,out description)) return "[遥控] "+description+" ["+wait+"]";
            return "[遥控] 手动待使能，VRB="+Field(line,"vrb_ok")+"，SWD确认="+Field(line,"release_seen");
        }
        if(line.Contains("TEST_MAP_AISLE ") && Field(line,"active")!="?")
            return "[纠偏] 前后配对="+Field(line,"active")+"；误差毫米="+Field(line,"error_mm")+
                "；轮向到位="+Field(line,"align")+"；速度上限="+Field(line,"limit")+
                (Field(line,"wall_mm")!="?"?"；边墙误差毫米="+Field(line,"wall_mm"):"");
        if(line.Contains("TEST_MAP_GUARD ") && Field(line,"near")!="?")
            return "[保护诊断] 近距位="+Field(line,"near")+"；制动位="+Field(line,"brake")+"；过期位="+Field(line,"stale")+
                "；四向保护毫米="+Field(line,"budget_mm")+(Field(line,"reject")!="?"?"；突变拒绝位="+Field(line,"reject"):"");
        if(line.Contains("TEST_PANEL ")) {
            string rejected=Field(line,"rejected");
            if(reasons.TryGetValue(rejected,out description)) return "[面板] "+description+" ["+rejected+"]";
            if(line.Contains("uart_online=0")) return "[串口] 数据中断，正在自动重连；原因="+reason;
            if(line.Contains("uart_open=1")) return "[串口] 已打开，等待底盘有效状态";
            if(line.Contains("log_category=WAVE")) return "[波形] "+(Field(line,"enabled")=="1"?"已开启":"已关闭");
        }
        return line;
    }
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
        if (!Regex.IsMatch(plain, @"^\[(INFO|WARN|ERROR)\] TEST_(STATUS|ACK|READY|INIT_FAILED|HEALTH|CONTROL|RAMP|REMOTE|IMU|IMU_RAW|IMU_BIAS|MOTOR|CAN|LASER|LASER_VALID|WALL|PAUSE|PAUSE_RANGE|FAULT|FAULT_RANGE|TIMING|DRIFT|PLAN|RANGE|BRAKE|PANEL|TRACE|TRACE_META|TRACE_STATE|TRACE_END|MAP_[A-Z_]+|WORK|WORK_EVENT)\b") ||
            plain.IndexOf(':') >= 0) return false;
        byte[] bytes = Encoding.UTF8.GetBytes(ChineseStatus(plain) + "\n");
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
        using (StreamWriter log = new StreamWriter(logPath, false, Encoding.UTF8))
        using (StreamWriter chineseLog = new StreamWriter(Path.Combine(Path.GetDirectoryName(logPath),
            Path.GetFileName(logPath).Replace("straight-console-","vofa-zh-")),false,new UTF8Encoding(true))) {
            Console.OutputEncoding = new UTF8Encoding(false);
            // Windows reports ICMP port-unreachable as a socket reset when
            // VOFA disconnects. Keep the UDP listener alive for its reconnect.
            wave.Client.IOControl((IOControlCode)unchecked((int)0x9800000C), new byte[4], null);
            serial.Handshake = Handshake.None;
            serial.DtrEnable = false; serial.RtsEnable = false;
            serial.WriteTimeout = 100; serial.ReadTimeout = 100;
            serial.NewLine = "\n";
            log.AutoFlush = true;
            chineseLog.AutoFlush = true;
            uint token = 0;
            StringBuilder input = new StringBuilder();
            StringBuilder pending = new StringBuilder();
            Stopwatch clock = Stopwatch.StartNew();
            long nextAppCheck = 0;
            Action<string> panelLog = delegate(string text) {
                string message = "[INFO] TEST_PANEL " + text;
                string translated=ChineseStatus(message);
                Console.WriteLine(translated); log.WriteLine(message);chineseLog.WriteLine(translated);
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
                        log.Write(rx); pending.Append(rx);
                        string text = pending.ToString(); int end;
                        while ((end = text.IndexOf('\n')) >= 0) {
                            string line = text.Substring(0, end); text = text.Substring(end + 1);
                            if(!line.StartsWith("FW:",StringComparison.Ordinal)) Console.WriteLine(ChineseStatus(line));
                            if (Regex.IsMatch(line, @"TEST_STATUS phase=\d+ token=\d+ healthy=[01] stopped=[01]"))
                                lastProtocol = clock.ElapsedMilliseconds;
                            panel.Observe(line, clock.ElapsedMilliseconds);
                            try {
                                if (line.StartsWith("FW:", StringComparison.Ordinal)) {
                                    if (panel.WaveEnabled) ForwardWave(wave, wavePort, line);
                                } else if (panel.ShouldForwardStatus(line, clock.ElapsedMilliseconds)) {
                                    chineseLog.WriteLine(ChineseStatus(line));ForwardStatus(wave, wavePort, line);
                                }
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
    private bool mapFirmware, mapAwaitingArm, mapArmed;
    private uint mapToken;
    private long lastMap = -10000, mapArmTime;
    private string mapPhase = "UNKNOWN";
    private string mapArmSelection;
    private string controlMode = "UNKNOWN";
    private long lastPlan = -10000, lastRange = -10000;
    private double firmwareTarget;
    private string selectedPlan = "FWD", observedPlan = "FWD";
    private int selectedTarget = 1, phaseValue;
    private readonly Dictionary<string, int> ranges = new Dictionary<string, int>();
    private static readonly string[] logCategories = {"STATUS","HEALTH","REMOTE","MOTOR","CAN1","CAN2","CAN3",
        "IMU","IMU_RAW","IMU_BIAS","DRIFT","LASER","RANGE","PLAN","BRAKE","TIMING","WAVE","MAP","WORK"};
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
        if (compactLogs) logEnabled["MAP"] = true;
        if (compactLogs) logEnabled["WORK"] = true;
        lastForward.Clear();
    }
    public bool ShouldForwardStatus(string raw, long now) {
        string line = Regex.Replace(raw, @"\x1b\[[0-9;]*m", "");
        Match tag = Regex.Match(line, @"^\[(INFO|WARN|ERROR)\] TEST_([A-Z_]+)\b");
        if (!tag.Success) return false;
        string category = tag.Groups[2].Value;
        if (category == "WORK" || category == "WORK_EVENT") return true;
        if (category == "MAP_ACK" || category == "MAP_EVENT" || category == "MAP_SNAPSHOT" || category == "MAP_TREE_EVENT" || category == "MAP_CONTROL_FAULT" || category == "PREDICT") return true;
        if (category == "MAP_STATUS" || category == "MAP_POSE" || category == "MAP_AISLE" || category == "MAP_TREE") category = "MAP";
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
        mapFirmware = mapAwaitingArm = mapArmed = false; mapToken = 0; lastMap = -10000; mapPhase = "UNKNOWN";
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
        Match mapState = Regex.Match(line, @"TEST_MAP_STATUS proto=1 phase=([A-Z_]+)\b");
        if (mapState.Success) { mapFirmware = true; lastMap = now; mapPhase = mapState.Groups[1].Value; }
        Match mapAck = Regex.Match(line, @"TEST_MAP_ACK ok=(\d) phase=([A-Z_]+) token=(\d+)\b");
        if (mapAwaitingArm && mapAck.Success && line.EndsWith("cmd=" + mapArmSelection, StringComparison.Ordinal)) {
            mapAwaitingArm = false;
            mapArmed = mapAck.Groups[1].Value == "1" && mapAck.Groups[2].Value == "ARMED" &&
                now - mapArmTime <= 1000 && UInt32.TryParse(mapAck.Groups[3].Value, out mapToken);
            mapArmTime = now;
            report(mapArmed ? "map_arm_ready=1 next=UI_MAP_GO expires_ms=30000" : "rejected=MAP_ARM_ACK");
        }
        Match planStatus = Regex.Match(line, @"TEST_PLAN proto=2 plan=(FWD|ROUND|WALL|APPROACH)\b");
        if (planStatus.Success) {
            firmwareV2 = true; lastPlan = now; observedPlan = planStatus.Groups[1].Value;
            firmwareOneShot = Regex.IsMatch(line, @"\bhb_required=0\b");
        }
        Match owner = Regex.Match(line, @"(?:TEST_CONTROL[^\r\n]*\bsource=|\bcontrol=)(HOST|MANUAL|STOP)\b");
        if (owner.Success) {
            controlMode = owner.Groups[1].Value;
            if (controlMode != "HOST") { Clear(); mapArmed = mapAwaitingArm = false; }
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
        if (command == "UI STOP") { tx("STOP"); Clear(); mapArmed = mapAwaitingArm = false; report("stop_requested=1"); return true; }
        if (command.StartsWith("UI MAP ", StringComparison.Ordinal)) {
            string action = command.Substring(7);
            if (action == "ELBOW HOME") {
                if (!stopped || now - lastStatus > 500) { report("rejected=ORIGIN_NEEDS_STOPPED_FEEDBACK"); return false; }
                tx("MAP ELBOW HOME"); return true;
            }
            if (action == "STATUS" || action == "STOP" || action == "WORK STATUS" || action == "WORK RESET") {
                tx("MAP " + action); if (action == "STOP") mapArmed = mapAwaitingArm = false;
                if (action == "WORK RESET") mapArmed = mapAwaitingArm = false;
                return true;
            }
            if (!mapFirmware || now - lastMap > 1500) { report("rejected=FIRMWARE_FULL_MAP_REQUIRED_OR_STALE"); return false; }
            if (action == "PAUSE") { tx("MAP PAUSE"); return true; }
            if (!allowMotion) { report("rejected=STATIC_MODE_USE_MOTION_LAUNCHER_IN_OPEN_AREA"); return false; }
            if (controlMode != "HOST" || now - lastStatus > 500 || awaitingArm || armed || awaitingGo || running) {
                report("rejected=MAP_NEEDS_HOST_AND_NO_STRAIGHT_SESSION"); return false;
            }
            if (action == "GO") {
                if (!mapArmed || now - mapArmTime > 30000 || !healthy || !stopped) { report("rejected=CLICK_MAP_ARM_FIRST_OR_NOT_READY"); return false; }
                tx("MAP GO " + mapToken.ToString(CultureInfo.InvariantCulture)); mapArmed = false; return true;
            }
            if (action == "RESUME") { tx("MAP RESUME"); return true; }
            if (mapPhase == "RUNNING" || mapPhase == "PAUSED" || mapPhase == "ARMED" || !stopped) {
                report("rejected=MAP_NEEDS_IDLE_STOPPED"); return false;
            }
            if (action == "ORIGIN" || Regex.IsMatch(action, @"^SPEED 0\.(50|60|70|80|90)$") ||
                Regex.IsMatch(action, @"^APPROACH 0\.(15|20|25|30|35)$")) { tx("MAP " + action); return true; }
            if (action == "MODE NAV" || action == "MODE WORK") {
                if (mapArmed || mapAwaitingArm) { report("rejected=MAP_NEEDS_IDLE_STOPPED"); return false; }
                mapArmed = mapAwaitingArm = false;
                tx("MAP " + action); return true;
            }
            if (action == "ARM ALL") {
                if (!healthy) {
                    string status;
                    cache.TryGetValue("TEST_STATUS", out status);
                    Match blocked = Regex.Match(status ?? "", @"\bblocked=([^\s]+)");
                    string reason = blocked.Success && blocked.Groups[1].Value != "NONE" ? blocked.Groups[1].Value : "ARM_NOT_READY";
                    report("rejected=" + reason + " action=MAP_ARM_ALL");
                    return false;
                }
                if (mapAwaitingArm) { report("rejected=MAP_ARM_ACK_PENDING"); return false; }
                mapAwaitingArm = true; mapArmTime = now; mapArmSelection = action; tx("MAP " + action); return true;
            }
            report("rejected=UNKNOWN_MAP_COMMAND"); return false;
        }
        if ((mapPhase == "RUNNING" || mapPhase == "PAUSED" || mapPhase == "ARMED" || mapArmed || mapAwaitingArm) &&
            Regex.IsMatch(command, @"^UI (ARM|GO|SPEED|ACCEL|EDGE)\b")) {
            report("rejected=FULL_MAP_ACTIVE"); return false;
        }
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
        if (mapAwaitingArm && now - mapArmTime > 1000) {
            mapAwaitingArm = mapArmed = false; report("rejected=MAP_ARM_ACK_TIMEOUT");
        }
        if (awaitingArm && now - armSent > 1000) { Clear(); report("rejected=ARM_ACK_TIMEOUT"); }
        if (awaitingGo && now - goSent > 500) {
            Clear();
            report("go_confirmation=TIMEOUT motion_state=UNKNOWN auto_stop=0 use_RC_or_STOP_to_stop=1");
        }
    }
}
'@
[MangoStraightConsole]::Run($Port, $BaudRate, $logPath, $WavePort, $CommandPort, $AllowMotion.IsPresent, $UiOnly.IsPresent, $RunSeconds, $ExitWithVofa.IsPresent)
