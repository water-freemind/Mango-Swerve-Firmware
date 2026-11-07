#requires -PSEdition Desktop
#requires -Version 5.1
param(
    [Parameter(Mandatory = $true)][string]$Port,
    [int]$BaudRate = 921600
)
$ErrorActionPreference = 'Stop'
if ([Console]::IsInputRedirected) { throw 'Use an interactive terminal for ARM / GO / STOP.' }
New-Item -ItemType Directory -Force (Join-Path $PSScriptRoot 'build/logs') | Out-Null
$logPath = Join-Path $PSScriptRoot ('build/logs/straight-console-' + (Get-Date -Format 'yyyyMMdd-HHmmss') + '.log')
Add-Type -TypeDefinition @'
using System;
using System.Diagnostics;
using System.IO;
using System.IO.Ports;
using System.Text;
using System.Text.RegularExpressions;
using System.Threading;
public static class MangoStraightConsole {
    public static void Run(string port, int baud, string logPath) {
        using (SerialPort serial = new SerialPort(port, baud, Parity.None, 8, StopBits.One))
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
            Console.WriteLine("No auto-start. Commands: PING, ARM, GO <token>, STOP, QUIT. HB every 200ms after GO. Escape = STOP.");
            try {
                while (true) {
                    string rx = serial.ReadExisting();
                    if (rx.Length != 0) {
                        Console.Write(rx); log.Write(rx); pending.Append(rx);
                        string text = pending.ToString(); int end;
                        while ((end = text.IndexOf('\n')) >= 0) {
                            string line = text.Substring(0, end); text = text.Substring(end + 1);
                            Match m = Regex.Match(line, @"TEST_ACK ok=1 phase=1 token=(\d+)");
                            if (m.Success) UInt32.TryParse(m.Groups[1].Value, out token);
                            m = Regex.Match(line, @"TEST_STATUS phase=(\d+)");
                            if (m.Success && (m.Groups[1].Value == "3" ||
                                m.Groups[1].Value == "4" || m.Groups[1].Value == "5")) heartbeat = null;
                        }
                        pending.Clear(); pending.Append(text.Length > 2048 ? "" : text);
                    }
                    if (Console.KeyAvailable) {
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
'@
[MangoStraightConsole]::Run($Port, $BaudRate, $logPath)
