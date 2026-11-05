param(
    [string]$Port = 'COM12',
    [int]$Seconds = 15,
    [int]$BaudRate = 921600
)

$ErrorActionPreference = 'Stop'
if ($Seconds -le 0) { throw 'Seconds must be positive' }
$logDir = Join-Path $PSScriptRoot 'build\logs'
New-Item -ItemType Directory -Force $logDir | Out-Null
$logPath = Join-Path $logDir ('uart-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff') + '.log')
$serial = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, None, 8, One
$serial.Handshake = 'None'
$writer = $null
try {
    $serial.Open()
    $serial.DiscardInBuffer()
    $writer = New-Object System.IO.StreamWriter $logPath, $false, ([Text.UTF8Encoding]::new($false))
    $timer = [Diagnostics.Stopwatch]::StartNew()
    while ($timer.Elapsed.TotalSeconds -lt $Seconds) {
        $text = $serial.ReadExisting()
        if ($text.Length -gt 0) {
            [Console]::Write($text)
            $writer.Write($text)
            $writer.Flush()
        }
        Start-Sleep -Milliseconds 50
    }
    Write-Host "`nLOG_SAVED $logPath"
} finally {
    if ($writer) { $writer.Dispose() }
    if ($serial.IsOpen) { $serial.Close() }
    $serial.Dispose()
}
