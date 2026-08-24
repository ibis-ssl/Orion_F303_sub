param(
  [Parameter(Mandatory = $true)]
  [string]$Port,

  [ValidateRange(1, 10000000)]
  [int]$BaudRate = 2000000,

  [ValidateRange(0, 2147483647)]
  [int]$DurationSec = 0,

  [ValidateRange(0, 2147483647)]
  [int]$MaxLines = 0,

  [string]$LogPath = ""
)

$ErrorActionPreference = "Stop"

$serialPort = New-Object System.IO.Ports.SerialPort $Port, $BaudRate, ([System.IO.Ports.Parity]::None), 8, ([System.IO.Ports.StopBits]::One)
$serialPort.ReadTimeout = 500
$serialPort.NewLine = "`n"

$writer = $null
if ($LogPath -ne "") {
  $resolvedLogPath = $ExecutionContext.SessionState.Path.GetUnresolvedProviderPathFromPSPath($LogPath)
  $logDirectory = Split-Path -Parent $resolvedLogPath
  if ($logDirectory -and -not (Test-Path -LiteralPath $logDirectory)) {
    New-Item -ItemType Directory -Path $logDirectory | Out-Null
  }
  $writer = New-Object System.IO.StreamWriter($resolvedLogPath, $false, (New-Object System.Text.UTF8Encoding($false)))
  $writer.AutoFlush = $true
}

try {
  $serialPort.Open()
  $stopwatch = [System.Diagnostics.Stopwatch]::StartNew()
  $lineCount = 0

  while ($true) {
    if ($DurationSec -gt 0 -and $stopwatch.Elapsed.TotalSeconds -ge $DurationSec) {
      break
    }

    if ($MaxLines -gt 0 -and $lineCount -ge $MaxLines) {
      break
    }

    try {
      $line = $serialPort.ReadLine().TrimEnd("`r", "`n")
      if ([string]::IsNullOrWhiteSpace($line)) {
        continue
      }

      Write-Output $line
      if ($null -ne $writer) {
        $writer.WriteLine($line)
      }
      $lineCount++
    } catch [System.TimeoutException] {
    }
  }
} finally {
  if ($null -ne $writer) {
    $writer.Dispose()
  }

  if ($serialPort.IsOpen) {
    $serialPort.Close()
  }
  $serialPort.Dispose()
}
