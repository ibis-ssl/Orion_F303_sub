# Backs up F303 sub Flash and installs bootloader, application, and metadata only with -Execute.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",
  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",
  [string]$ProbeSerial = "",
  [string]$BackupDirectory = "",
  [switch]$Execute
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$bootloaderBin = Join-Path $repoRoot "Bootloader\Build\Orion_F303_sub_bootloader.bin"
$applicationBin = Join-Path $repoRoot "$Configuration\Orion_F303_sub_app.bin"
$metadataBin = Join-Path $repoRoot "$Configuration\Orion_F303_sub_app.metadata.bin"
$crcScript = Join-Path $scriptDir "orion_crc32c.py"
if ([string]::IsNullOrWhiteSpace($BackupDirectory)) {
  $BackupDirectory = Join-Path $scriptDir ("Logs\bootloader_install_" + (Get-Date -Format "yyyyMMdd_HHmmss"))
}
$logsRoot = [IO.Path]::GetFullPath((Join-Path $scriptDir "Logs")).TrimEnd('\') + '\'
$backupFull = [IO.Path]::GetFullPath($BackupDirectory)
if (-not $backupFull.StartsWith($logsRoot, [StringComparison]::OrdinalIgnoreCase)) { throw "BackupDirectory must be below $logsRoot" }
foreach ($path in @($ProgrammerPath, $bootloaderBin, $applicationBin, $metadataBin, $crcScript)) {
  if (-not (Test-Path $path)) { throw "Required file not found: $path" }
}
$applicationSize = (Get-Item -LiteralPath $applicationBin).Length
$applicationPageSize = 0x800
$applicationFirstPage = 8
$applicationMaxSize = 0x1B800
if ($applicationSize -le 0 -or $applicationSize -gt $applicationMaxSize) {
  throw "Application binary size is outside the application region: $applicationSize bytes"
}
$applicationLastPage = $applicationFirstPage + [Math]::Floor(($applicationSize - 1) / $applicationPageSize)
$applicationPages = @($applicationFirstPage..$applicationLastPage)
New-Item -ItemType Directory -Path $BackupDirectory -Force | Out-Null
$flashBackup = Join-Path $BackupDirectory "sub_flash_before_bootloader.bin"
$optionBackup = Join-Path $BackupDirectory "option_bytes_before_bootloader.txt"
$crcBackup = Join-Path $BackupDirectory "sub_flash_before_bootloader.crc32c.txt"
$hotplug = "port=SWD mode=HOTPLUG"
$underReset = "port=SWD mode=UR freq=1000"
if (-not [string]::IsNullOrWhiteSpace($ProbeSerial)) {
  $hotplug += " sn=$ProbeSerial"
  $underReset += " sn=$ProbeSerial"
}
$connectOutput = & $ProgrammerPath -c $hotplug 2>&1
if ($LASTEXITCODE -ne 0 -or ($connectOutput -join "`n") -notmatch 'Device ID\s+: 0x422') { throw "Expected STM32F303xB/C target (0x422) was not detected" }
$optionOutput = & $ProgrammerPath -c $hotplug -ob displ 2>&1
if ($LASTEXITCODE) { throw "Option Bytes read failed" }
$optionOutput | Set-Content -Encoding UTF8 $optionBackup
& $ProgrammerPath -c $hotplug -u 0x08000000 0x20000 $flashBackup
if ($LASTEXITCODE -or (Get-Item $flashBackup).Length -ne 0x20000) { throw "128KB Flash backup failed" }
$backupCrc = & python $crcScript $flashBackup
if ($LASTEXITCODE -or $backupCrc -notmatch '^0x[0-9A-F]{8}$') { throw "Flash backup CRC32C failed" }
$backupCrc | Set-Content -Encoding ASCII $crcBackup
Write-Output "Flash backup: $flashBackup"
Write-Output "Flash backup CRC32C: $backupCrc"
Write-Output "Option Bytes backup: $optionBackup"
if (-not $Execute) {
  Write-Output "Dry run completed. No Flash write or reset was performed."
  exit 0
}
$fullEraseSucceeded = $false
foreach ($attempt in 1..3) {
  $eraseOutput = & $ProgrammerPath -c $underReset -e all 2>&1
  if ($LASTEXITCODE -eq 0) { $fullEraseSucceeded = $true; break }
  Start-Sleep -Milliseconds 300
}
if (-not $fullEraseSucceeded) { throw "Full Flash erase failed after 3 attempts:`n$($eraseOutput -join "`n")" }
Write-Output "Full Flash erase completed."
& $ProgrammerPath -c $underReset --skipErase -w $bootloaderBin 0x08000000 -v
if ($LASTEXITCODE) { throw "Bootloader write failed" }
# STM32CubeProgrammer 2.22.0 can report a successful mass erase while an
# immediate large program still contains stale bits. Re-erase every page used
# by the application. A sector list avoids reconnecting for every page.
$eraseArgs = @("-c", $underReset, "-e") + $applicationPages
$eraseOutput = & $ProgrammerPath @eraseArgs 2>&1
if ($LASTEXITCODE) {
  Write-Warning "Batch application page erase failed; retrying one page at a time."
  foreach ($sector in $applicationPages) {
    $pageEraseSucceeded = $false
    foreach ($attempt in 1..3) {
      $eraseOutput = & $ProgrammerPath -c $underReset -e $sector 2>&1
      if ($LASTEXITCODE -eq 0) { $pageEraseSucceeded = $true; break }
      Start-Sleep -Milliseconds 300
    }
    if (-not $pageEraseSucceeded) { throw "Application page erase failed: sector $sector`n$($eraseOutput -join "`n")" }
  }
}
& $ProgrammerPath -c $underReset --skipErase -w $applicationBin 0x08004000 -v fast
if ($LASTEXITCODE) { throw "Application write failed" }
$eraseOutput = & $ProgrammerPath -c $underReset -e 63 2>&1
if ($LASTEXITCODE) { throw "Metadata page erase failed:`n$($eraseOutput -join "`n")" }
& $ProgrammerPath -c $underReset --skipErase -w $metadataBin 0x0801F800 -v fast -rst
if ($LASTEXITCODE) { throw "Metadata write or reset failed" }
Write-Output "F303 sub bootloader installation completed."
