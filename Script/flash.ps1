# Writes the relocated F303 sub application and metadata after bootloader installation.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [string]$ProbeSerial = "",

  [switch]$List,
  [switch]$ConnectOnly,
  [switch]$BootloaderInstalled,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildDir = Join-Path $repoRoot $Configuration
$makefilePath = Join-Path $buildDir "makefile"
$connection = "port=SWD mode=UR freq=1000"
$hotplug = "port=SWD mode=HOTPLUG freq=1000"
if (-not [string]::IsNullOrWhiteSpace($ProbeSerial)) {
  $connection += " sn=$ProbeSerial"
  $hotplug += " sn=$ProbeSerial"
}

if (-not (Test-Path -LiteralPath $ProgrammerPath -PathType Leaf)) {
  throw "STM32_Programmer_CLI.exe not found: $ProgrammerPath"
}

if ($List) {
  & $ProgrammerPath "-l" "stlink"
  if ($LASTEXITCODE -ne 0) {
    throw "ST-Link listing failed"
  }
  exit 0
}

if ($ConnectOnly) {
  & $ProgrammerPath "-c" $connection "-rst"
  if ($LASTEXITCODE -ne 0) {
    throw "Target connection failed"
  }
  exit 0
}

if (-not (Test-Path -LiteralPath $makefilePath -PathType Leaf)) {
  throw "Build makefile not found: $makefilePath"
}

$artifactLine = Select-String -LiteralPath $makefilePath -Pattern '^\s*BUILD_ARTIFACT_NAME\s*:?=\s*(\S+)\s*$' | Select-Object -First 1
if ($null -eq $artifactLine) {
  throw "BUILD_ARTIFACT_NAME not found in: $makefilePath"
}

$artifactName = $artifactLine.Matches[0].Groups[1].Value
$elfPath = Join-Path $buildDir ($artifactName + ".elf")
if (-not (Test-Path -LiteralPath $elfPath -PathType Leaf)) {
  throw "ELF not found: $elfPath"
}

if (-not $BootloaderInstalled) {
  throw "The application is linked at 0x08004000. Use install_sub_bootloader.ps1 -Execute first, or pass -BootloaderInstalled."
}

$metadataPath = Join-Path $buildDir "Orion_F303_sub_app.metadata.bin"
if (-not (Test-Path $metadataPath)) { throw "Metadata not found: $metadataPath. Run build_application.ps1 first." }

Write-Output "Checking STM32F303 target connection..."
$targetOutput = & $ProgrammerPath -c $hotplug 2>&1
if ($LASTEXITCODE -ne 0 -or ($targetOutput -join "`n") -notmatch 'Device ID\s+: 0x422') {
  throw "Expected STM32F303xB/C target (0x422) was not detected; no write was performed."
}
Write-Output "STM32F303 target detected."

$applicationBin = Join-Path $buildDir "Orion_F303_sub_app.bin"
if (-not (Test-Path $applicationBin)) { throw "Application binary not found: $applicationBin. Run build_application.ps1 first." }
$applicationSize = (Get-Item -LiteralPath $applicationBin).Length
$applicationPageSize = 0x800
$applicationFirstPage = 8
$applicationMaxSize = 0x1B800
if ($applicationSize -le 0 -or $applicationSize -gt $applicationMaxSize) {
  throw "Application binary size is outside the application region: $applicationSize bytes"
}
$applicationLastPage = $applicationFirstPage + [Math]::Floor(($applicationSize - 1) / $applicationPageSize)
$pagesToErase = @($applicationFirstPage..$applicationLastPage) + 63

# Invalidate commit metadata before modifying any application page.
Write-Output "Erasing application pages $applicationFirstPage..$applicationLastPage and metadata page 63 in one connection..."
$eraseArgs = @("-c", $connection, "-e") + $pagesToErase
$eraseOutput = & $ProgrammerPath @eraseArgs 2>&1
if ($LASTEXITCODE) {
  Write-Warning "Batch page erase failed; retrying one page at a time."
  foreach ($sector in $pagesToErase) {
    Write-Output "Erasing page $sector..."
    $eraseOutput = & $ProgrammerPath -c $connection -e $sector 2>&1
    if ($LASTEXITCODE) { throw "Flash page erase failed: sector $sector`n$($eraseOutput -join "`n")" }
  }
}
Write-Output "Application pages $applicationFirstPage..$applicationLastPage and metadata page 63 erased."

Write-Output "Programming application image..."
$programmerArgs = @("-c", $connection, "--skipErase", "-w", $applicationBin, "0x08004000")
if (-not $NoVerify) { $programmerArgs += @("-v", "fast") }
& $ProgrammerPath @programmerArgs
if ($LASTEXITCODE -ne 0) { throw "Flash failed for $Configuration" }

Write-Output "Programming application metadata..."
$metadataArgs = @("-c", $connection, "--skipErase", "-w", $metadataPath, "0x0801F800")
if (-not $NoVerify) { $metadataArgs += @("-v", "fast") }
if (-not $NoReset) { $metadataArgs += "-rst" }
& $ProgrammerPath @metadataArgs
if ($LASTEXITCODE) { throw "Metadata Flash failed for $Configuration" }
Write-Output "Application and metadata flash completed."
