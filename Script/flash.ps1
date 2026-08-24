param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [switch]$List,
  [switch]$ConnectOnly,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildDir = Join-Path $repoRoot $Configuration
$makefilePath = Join-Path $buildDir "makefile"

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
  & $ProgrammerPath "-c" "port=SWD mode=UR" "-rst"
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

$programmerArgs = @(
  "-c", "port=SWD mode=UR",
  "-w", $elfPath
)

if (-not $NoVerify) {
  $programmerArgs += "-v"
}

if (-not $NoReset) {
  $programmerArgs += "-rst"
}

& $ProgrammerPath @programmerArgs
if ($LASTEXITCODE -ne 0) {
  throw "Flash failed for $Configuration"
}
