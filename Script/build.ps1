param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",

  [ValidateRange(1, 64)]
  [int]$Jobs = 4,

  [switch]$Rebuild
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildDir = Join-Path $repoRoot $Configuration

if (-not (Test-Path -LiteralPath $MakePath -PathType Leaf)) {
  throw "make.exe not found: $MakePath"
}

if (-not (Test-Path -LiteralPath $buildDir -PathType Container)) {
  throw "Build directory not found: $buildDir"
}

$makefilePath = Join-Path $buildDir "makefile"
if (-not (Test-Path -LiteralPath $makefilePath -PathType Leaf)) {
  throw "Build makefile not found: $makefilePath"
}

$makeArgs = @()
if ($Rebuild) {
  $makeArgs += "-B"
}
$makeArgs += "-j$Jobs"
$makeArgs += "all"

Push-Location $buildDir
try {
  & $MakePath @makeArgs
  if ($LASTEXITCODE -ne 0) {
    throw "Build failed for $Configuration"
  }
} finally {
  Pop-Location
}
