param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",

  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",

  [string]$ProgrammerPath = "C:\ST\STM32CubeCLT_1.21.0\STM32CubeProgrammer\bin\STM32_Programmer_CLI.exe",

  [ValidateRange(1, 64)]
  [int]$Jobs = 4,

  [switch]$Rebuild,
  [switch]$NoVerify,
  [switch]$NoReset
)

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$buildScript = Join-Path $scriptDir "build.ps1"
$flashScript = Join-Path $scriptDir "flash.ps1"

$buildArgs = @{
  Configuration = $Configuration
  MakePath = $MakePath
  Jobs = $Jobs
}
if ($Rebuild) {
  $buildArgs.Rebuild = $true
}

& $buildScript @buildArgs
if ($LASTEXITCODE -ne 0) {
  throw "Build step failed"
}

$flashArgs = @{
  Configuration = $Configuration
  ProgrammerPath = $ProgrammerPath
}
if ($NoVerify) {
  $flashArgs.NoVerify = $true
}
if ($NoReset) {
  $flashArgs.NoReset = $true
}

& $flashScript @flashArgs
if ($LASTEXITCODE -ne 0) {
  throw "Flash step failed"
}
