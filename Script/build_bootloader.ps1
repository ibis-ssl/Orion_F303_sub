# Builds the F303 sub bootloader and enforces its 16 KB size limit.
param(
  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",
  [string]$ToolchainBin = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin",
  [switch]$Rebuild
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$bootloaderDir = Join-Path $repoRoot "Bootloader"
$elfPath = Join-Path $bootloaderDir "Build\Orion_F303_sub_bootloader.elf"
if (-not (Test-Path $MakePath)) { throw "make.exe not found: $MakePath" }
if (-not (Test-Path $ToolchainBin)) { throw "GNU toolchain not found: $ToolchainBin" }
$env:Path = "$ToolchainBin;$env:Path"
$args = @("-C", $bootloaderDir, "-j4")
if ($Rebuild) { $args += "-B" }
$args += "all"
& $MakePath @args
if ($LASTEXITCODE) { throw "F303 sub bootloader build failed" }
$sizeOutput = & arm-none-eabi-size.exe -A $elfPath
if ($LASTEXITCODE) { throw "arm-none-eabi-size failed" }
$flashUsed = 0
foreach ($line in $sizeOutput) {
  if ($line -match '^\.(isr_vector|text|rodata|ARM|init_array|fini_array|data)\s+(\d+)') { $flashUsed += [int]$Matches[2] }
}
if ($flashUsed -gt 0x4000) { throw "Bootloader exceeds 16KB: $flashUsed bytes" }
Write-Output "Bootloader Flash usage: $flashUsed / 16384 bytes"
Write-Output "Bootloader ELF: $elfPath"
& python (Join-Path $scriptDir "stamp_fw_version.py") --log-only --repo $repoRoot --target sub_bootloader --log-dir (Join-Path $scriptDir "Logs\Build")
if ($LASTEXITCODE) { throw "Bootloader build logging failed" }
