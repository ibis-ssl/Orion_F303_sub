# Builds the relocated F303 sub application and generates matching metadata.
param(
  [ValidateSet("Debug", "Release")]
  [string]$Configuration = "Debug",
  [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",
  [string]$ToolchainBin = "C:\ST\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin",
  [ValidateRange(1, 64)]
  [int]$Jobs = 4,
  [switch]$Rebuild,
  [int]$Generation = 1
)

$ErrorActionPreference = "Stop"
$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Split-Path -Parent $scriptDir
$buildDir = Join-Path $repoRoot $Configuration
$elfPath = Join-Path $buildDir "Orion_F303_sub.elf"
$binPath = Join-Path $buildDir "Orion_F303_sub_app.bin"
$mapPath = Join-Path $buildDir "Orion_F303_sub.map"
$metadataPath = Join-Path $buildDir "Orion_F303_sub_app.metadata.bin"
$buildArgs = @{ Configuration = $Configuration; MakePath = $MakePath; Jobs = $Jobs }
if ($Rebuild) { $buildArgs.Rebuild = $true }
& (Join-Path $scriptDir "build.ps1") @buildArgs
if ($LASTEXITCODE) { throw "F303 sub application build failed" }
if (-not (Test-Path $elfPath) -or -not (Test-Path $mapPath)) { throw "Application artifacts missing" }
if ($null -eq (Select-String -Path $mapPath -Pattern '^FLASH\s+0x08004000\s+0x0001b800' | Select-Object -First 1)) { throw "Link map is not configured for the application region" }
if (-not (Test-Path $ToolchainBin)) { throw "GNU toolchain not found: $ToolchainBin" }
& python (Join-Path $scriptDir "stamp_fw_version.py") $elfPath --objcopy (Join-Path $ToolchainBin "arm-none-eabi-objcopy.exe") --repo $repoRoot --target sub --log-dir (Join-Path $scriptDir "Logs\Build")
if ($LASTEXITCODE) { throw "FW version stamping failed" }
& (Join-Path $ToolchainBin "arm-none-eabi-objcopy.exe") -O binary $elfPath $binPath
if ($LASTEXITCODE) { throw "objcopy failed" }
& python (Join-Path $scriptDir "generate_boot_metadata.py") $binPath $metadataPath --generation $Generation
if ($LASTEXITCODE) { throw "Metadata generation failed" }
if ((Get-Item $binPath).Length -gt 0x1B800) { throw "Application exceeds 110KB" }
Write-Output "Application image: $binPath"
Write-Output "Application metadata: $metadataPath"
