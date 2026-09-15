# Programs the bootloader, relocated application, and metadata in one operation.

$ErrorActionPreference = "Stop"

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$installScript = Join-Path $scriptDir "install_sub_bootloader.ps1"

& $installScript -Execute
if ($LASTEXITCODE -ne 0) {
  throw "Bootloader and application flash failed"
}

