# Refresh IntelliSense from make's dry-run commands; no firmware is built or flashed.
param(
    [string]$MakePath = "C:\ST\STM32CubeIDE_1.17.0\STM32CubeIDE\plugins\com.st.stm32cube.ide.mcu.externaltools.make.win32_2.2.0.202409170845\tools\bin\make.exe",
    [string]$CompilerPath = ""
)

$ErrorActionPreference = "Stop"
$repoRoot = Split-Path -Parent $PSScriptRoot
if (-not (Test-Path -LiteralPath $MakePath -PathType Leaf)) {
    throw "make.exe not found: $MakePath. Specify -MakePath."
}
if (-not $CompilerPath) {
    $compiler = Get-Command arm-none-eabi-gcc.exe -ErrorAction SilentlyContinue
    if ($compiler) { $CompilerPath = $compiler.Source }
}
if (-not $CompilerPath -or -not (Test-Path -LiteralPath $CompilerPath -PathType Leaf)) {
    throw "ARM GCC not found. Add it to PATH or specify -CompilerPath."
}
$CompilerPath = (Resolve-Path -LiteralPath $CompilerPath).Path.Replace('\', '/')

function Get-CompileCommands([string]$BuildDirectory) {
    $buildPath = Join-Path $repoRoot $BuildDirectory
    # Request only object commands: code browsing does not need linker outputs.
    $temporaryMakefile = [System.IO.Path]::GetTempFileName()
    try {
        [System.IO.File]::WriteAllText($temporaryMakefile, '__vscode_browse__: $(OBJS)' + "`n")
        $lines = & $MakePath --no-print-directory -C $buildPath -f Makefile -f $temporaryMakefile -n -B __vscode_browse__
        if ($LASTEXITCODE -ne 0) { throw "make dry run failed: $BuildDirectory" }
    } finally {
        Remove-Item -LiteralPath $temporaryMakefile
    }
    $count = 0
    foreach ($line in $lines) {
        if ($line -notmatch '^arm-none-eabi-gcc\s+(.+)$') { continue }
        $commandArgs = $Matches[1]
        if ($commandArgs -notmatch '(?:^|\s)-c(?:\s|$)') { continue }
        # CubeIDE/bootloader recipes use double-quoted or unquoted relative paths.
        $sourceTokens = @([regex]::Matches($commandArgs, '(?:[^\s"]+|"[^"]*")+') |
            ForEach-Object { $_.Value.Replace('"', '') } |
            Where-Object { $_ -match '\.c$' -and $_ -notmatch '^-' })
        if ($sourceTokens.Count -ne 1) { continue }
        $sourcePath = (Resolve-Path -LiteralPath (Join-Path $buildPath $sourceTokens[0])).Path
        [ordered]@{
            directory = $buildPath.Replace('\', '/')
            file = $sourcePath.Replace('\', '/')
            command = '"' + $CompilerPath + '" ' + $commandArgs
        }
        $count++
    }
    if ($count -eq 0) { throw "No C compilation commands found: $BuildDirectory" }
}

$utf8 = New-Object System.Text.UTF8Encoding($false)
$bootCommands = @(Get-CompileCommands "Bootloader")
foreach ($configuration in @("Debug", "Release")) {
    $commands = @(Get-CompileCommands $configuration) + $bootCommands
    $databasePath = Join-Path $repoRoot ".vscode/compile_commands_$configuration.json"
    [System.IO.File]::WriteAllText($databasePath, (ConvertTo-Json -InputObject $commands -Depth 5), $utf8)
    Write-Output "$configuration : $($commands.Count) C files -> $databasePath"
}
$propertiesPath = Join-Path $repoRoot ".vscode/c_cpp_properties.json"
$properties = Get-Content -LiteralPath $propertiesPath -Raw | ConvertFrom-Json
$debug = $properties.configurations[0]
$debug.compilerPath = $CompilerPath
# Copy the fallback configuration; each database also contains bootloader commands.
$release = $debug | ConvertTo-Json -Depth 10 | ConvertFrom-Json
$release.name = "STM32 Release"
$release.defines = @($debug.defines | Where-Object { $_ -ne 'DEBUG' })
$release.compileCommands = '${workspaceFolder}/.vscode/compile_commands_Release.json'
$properties.configurations = @($debug, $release)
[System.IO.File]::WriteAllText($propertiesPath, ($properties | ConvertTo-Json -Depth 10), $utf8)
Write-Output "IntelliSense compiler: $CompilerPath"
