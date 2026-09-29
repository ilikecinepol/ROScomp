$ErrorActionPreference = 'Stop'
$taskRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
$taskPython = Join-Path $taskRoot 'work/go2-perception/.venv/Scripts/python.exe'
$taskRuntime = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python'
$env:TCL_LIBRARY = Join-Path $taskRuntime 'tcl/tcl8.6'
$env:TK_LIBRARY = Join-Path $taskRuntime 'tcl/tk8.6'
Set-Location -LiteralPath $taskRoot
& $taskPython (Join-Path $taskRoot 'work/go2-teleop/keyboard_client.py')
if ($LASTEXITCODE -ne 0) { Read-Host 'Startup failed. Keep this error for diagnostics; press Enter' }
