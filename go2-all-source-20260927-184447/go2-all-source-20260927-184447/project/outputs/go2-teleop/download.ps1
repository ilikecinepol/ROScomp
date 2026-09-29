$ErrorActionPreference = 'Stop'
try {
    $taskRoot = Split-Path (Split-Path $PSScriptRoot -Parent) -Parent
    $taskPython = Join-Path $taskRoot 'work/go2-perception/.venv/Scripts/pythonw.exe'
    $taskScript = Join-Path $taskRoot 'work/go2-teleop/download_recordings.py'
    $taskRuntime = Join-Path $env:USERPROFILE '.cache/codex-runtimes/codex-primary-runtime/dependencies/python'
    $env:TCL_LIBRARY = Join-Path $taskRuntime 'tcl/tcl8.6'
    $env:TK_LIBRARY = Join-Path $taskRuntime 'tcl/tk8.6'
    if (!(Test-Path -LiteralPath $taskPython)) { throw "Python not found: $taskPython" }
    if (!(Test-Path -LiteralPath $taskScript)) { throw "Downloader not found: $taskScript" }
    Start-Process -FilePath $taskPython -ArgumentList ('"' + $taskScript + '"') -WorkingDirectory $taskRoot -WindowStyle Hidden
} catch {
    Add-Type -AssemblyName System.Windows.Forms
    [void][System.Windows.Forms.MessageBox]::Show($_.Exception.Message, 'Go2 download startup error')
    exit 1
}
