$ErrorActionPreference = "Stop"

$logPath = "C:\Users\drmma\Documents\ChatGPT\ROS2 Competition\webots\tools\enable_wsl2.log"
$stdoutPath = "$logPath.stdout"
$stderrPath = "$logPath.stderr"

function Enable-Feature([string] $featureName) {
    Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
    $process = Start-Process -FilePath "dism.exe" -Wait -PassThru -NoNewWindow `
        -ArgumentList "/online", "/enable-feature", "/featurename:$featureName", "/all", "/norestart" `
        -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath
    if (Test-Path -LiteralPath $stdoutPath) {
        Get-Content -LiteralPath $stdoutPath | Add-Content -LiteralPath $logPath -Encoding utf8
    }
    if (Test-Path -LiteralPath $stderrPath) {
        Get-Content -LiteralPath $stderrPath | Add-Content -LiteralPath $logPath -Encoding utf8
    }
    "DISM exit code for ${featureName}: $($process.ExitCode)" | Add-Content -LiteralPath $logPath -Encoding utf8
    if ($process.ExitCode -notin 0, 3010) {
        throw "DISM failed for $featureName with exit code $($process.ExitCode)"
    }
}

"Enabling Windows Subsystem for Linux..." | Set-Content -LiteralPath $logPath -Encoding utf8
Enable-Feature "Microsoft-Windows-Subsystem-Linux"

"Enabling Virtual Machine Platform..." | Add-Content -LiteralPath $logPath -Encoding utf8
Enable-Feature "VirtualMachinePlatform"

"WSL2 Windows features enabled. Restart required: yes" | Add-Content -LiteralPath $logPath -Encoding utf8
Remove-Item -LiteralPath $stdoutPath, $stderrPath -Force -ErrorAction SilentlyContinue
