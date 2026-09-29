$ErrorActionPreference = "Stop"

$basePath = "C:\Users\drmma\Documents\ChatGPT\ROS2 Competition\webots\tools\wsl_install"
$stdoutPath = "$basePath.stdout.log"
$stderrPath = "$basePath.stderr.log"
$resultPath = "$basePath.result.log"

Remove-Item -LiteralPath $stdoutPath, $stderrPath, $resultPath -Force -ErrorAction SilentlyContinue
$process = Start-Process -FilePath "wsl.exe" -Wait -PassThru -NoNewWindow `
    -ArgumentList "--install", "-d", "Ubuntu-24.04", "--no-launch" `
    -RedirectStandardOutput $stdoutPath -RedirectStandardError $stderrPath

"WSL install exit code: $($process.ExitCode)" | Set-Content -LiteralPath $resultPath -Encoding utf8
if ($process.ExitCode -ne 0) {
    exit $process.ExitCode
}
