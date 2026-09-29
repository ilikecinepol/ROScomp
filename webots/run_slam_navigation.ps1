param([switch]$NoBrowser)

$ErrorActionPreference = 'Stop'

$webotsCandidates = @(
    'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe',
    'C:\Program Files\Webots\webots.exe'
)
$webotsExe = $webotsCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $webotsExe) {
    throw 'Webots was not found in the standard installation directories.'
}

# Only one Webots instance may own the Go2 UDP port.
Get-Process webots, webots-bin -ErrorAction SilentlyContinue | Stop-Process -Force
Start-Sleep -Milliseconds 800

$world = Join-Path $PSScriptRoot 'worlds\truetech_arena.wbt'
$quotedWorld = '"{0}"' -f $world
Start-Process -FilePath $webotsExe -ArgumentList $quotedWorld
Start-Sleep -Seconds 5

# Open the native Windows browser after the ROS web map has started in WSL.
if (-not $NoBrowser) {
    $browserCommand = 'Start-Sleep -Seconds 12; Start-Process "http://localhost:8765"'
    Start-Process -FilePath 'powershell.exe' -ArgumentList '-NoProfile', '-Command', $browserCommand -WindowStyle Hidden
}

$linuxScript = '/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/webots/run_slam_navigation.sh'
wsl.exe -d Ubuntu-24.04 -- bash $linuxScript
