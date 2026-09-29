$ErrorActionPreference = 'Stop'

$webotsExe = 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe'
if (-not (Test-Path -LiteralPath $webotsExe)) {
    throw 'Webots R2025a was not found.'
}

$world = Join-Path $PSScriptRoot 'worlds\go2_aframe_climb.wbt'
$quotedWorld = '"{0}"' -f $world
Start-Process -FilePath $webotsExe -ArgumentList $quotedWorld
Start-Sleep -Seconds 4

$rosScript = '/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/webots/run_aframe_ros.sh'
wsl.exe -d Ubuntu-24.04 -- bash $rosScript
