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
$env:QT_QPA_PLATFORM = 'windows'
$env:QT_FONT_DPI = '96'
$env:QT_SCALE_FACTOR = '1'
$env:QT_AUTO_SCREEN_SCALE_FACTOR = '0'
Remove-Item Env:QT_SCREEN_SCALE_FACTORS, Env:QT_DEVICE_PIXEL_RATIO, Env:QT_ENABLE_HIGHDPI_SCALING `
    -ErrorAction SilentlyContinue
$resultDirectory = Join-Path $PSScriptRoot 'test-results'
New-Item -ItemType Directory -Force -Path $resultDirectory | Out-Null
$webotsOut = Join-Path $resultDirectory 'slam-webots.out.log'
$webotsErr = Join-Path $resultDirectory 'slam-webots.err.log'
$webotsArguments = @(
    '--mode=realtime'
    '--stdout'
    '--stderr'
    $quotedWorld
)
Start-Process -FilePath $webotsExe -ArgumentList $webotsArguments `
    -RedirectStandardOutput $webotsOut -RedirectStandardError $webotsErr -WindowStyle Normal
$controllerReady = $false
for ($attempt = 0; $attempt -lt 30; $attempt++) {
    Start-Sleep -Seconds 1
    # Get-NetUDPEndpoint may omit an endpoint owned by Webots' child Python
    # process on some Windows builds. netstat observes the socket reliably.
    $controllerReady = [bool](netstat -ano | Select-String -Quiet 'UDP\s+\S+:15000\s')
    if ($controllerReady) {
        break
    }
}

if (-not $controllerReady) {
    $details = Get-Content -LiteralPath $webotsErr -Raw -ErrorAction SilentlyContinue
    throw "Webots controller did not open UDP port 15000 within 30 seconds. $details"
}

# Open the native Windows browser after the ROS web map has started in WSL.
if (-not $NoBrowser) {
    $browserCommand = 'Start-Sleep -Seconds 12; Start-Process "http://localhost:8765"'
    Start-Process -FilePath 'powershell.exe' -ArgumentList '-NoProfile', '-Command', $browserCommand -WindowStyle Hidden
}

$linuxScript = '/mnt/c/Users/drmma/Documents/ChatGPT/ROS2 Competition/webots/run_slam_navigation.sh'
wsl.exe -d Ubuntu-24.04 -- bash $linuxScript
