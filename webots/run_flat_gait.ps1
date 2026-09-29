$ErrorActionPreference = 'Stop'

$webotsExe = 'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe'
if (-not (Test-Path -LiteralPath $webotsExe)) {
    throw 'Webots R2025a was not found.'
}

$world = Join-Path $PSScriptRoot 'worlds\go2_flat_gait.wbt'
$quotedWorld = '"{0}"' -f $world
Start-Process -FilePath $webotsExe -ArgumentList $quotedWorld
