$ErrorActionPreference = 'Stop'

$webotsCandidates = @(
    'C:\Program Files\Webots\msys64\mingw64\bin\webots.exe',
    'C:\Program Files\Webots\webots.exe'
)

$webotsExe = $webotsCandidates | Where-Object { Test-Path -LiteralPath $_ } | Select-Object -First 1
if (-not $webotsExe) {
    throw 'Webots was not found in the standard installation directories.'
}

$world = Join-Path $PSScriptRoot 'worlds\truetech_arena.wbt'
Start-Process -FilePath $webotsExe -ArgumentList @($world)
