$ErrorActionPreference = 'Stop'
[Console]::OutputEncoding = New-Object System.Text.UTF8Encoding($false)
$env:PYTHONIOENCODING = 'utf-8'
Set-Location -LiteralPath $PSScriptRoot
$candidateInterpreter = Join-Path $PSScriptRoot '../../work/go2-perception/.venv/Scripts/python.exe'
if (Test-Path -LiteralPath $candidateInterpreter) {
    $pythonExe = (Resolve-Path -LiteralPath $candidateInterpreter).Path
} else {
    $pythonExe = (Get-Command python -ErrorAction Stop).Source
}
$runName = 'offline-' + (Get-Date -Format 'yyyyMMdd-HHmmss-fff')
$runPath = Join-Path $PSScriptRoot $runName
New-Item -ItemType Directory -Path $runPath | Out-Null
Write-Host 'Проверка без робота: соединение и команды движения отсутствуют.'
foreach ($sceneSeed in 0..3) {
    $resultPath = Join-Path $runPath ('demo-' + $sceneSeed + '.json')
    & $pythonExe -m wolf_go2 demo --seed $sceneSeed --output $resultPath
    if ($LASTEXITCODE -ne 0) { throw 'Проверка не завершилась успешно. Смотрите отчёт выше.' }
}
Write-Host ('Результаты: ' + $runPath)
Write-Host 'Проверена только кинематика. Это не испытание физического Go2.'
