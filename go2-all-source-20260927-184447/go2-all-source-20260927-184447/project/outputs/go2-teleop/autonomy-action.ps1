param([Parameter(Mandatory=$true)][string]$Ip,[ValidateSet('upload','status','start','stop')][string]$Action)
$ErrorActionPreference='Stop'
& "$PSScriptRoot\set-ip.ps1" $Ip | Out-Host
$key=$env:GO2_PI_KEY; $target=$env:GO2_PI_TARGET
$root='C:\Users\VladO\Documents\Codex\2026-09-09\new-chat'; $stamp=Get-Date -Format yyyyMMdd-HHmmss; $ver="autonomy-$stamp"; $archive=Join-Path $PSScriptRoot "$ver.zip"
if($Action -eq 'upload'){
  $items=@("$root\work\go2-autonomy\autonomous.py","$root\work\go2-autonomy\wolf_go2")
  if(!(Test-Path $items[0])){throw 'Локальный автономный исходник не найден'}
  Compress-Archive -Path $items -DestinationPath $archive -Force
  & scp -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -i $key $archive "${target}:/home/ubuntu/ai-robot/team_wolf_setup/"
  if($LASTEXITCODE -ne 0){throw 'Не удалось загрузить архив'}
  & ssh -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -i $key $target "cd /home/ubuntu/ai-robot/team_wolf_setup && mkdir -p $ver && python3 -m zipfile -e $ver.zip $ver"
  if($LASTEXITCODE -ne 0){throw 'Не удалось распаковать автономную версию'}
  Write-Host "Загружено: $ver"
}elseif($Action -eq 'status'){
  & ssh -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -i $key $target "pgrep -af 'autonomous.py|run_autonomy' || true"
}elseif($Action -eq 'stop'){
  & ssh -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -i $key $target "pkill -f 'team_wolf_setup/.*/autonomous.py' || true"
  Write-Host 'Команда остановки отправлена.'
}else{
  & ssh -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -i $key $target "cd /home/ubuntu/ai-robot/team_wolf_setup/autonomy-20260918 && nohup /home/ubuntu/ai-robot/venv/bin/python autonomous.py live > autonomy-live.log 2>&1 & echo `$!"
  Write-Host 'Команда запуска отправлена.'
}