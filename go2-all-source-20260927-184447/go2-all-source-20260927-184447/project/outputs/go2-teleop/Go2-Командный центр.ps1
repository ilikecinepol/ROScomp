Add-Type -AssemblyName System.Windows.Forms
Add-Type -AssemblyName System.Drawing
$ErrorActionPreference='Stop'
$root = Split-Path -Parent $MyInvocation.MyCommand.Path
$workspace = 'C:\Users\VladO\Documents\Codex\2026-09-09\new-chat'
$python = Join-Path $workspace 'work\go2-perception\.venv\Scripts\python.exe'
$client = Join-Path $workspace 'work\go2-teleop\keyboard_client.py'
$config = Join-Path $root 'go2-quick-config.json'
$old = if(Test-Path $config){try{Get-Content $config -Raw | ConvertFrom-Json}catch{[pscustomobject]@{Ip=''}}}else{[pscustomobject]@{Ip=''}}
$form = New-Object Windows.Forms.Form
$form.Text='Go2 — быстрое подключение и управление'
$form.Width=650; $form.Height=330; $form.StartPosition='CenterScreen'; $form.Font=New-Object Drawing.Font('Segoe UI',10)
$label=New-Object Windows.Forms.Label; $label.Text='IP Raspberry Pi:'; $label.Location='20,22'; $label.AutoSize=$true
$ip=New-Object Windows.Forms.TextBox; $ip.Location='145,18'; $ip.Width=220; $ip.Text=$old.Ip
$paste=New-Object Windows.Forms.Button; $paste.Text='Вставить IP'; $paste.Location='380,16'; $paste.Width=110; $paste.Add_Click({try{$ip.Text=[Windows.Forms.Clipboard]::GetText().Trim()}catch{}})
$status=New-Object Windows.Forms.Label; $status.Text='Введите IP и нажмите «Запустить управление»'; $status.Location='20,62'; $status.Width=590; $status.Height=42
$log=New-Object Windows.Forms.TextBox; $log.Multiline=$true; $log.ReadOnly=$true; $log.ScrollBars='Vertical'; $log.Location='20,185'; $log.Width=590; $log.Height=85
$form.Controls.AddRange(@($label,$ip,$paste,$status,$log))
function ValidIp($v){return $v -match '^(?:\d{1,3}\.){3}\d{1,3}$' -and (($v.Split('.')|ForEach-Object {[int]$_})|Where-Object {$_ -gt 255}).Count -eq 0}
function SaveIp{$v=$ip.Text.Trim(); [pscustomobject]@{Ip=$v}|ConvertTo-Json|Set-Content $config -Encoding UTF8; return $v}
$check=New-Object Windows.Forms.Button; $check.Text='Проверить Pi'; $check.Location='20,105'; $check.Width=180; $check.Height=45
$check.Add_Click({try{$v=SaveIp;if(!(ValidIp $v)){throw 'Некорректный IPv4'};$status.Text='Проверяем порт SSH…';$r=Test-NetConnection $v -Port 22 -InformationLevel Quiet;$log.AppendText("`r`nIP: $v`r`nSSH 22: $r`r`n");$status.Text=if($r){'Pi доступна по SSH'}else{'SSH недоступен — проверьте VPN'}}catch{$status.Text='Ошибка: '+$_.Exception.Message}})
$run=New-Object Windows.Forms.Button; $run.Text='Запустить управление и запись'; $run.Location='220,105'; $run.Width=250; $run.Height=45
$run.Add_Click({try{$v=SaveIp;if(!(ValidIp $v)){throw 'Введите актуальный IPv4'};if(!(Test-Path $python)){throw "Не найден Python: $python"};if(!(Test-Path $client)){throw "Не найден клиент: $client"};$env:GO2_PI_IP=$v;$p=Start-Process -FilePath $python -ArgumentList @($client,'--host',$v) -WorkingDirectory $workspace -PassThru; $status.Text='Окно управления запущено';$log.AppendText("`r`nЗапущен клиент, PID $($p.Id).`r`nW/S — вперёд/назад, A/D — поворот, Q/E — боком.`r`nВ записи нажмите «Начало записи», затем «Конец записи».`r`n")}catch{$status.Text='Ошибка: '+$_.Exception.Message;$log.AppendText("`r`n$($_.Exception.Message)`r`n")}})
$close=New-Object Windows.Forms.Button; $close.Text='Закрыть'; $close.Location='490,105'; $close.Width=120; $close.Height=45; $close.Add_Click({$form.Close()})
$form.Controls.AddRange(@($check,$run,$close))
[void]$form.ShowDialog()