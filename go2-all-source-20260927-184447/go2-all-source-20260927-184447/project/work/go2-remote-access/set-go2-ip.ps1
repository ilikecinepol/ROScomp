param(
    [Parameter(Mandatory=$true)][ValidatePattern('^(?:\d{1,3}\.){3}\d{1,3}$')]
    [string]$Ip
)
$parts = $Ip.Split('.') | ForEach-Object { [int]$_ }
if ($parts | Where-Object { $_ -gt 255 }) { throw "Некорректный IPv4-адрес: $Ip" }
$env:GO2_PI_IP = $Ip
Write-Host "GO2_PI_IP=$env:GO2_PI_IP (только это окно PowerShell)"
Write-Host "Проверка SSH:"
Test-NetConnection $env:GO2_PI_IP -Port 22
Write-Host "Подключение:"
ssh -o ConnectTimeout=10 -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes `
  -i "$env:USERPROFILE\.ssh\go2-pi-0008\id_ed25519" "ubuntu@$env:GO2_PI_IP"
