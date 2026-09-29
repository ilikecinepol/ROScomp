# Вход на назначенную Raspberry Pi. Команды роботу не отправляются.
$ErrorActionPreference = 'Stop'
$taskKey = Join-Path $env:USERPROFILE '.ssh\go2-pi-0008\id_ed25519'
$taskKnownHosts = 'C:\Users\VladO\Documents\Codex\2026-09-09\new-chat\work\go2-remote-access\known_hosts'
if (-not (Test-Path -LiteralPath $taskKey)) {
    throw 'Secure SSH key is missing. See the connection notes.'
}
Write-Host 'OpenVPN profile robot_8 must be Connected.'
& ssh -o IdentitiesOnly=yes -o StrictHostKeyChecking=yes -o ConnectTimeout=10 -o ServerAliveInterval=15 -o ServerAliveCountMax=2 -o "UserKnownHostsFile=$taskKnownHosts" -i $taskKey ubuntu@192.168.11.82
exit $LASTEXITCODE
