@echo off
setlocal
set "FZ=C:\Program Files\FileZilla FTP Client\filezilla.exe"
if not exist "%FZ%" (
  echo FileZilla not found.
  pause
  exit /b 1
)
if not exist "%~dp0recordings" mkdir "%~dp0recordings"
start "" "%FZ%" --site="0/Go2-Pi-0008" --local="%~dp0recordings"
endlocal
