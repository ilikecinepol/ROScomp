@echo off
cd /d "%~dp0"
if not exist ".venv\Scripts\python.exe" (
  echo Please follow the HTML guide to install Python and Pillow.
  pause
  exit /b 1
)
".venv\Scripts\python.exe" "work\go2-teleop\keyboard_client.py" --host 192.168.11.81
if errorlevel 1 pause
