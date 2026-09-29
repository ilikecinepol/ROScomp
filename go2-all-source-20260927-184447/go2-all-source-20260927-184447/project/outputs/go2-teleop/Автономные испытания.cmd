@echo off
setlocal
cd /d "%~dp0..\.."
"work\go2-perception\.venv\Scripts\python.exe" "work\go2-autonomy\training_panel.py"
if errorlevel 1 pause
endlocal
