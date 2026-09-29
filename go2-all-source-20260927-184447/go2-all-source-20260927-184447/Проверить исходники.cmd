@echo off
setlocal
cd /d "%~dp0"
where py >nul 2>nul
if errorlevel 1 (
  python tools\verify_bundle.py
) else (
  py -3 tools\verify_bundle.py
)
pause
endlocal
