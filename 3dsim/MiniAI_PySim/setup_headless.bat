@echo off
rem Sets up the headless (MuJoCo) simulator: .venv + packages + environment check
setlocal
cd /d "%~dp0"
set "PY=python"
where py >nul 2>nul
if not errorlevel 1 set "PY=py -3"
set PYTHONUTF8=1
%PY% tools\setup_headless.py %*
pause
