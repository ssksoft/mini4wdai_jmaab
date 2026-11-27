@echo off
rem Headless (MuJoCo) simulator.  Examples:  headless check  /  headless lc_brake --plot  /  headless sweep lc_brake --param RUN_DUTY=0.6,0.7
setlocal
set "VPY=%~dp0.venv\Scripts\python.exe"
if not exist "%VPY%" (
  echo [headless] .venv not found. Run setup_headless.bat first.
  exit /b 1
)
set PYTHONUTF8=1
"%VPY%" "%~dp0tools\headless.py" %*
