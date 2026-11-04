@echo off
setlocal enabledelayedexpansion

set "SCRIPT_DIR=%~dp0"
set "GCS_DIR=%SCRIPT_DIR%.."
set "ROOT_DIR=%GCS_DIR%\.."

set "MODE=%~1"
if "%MODE%"=="" set "MODE=wifi"
set "MODE=%MODE:--=%"

if exist "%ROOT_DIR%\.venv\Scripts\activate.bat" (
    call "%ROOT_DIR%\.venv\Scripts\activate.bat"
)

set "PYTHONPATH=%GCS_DIR%;%PYTHONPATH%"

python "%GCS_DIR%\panel\master_panel.py" --mode %MODE%
endlocal