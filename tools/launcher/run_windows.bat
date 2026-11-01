@echo off
setlocal

# Get root directory (1 level up from launch\)
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%.."
set "ROOT_DIR=%CD%"

echo ==================================================
echo  Starting UAV Hardware Connection (Windows Mode)
echo ==================================================

if not exist "%ROOT_DIR%\.venv\Scripts\activate.bat" (
    echo Error: Virtual environment not found. Run tools\setup\windows\setup_python.bat first.
    pause
    exit /b 1
)

call "%ROOT_DIR%\.venv\Scripts\activate.bat"

echo Launching MAVLink Hardware Interface...
python -m src.interfaces.hardware.mavlink.launcher --port COM3 --baud 57600

pause
endlocal