@echo off
set "PI_USER=carloscs"
set "PI_IP=10.42.1.51"
set "REMOTE_SCRIPT=~/codebase/tools/launcher/raspberry_pi/run_py.sh"

echo === Ground Station: Connecting to UAV Raspberry Pi ===

:: Check network ping
ping -n 2 %PI_IP% >nul 2>&1
if %errorlevel% neq 0 (
    echo [!] Error: Cannot reach %PI_IP%. Ensure your USB Wi-Fi is connected.
    pause
    exit /b 1
)

echo [+] Host %PI_IP% is reachable. Connecting via SSH...
ssh -t %PI_USER%@%PI_IP% "bash --login %REMOTE_SCRIPT%"
pause