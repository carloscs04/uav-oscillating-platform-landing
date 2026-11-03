@echo off
setlocal enabledelayedexpansion

REM Get root directory relative to script (4 levels up from tools\setup\windows\wifi_usb)
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%..\..\..\.."
set "ROOT_DIR=%CD%"

set "SSID=project_uav_opl"
set "PASSWORD=project_uav_opl"

echo === Setting up Wi-Fi Hotspot on Windows ===
echo [+] Root directory: %ROOT_DIR%

REM 1. Configure hosted network
echo [+] Configuring hosted network settings...
netsh wlan set hostednetwork mode=allow ssid="%SSID%" key="%PASSWORD%"

REM 2. Start hosted network
echo [+] Starting Hotspot...
netsh wlan start hostednetwork

echo.
echo === Hotspot '%SSID%' started successfully ===
echo [!] Note: Ensure this script is executed with Administrator privileges.
pause