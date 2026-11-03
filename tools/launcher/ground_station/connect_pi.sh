#!/bin/bash

# Configuration variables
PI_USER="carloscs"             # Change to your Pi username if different
PI_IP="10.42.1.51"             # Raspberry Pi IP address
TARGET_SSID="project_uav_opl"
REMOTE_SCRIPT="~/codebase/tools/launcher/raspberry_pi/run_py.sh" # Adjust remote path if needed

echo "=== Ground Station: Connecting to UAV Raspberry Pi ==="

# 1. Verify Wi-Fi network connection (Linux)
if command -v nmcli &> /dev/null; then
    CURRENT_SSID=$(nmcli -t -f active,ssid dev wifi | grep '^yes' | cut -d: -f2)
    echo "[+] Current Wi-Fi Network: $CURRENT_SSID"
    if [ "$CURRENT_SSID" != "$TARGET_SSID" ]; then
        echo "[!] Warning: Not connected to '$TARGET_SSID'. Attempting to connect..."
        nmcli dev wifi connect "$TARGET_SSID" || echo "[!] Could not switch Wi-Fi automatically."
    fi
fi

# 2. Check reachability via Ping
echo "[+] Checking connection to Raspberry Pi ($PI_IP)..."
if ping -c 2 -W 2 "$PI_IP" > /dev/null 2>&1; then
    echo "[+] Host $PI_IP is reachable."
else
    echo "[!] Error: Cannot reach $PI_IP. Ensure the Pi is connected to $TARGET_SSID."
    exit 1
fi

# 3. Connect via SSH and run remote script
echo "[+] Executing remote launcher on Raspberry Pi..."
ssh -t "${PI_USER}@${PI_IP}" "bash --login ${REMOTE_SCRIPT}"