#!/bin/bash
set -e

# Get project root directory relative to this script location (4 levels up)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../../.." && pwd )"

# Configuration variables
INTERFACE="${1:-wlx8c773b0263f6}"
SSID="project_uav_opl"
PASSWORD="project_uav_opl"

echo "=== Setting up Wi-Fi Hotspot on Linux ==="
echo "[+] Root directory: $ROOT_DIR"
echo "[+] Using interface: $INTERFACE"

# 1. Remove existing Hotspot connection if present
echo "[+] Cleaning up previous 'Hotspot' profiles..."
sudo nmcli connection delete Hotspot 2>/dev/null || true

# 2. Create Hotspot profile bound to the specified USB interface
echo "[+] Creating Hotspot profile for $INTERFACE..."
sudo nmcli connection add type wifi ifname "$INTERFACE" mode ap con-name Hotspot ssid "$SSID"
sudo nmcli connection modify Hotspot 802-11-wireless-security.key-mgmt wpa-psk
sudo nmcli connection modify Hotspot 802-11-wireless-security.psk "$PASSWORD"
sudo nmcli connection modify Hotspot ipv4.method shared

# 3. Enable autoconnect on interface detection
echo "[+] Configuring autoconnect rules..."
sudo nmcli connection modify Hotspot connection.autoconnect yes
sudo nmcli connection modify Hotspot connection.autoconnect-priority 10

# 4. Activate Hotspot
echo "[+] Starting network '$SSID'..."
sudo nmcli connection up Hotspot

echo "=== Hotspot setup completed successfully ==="