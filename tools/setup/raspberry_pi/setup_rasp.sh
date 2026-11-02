#!/bin/bash
set -e

# Resolve repository root directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$ROOT_DIR"

echo "=================================================="
echo " Bootstrapping Raspberry Pi OS System"
echo " Target Directory: $ROOT_DIR"
echo "=================================================="

# 1. Elevate permissions if superuser is required
if [ "$EUID" -ne 0 ]; then
    echo "[+] Elevating to root privileges..."
    exec sudo "$0" "$@"
fi

# 2. Install base OS packages via apt
echo "[+] Installing system packages (apt)..."
apt-get update -y
apt-get install -y apt-utils git sudo python3 python3-venv python3-pip python3-full

# 3. Call dedicated updater to pull/sync repository from GitHub
UPDATER_SCRIPT="$ROOT_DIR/tools/updater/rasp_repo.sh"
if [ -f "$UPDATER_SCRIPT" ]; then
    echo "[+] Executing repo sync via rasp_repo.sh..."
    bash "$UPDATER_SCRIPT"
else
    echo "[!] Warning: rasp_repo.sh not found at $UPDATER_SCRIPT. Skipping Git sync."
fi

# 4. Create Python virtual environment (.venv)
if [ ! -d ".venv" ]; then
    echo "[+] Creating Python virtual environment (.venv)..."
    python3 -m venv .venv
fi

VENV_PYTHON="$ROOT_DIR/.venv/bin/python"
$VENV_PYTHON -m pip install --upgrade pip

if [ -f "requirements.txt" ]; then
    $VENV_PYTHON -m pip install -r requirements.txt
fi

if [ -f "pyproject.toml" ] || [ -f "setup.py" ]; then
    $VENV_PYTHON -m pip install -e .
fi

# 5. Register systemd auto-start service
SERVICE_SRC="$ROOT_DIR/tools/launcher/raspberry_pi/drone.service"
if [ -f "$SERVICE_SRC" ]; then
    echo "[+] Registering drone.service..."
    cp "$SERVICE_SRC" /etc/systemd/system/
    [ -x "$(command -v systemctl)" ] && systemctl daemon-reload || true
fi

echo "=================================================="
echo " System Bootstrap Complete!"
echo "=================================================="