#!/bin/bash
set -e

# Resolve repository root
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$ROOT_DIR"

echo "=================================================="
echo " Bootstrapping Raspberry Pi OS Environment"
echo " Target Path: $ROOT_DIR"
echo "=================================================="

# 1. Elevate to root if not running as root
if [ "$EUID" -ne 0 ]; then
    echo "[+] Requesting root privileges..."
    exec sudo "$0" "$@"
fi

# 2. Install necessary system dependencies (Works on physical RPi & Docker)
echo "[+] Updating package list and installing system tools..."
apt-get update -y
apt-get install -y \
    apt-utils \
    git \
    sudo \
    python3 \
    python3-venv \
    python3-pip \
    python3-full

# 3. Handle virtual environment creation/re-creation
if [ -d ".venv" ]; then
    if ! .venv/bin/python --version >/dev/null 2>&1; then
        echo "[!] Existing .venv architecture mismatch. Recreating..."
        rm -rf .venv
    fi
fi

if [ ! -d ".venv" ]; then
    echo "[+] Creating Python virtual environment (.venv)..."
    python3 -m venv .venv
fi

VENV_PYTHON="$ROOT_DIR/.venv/bin/python"

# 4. Install Python packages inside .venv
echo "[+] Upgrading pip..."
$VENV_PYTHON -m pip install --upgrade pip

if [ -f "requirements.txt" ]; then
    echo "[+] Installing requirements.txt dependencies..."
    $VENV_PYTHON -m pip install -r requirements.txt
fi

if [ -f "pyproject.toml" ] || [ -f "setup.py" ]; then
    echo "[+] Installing codebase in editable mode..."
    $VENV_PYTHON -m pip install -e .
fi

echo "=================================================="
echo " Environment Bootstrap Complete!"
echo " Python Version: $($VENV_PYTHON --version)"
echo "=================================================="