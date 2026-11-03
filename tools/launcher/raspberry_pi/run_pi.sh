#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$ROOT_DIR"

echo "=================================================="
echo " Starting Raspberry Pi Application"
echo "=================================================="

# 1. Call dedicated updater to pull newest GitHub code
UPDATER_SCRIPT="$ROOT_DIR/tools/updater/rasp_repo.sh"
if [ -f "$UPDATER_SCRIPT" ]; then
    bash "$UPDATER_SCRIPT"
fi

# 2. Activate virtual environment and run main entrypoint
if [ -d ".venv" ]; then
    source .venv/bin/activate
    echo "[+] Launching application..."
    python3 src/main.py
else
    echo "[!] Virtual environment missing. Please run setup_rasp.sh first."
    exit 1
fi