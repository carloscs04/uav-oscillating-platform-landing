#!/bin/bash
set -e

# Get project root directory (3 levels up from tools/launcher/raspberry_pi)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "=== Launching UAV Main Script on Raspberry Pi ==="
echo "[+] Project Root: $ROOT_DIR"

# 1. Activate virtual environment
if [ -f "$ROOT_DIR/.venv/bin/activate" ]; then
    source "$ROOT_DIR/.venv/bin/activate"
    echo "[+] Virtual environment activated."
else
    echo "[!] Error: .venv not found at $ROOT_DIR/.venv"
    exit 1
fi

# 2. Run the main Python entrypoint
# (Adjust src/main.py if your entrypoint file has a different name)
echo "[+] Starting Python application..."
python3 "$ROOT_DIR/src/main.py"