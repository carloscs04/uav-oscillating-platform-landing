#!/bin/bash
set -e

# Project root = two levels up from src/launchers
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../.." && pwd )"

echo "=== Launching UAV Main Script on Raspberry Pi ==="
echo "[+] Project Root: $ROOT_DIR"

# 1. Activate virtual environment (repo/.venv first, then the folder above the repo)
VENV=""
for candidate in "$ROOT_DIR/.venv" "$ROOT_DIR/../.venv"; do
    if [ -f "$candidate/bin/activate" ]; then
        VENV="$candidate"
        break
    fi
done

if [ -z "$VENV" ]; then
    echo "[!] Error: .venv not found in $ROOT_DIR/.venv or $ROOT_DIR/../.venv"
    exit 1
fi
source "$VENV/bin/activate"
echo "[+] Virtual environment activated: $VENV"

# 2. Power check: undervoltage makes USB devices (camera, Pixhawk serial) drop mid-flight
if command -v vcgencmd &> /dev/null; then
    echo "[+] Power: $(vcgencmd get_throttled)  (anything but 0x0 = undervoltage/throttling)"
fi

# 3. Run the main Python entrypoint
cd "$ROOT_DIR"
export PYTHONUNBUFFERED=1
echo "[+] Starting Python application..."
exec python3 "$ROOT_DIR/src/main.py"
