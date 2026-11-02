#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

cd "$ROOT_DIR"

export PYTHONUNBUFFERED=1
export UAV_MODE="REAL"

if [ -f ".venv/bin/activate" ]; then
    source .venv/bin/activate
else
    echo "[Error] .venv not found! Run tools/setup/raspberry_pi/setup_rasp.sh first."
    exit 1
fi

exec python -m src.main