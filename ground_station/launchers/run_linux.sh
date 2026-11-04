#!/bin/bash
set -e

# Resolve paths
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
GCS_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
ROOT_DIR="$(cd "$GCS_DIR/.." && pwd)"

# Default to --wifi if no argument provided
MODE="${1:---school}"

# Strip leading dashes if user wrote --usb vs usb
MODE_CLEAN="${MODE#--}"

# Activate python environment if present
if [ -d "$ROOT_DIR/.venv" ]; then
    source "$ROOT_DIR/.venv/bin/activate"
fi

export PYTHONPATH="$GCS_DIR:$PYTHONPATH"

# Launch Master Panel CLI immediately
exec python3 "$GCS_DIR/panel/master_panel.py" --mode "$MODE_CLEAN"