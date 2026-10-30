#!/bin/bash
set -e

# Get project root directory relative to this script location (3 levels up)
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

echo "=== Setting up Python Environment (Linux) ==="
cd "$ROOT_DIR"

# 1. Create .venv if it does not exist
VENV_DIR="$ROOT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Virtual environment not found. Creating .venv..."
    python3 -m venv "$VENV_DIR"
else
    echo "Existing .venv directory found."
fi

# 2. Enforce strict isolation (disable system site packages)
if [ -f "$ROOT_DIR/.venv/pyvenv.cfg" ]; then
    sed -i 's/include-system-site-packages = true/include-system-site-packages = false/g' "$ROOT_DIR/.venv/pyvenv.cfg"
fi

# 3. Activate virtual environment
echo "Activating virtual environment (.venv)..."
source "$VENV_DIR/bin/activate"

# 4. Upgrade pip
echo "Upgrading pip..."
python3 -m pip install --upgrade pip

# 5. Install requirements if file exists
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies from requirements.txt..."
    pip install -r requirements.txt
else
    echo "requirements.txt not found, skipping..."
fi

# 6. Install codebase in editable mode
if [ -f "setup.py" ]; then
    echo "Installing project root in editable mode (pip install -e .)..."
    pip install -e . --config-settings editable_mode=compat
else
    echo "Root setup.py not found, skipping..."
fi

# 7. Clean build artifacts (.egg-info)
echo "Cleaning up .egg-info directories..."
rm -rf *.egg-info

echo "=== Python Setup Complete! Virtual environment is active. ==="
