#!/bin/bash
set -e

# Get project root directory relative to this script
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$SCRIPT_DIR/.."

echo "=== Setting up Python Environment ==="
cd "$ROOT_DIR"

# 1. Create .venv if it does not exist
VENV_DIR="$ROOT_DIR/.venv"
if [ ! -d "$VENV_DIR" ]; then
    echo "Virtual environment not found. Creating .venv..."
    python3 -m venv "$VENV_DIR"
else
    echo "Existing .venv directory found."
fi

# 2. Activate virtual environment
echo "Activating virtual environment (.venv)..."
source "$VENV_DIR/bin/activate"

# 3. Upgrade pip
echo "Upgrading pip..."
pip install --upgrade pip

# 4. Install requirements if file exists
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies from requirements.txt..."
    pip install -r requirements.txt
else
    echo "requirements.txt not found, skipping..."
fi

# 5. Install codebase in editable mode
if [ -f "setup.py" ]; then
    echo "Installing project root in editable mode (pip install -e .)..."
    pip install -e .
else
    echo "Root setup.py not found, skipping..."
fi

echo "=== Python Setup Complete! Virtual environment is active. ==="
