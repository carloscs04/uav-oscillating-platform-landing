#!/bin/bash
set -e

REPO_URL="${1:-git@github.com:carloscs04/uav-oscillating-platform-landing.git}"
TARGET_DIR="/home/pi/uav-opl"

echo "=================================================="
echo " Syncing Production (Sparse) from GitHub"
echo "=================================================="

# Initialize the repository if it does not exist
if [ ! -d "$TARGET_DIR/.git" ]; then
    echo "[+] Cloning repository with sparse-checkout enabled..."
    
    # Clone the basic structure without downloading files yet
    git clone --filter=blob:none --sparse "$REPO_URL" "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    # Disable cone mode and set exact files/folders to include
    git sparse-checkout set --no-cone /src/ /tools/ /tests/ /requirements.txt /setup.py
else
    echo "[+] Repository found. Pulling latest changes..."
    cd "$TARGET_DIR"
    
    # Ensure sparse-checkout is strictly set to our required files even on existing repos
    git sparse-checkout set --no-cone /src/ /tools/ /requirements.txt /setup.py
    
    # Pull latest updates
    git pull origin main
fi

# Clean up local build directories (this does not affect Git)
echo "[+] Cleaning up local workspace directories..."
rm -rf "$TARGET_DIR/ros2_ws"

echo "=================================================="
echo " Sync Complete! Sparse production codebase ready."
echo "=================================================="