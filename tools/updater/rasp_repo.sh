#!/bin/bash
set -e

REPO_URL="${1:-git@github.com:carloscs04/uav-oscillating-platform-landing.git}"
TARGET_DIR="/home/pi/uav-opl"

echo "=================================================="
echo " Syncing Production Codebase (Sparse) from GitHub"
echo "=================================================="

# Initialize the repository if it does not exist
if [ ! -d "$TARGET_DIR/.git" ]; then
    echo "[+] Cloning repository with sparse-checkout enabled..."
    # Clone the basic structure without downloading files yet
    git clone --filter=blob:none --sparse "$REPO_URL" "$TARGET_DIR"
    cd "$TARGET_DIR"
    
    # Define EXACTLY which folders/files should exist on the Raspberry Pi
    git sparse-checkout set src tools requirements.txt setup.py
else
    echo "[+] Repository found. Pulling latest changes..."
    # If it already exists, simply pull the latest changes
    cd "$TARGET_DIR"
    git pull origin main
fi

# Clean up local build directories (this does not affect Git)
rm -rf "$TARGET_DIR/ros2_ws"

echo "=================================================="
echo " Sync Complete! Sparse production codebase ready."
echo "=================================================="