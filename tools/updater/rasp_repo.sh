#!/bin/bash
set -e

REPO_URL="${1:-https://github.com/carloscs04/uav-oscillating-platform-landing.git}"
TARGET_DIR="/home/pi/codebase"

echo "=================================================="
echo " Syncing Production Codebase from GitHub"
echo " Target Directory: $TARGET_DIR"
echo "=================================================="

mkdir -p "$TARGET_DIR"
cd "$TARGET_DIR"

# 1. Initial sparse clone if repository does not exist
if [ ! -d ".git" ]; then
    echo "[+] Initializing sparse clone from GitHub..."
    git clone --filter=blob:none --no-checkout "$REPO_URL" .
fi

# 2. Enforce non-cone sparse checkout (pulls ONLY src, tools, requirements.txt, setup.py)
echo "[+] Setting production sparse-checkout patterns..."
git sparse-checkout set --no-cone "src/*" "tools/*" "requirements.txt" "setup.py"
git checkout main || git checkout master

echo "=================================================="
echo " GitHub Sync Complete!"
echo "=================================================="