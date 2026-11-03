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

# 1. Initialize git repo if not present (works in non-empty folders)
if [ ! -d ".git" ]; then
    echo "[+] Initializing Git repo in target directory..."
    git init
    git remote add origin "$REPO_URL" 2>/dev/null || git remote set-url origin "$REPO_URL"
fi

# 2. Set sparse-checkout patterns (--no-cone mode)
echo "[+] Setting production sparse-checkout patterns..."
git config core.sparseCheckout true
git sparse-checkout set --no-cone "src/*" "tools/*" "requirements.txt" "setup.py"

# 3. Fetch latest main branch and force checkout
echo "[+] Fetching latest codebase from GitHub..."
git fetch --filter=blob:none origin main
git checkout -f main

echo "=================================================="
echo " GitHub Sync Complete!"
echo "=================================================="