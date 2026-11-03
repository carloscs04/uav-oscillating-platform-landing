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

# 1. Initialize git repo if not present (handles existing directories safely)
if [ ! -d ".git" ]; then
    echo "[+] Initializing Git repo..."
    git init
    git remote add origin "$REPO_URL" 2>/dev/null || git remote set-url origin "$REPO_URL"
else
    echo "[+] Updating remote URL..."
    git remote set-url origin "$REPO_URL" 2>/dev/null || true
fi

# 2. Enforce strict sparse-checkout (pulls ONLY src, tools, requirements.txt, setup.py)
echo "[+] Configuring production sparse-checkout patterns..."
git config core.sparseCheckout true
git sparse-checkout set --no-cone \
    "src/*" \
    "tools/*" \
    "requirements.txt" \
    "setup.py"

# 3. Fetch latest main branch and apply checkout
echo "[+] Fetching latest codebase from GitHub..."
git fetch --filter=blob:none origin main
git checkout -f main

# 4. Re-apply sparse rules and purge excluded folders (removes ros2_ws, keeps .venv)
echo "[+] Purging excluded folders (ros2_ws) and applying sparse filter..."
git sparse-checkout reapply
git clean -fd -e .venv

echo "=================================================="
echo " GitHub Sync Complete!"
echo " Production files ready (ros2_ws excluded)."
echo "=================================================="