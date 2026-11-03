#!/bin/bash
set -e

REPO_URL="${1:-https://github.com/carloscs04/uav-oscillating-platform-landing.git}"
TARGET_DIR="/home/pi/codebase"

echo "=================================================="
echo " Syncing Production Codebase from GitHub"
echo " Target Directory: $TARGET_DIR"
echo "=================================================="

mkdir -p "$TARGET_DIR"

# 1. Fetch to temporary staging folder
TMP_DIR=$(mktemp -d)
echo "[+] Fetching fresh production archive to staging..."
git clone --depth 1 "$REPO_URL" "$TMP_DIR"

# 2. Copy ONLY essential production assets
echo "[+] Transferring production files..."
cp -r "$TMP_DIR/src" "$TARGET_DIR/" 2>/dev/null || true
cp -r "$TMP_DIR/tools" "$TARGET_DIR/" 2>/dev/null || true
[ -f "$TMP_DIR/requirements.txt" ] && cp "$TMP_DIR/requirements.txt" "$TARGET_DIR/"
[ -f "$TMP_DIR/setup.py" ] && cp "$TMP_DIR/setup.py" "$TARGET_DIR/"
[ -f "$TMP_DIR/pyproject.toml" ] && cp "$TMP_DIR/pyproject.toml" "$TARGET_DIR/"

# 3. Clean up staging area and enforce strict folder exclusion
rm -rf "$TMP_DIR"
rm -rf "$TARGET_DIR/.git" "$TARGET_DIR/ros2_ws"

echo "=================================================="
echo " Sync Complete! Clean production codebase ready."
echo " Verified: No .git history | No ros2_ws"
echo "=================================================="