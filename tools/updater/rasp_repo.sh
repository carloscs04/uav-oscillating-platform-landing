#!/bin/bash
set -e

# 1. Cambiado a conexión SSH para aprovechar la Deploy Key
REPO_URL="${1:-git@github.com:carloscs04/uav-oscillating-platform-landing.git}"
TARGET_DIR="/home/pi/uav-opl"

echo "=================================================="
echo " Syncing Production Codebase from GitHub"
echo " Target Directory: $TARGET_DIR"
echo "=================================================="

mkdir -p "$TARGET_DIR"

# Fetch to temporary staging folder
TMP_DIR=$(mktemp -d)
echo "[+] Fetching fresh production archive to staging..."
# Quitamos --depth 1 temporalmente para tener el historial completo y poder hacer push
git clone "$REPO_URL" "$TMP_DIR"

echo "[+] Transferring production files..."
cp -r "$TMP_DIR/src" "$TARGET_DIR/" 2>/dev/null || true
cp -r "$TMP_DIR/tools" "$TARGET_DIR/" 2>/dev/null || true
[ -f "$TMP_DIR/requirements.txt" ] && cp "$TMP_DIR/requirements.txt" "$TARGET_DIR/"
[ -f "$TMP_DIR/setup.py" ] && cp "$TMP_DIR/setup.py" "$TARGET_DIR/"
cp -r "$TMP_DIR/.git" "$TARGET_DIR/" 2>/dev/null || true

# Clean up staging area and enforce strict folder exclusion
rm -rf "$TMP_DIR"
rm -rf "$TARGET_DIR/ros2_ws"

echo "=================================================="
echo " Sync Complete! Clean production codebase ready."
echo "=================================================="