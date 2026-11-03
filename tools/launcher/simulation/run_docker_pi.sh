#!/bin/bash
set -e

CONTAINER_NAME="simulated_rpi5"

echo "=================================================="
echo " Starting Isolated Raspberry Pi OS (ARM64 Docker)"
echo "=================================================="

# 1. Wipe previous container instance
docker rm -f $CONTAINER_NAME >/dev/null 2>&1 || true

# 2. Boot blank ARM64 OS (No local files mounted or copied)
docker run -d \
  --name $CONTAINER_NAME \
  --platform linux/arm64 \
  -p 5005:5005/udp \
  -p 8080:8080 \
  debian:bookworm tail -f /dev/null

echo "[+] Bare Raspberry Pi OS running."
echo "[+] Entering Pi shell. Run setup script next!"
echo "=================================================="

# 3. Enter clean Raspberry Pi terminal shell
docker exec -it $CONTAINER_NAME bash