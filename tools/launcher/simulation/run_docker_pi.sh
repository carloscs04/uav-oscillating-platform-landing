#!/bin/bash
set -e

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
ROOT_DIR="$( cd "$SCRIPT_DIR/../../.." && pwd )"

CONTAINER_NAME="simulated_rpi5"

echo "=================================================="
echo " Launching Simulated Raspberry Pi 5 (ARM64 Docker)"
echo "=================================================="

# Stop previous instance if running
docker rm -f $CONTAINER_NAME >/dev/null 2>&1 || true

# Boot plain ARM64 Debian container
docker run -d \
  --name $CONTAINER_NAME \
  --platform linux/arm64 \
  -v "$ROOT_DIR":/home/pi/codebase \
  -p 5005:5005/udp \
  -p 8080:8080 \
  debian:bookworm tail -f /dev/null

echo "[+] Entering Pi shell. Run setup script next:"
echo "    ./tools/setup/raspberry_pi/setup_rasp.sh"
echo "=================================================="

docker exec -it -w /home/pi/codebase $CONTAINER_NAME bash