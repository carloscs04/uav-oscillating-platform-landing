# First-Time System Setup & Provisioning

Execute these commands inside your Raspberry Pi terminal or inside the active Docker shell.

---

## Installation Steps

### Option A: Interactive Installation (Recommended)

```bash
# 1. Install system utilities and create target directory
SUDO=$(command -v sudo || true)
$SUDO apt-get update &&$SUDO apt-get install -y git curl
mkdir -p /home/pi/codebase && cd /home/pi/codebase

# 2. Download updater logic to /tmp and execute sparse sync
curl -sSL [https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh](https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh) -o /tmp/rasp_repo.sh
chmod +x /tmp/rasp_repo.sh
/tmp/rasp_repo.sh

# 3. Execute system bootstrap (apt packages, Python .venv, systemd service)
./tools/setup/raspberry_pi/setup_rasp.sh
```

## Clean Reset & Reinstallation (Docker Simulation)

To wipe the codebase and restore the system to a fresh state before reinstalling:

* **Inside Docker or RPi Terminal:**
  ```bash
  SUDO=$(command -v sudo || true)
  [ -x "$(command -v systemctl)" ] && $SUDO systemctl stop drone.service 2>/dev/null || true
  [ -x "$(command -v systemctl)" ] && $SUDO systemctl disable drone.service 2>/dev/null \vert{}\vert{} true$SUDO rm -f /etc/systemd/system/drone.service
  [ -x "$(command -v systemctl)" ] && $SUDO systemctl daemon-reload \vert{}\vert{} true$SUDO rm -rf /home/pi/codebase
  ```

* **Full Docker Container Wipe (Host PC Terminal):**
  ```bash
  docker rm -fv simulated_rpi5 2>/dev/null || true && ./tools/launcher/simulation/run_docker_pi.sh
  ```


