# Installation & Provisioning Guide

Follow these instructions to provision a fresh **Raspberry Pi 5** or **ARM64 Docker Container**.

---

## First-Time System Bootstrap

On a clean Raspberry Pi OS image or a bare Docker container, Git is not yet installed and local repository files do not exist. Run the commands below to execute the initial setup.

### Interactive Installation

Run these commands inside your Raspberry Pi terminal (via SSH/local terminal) or inside the active Docker shell:

```bash
# 1. Install system utilities and navigate to codebase directory
SUDO=$(command -v sudo || true)
$SUDO apt-get update &&$SUDO apt-get install -y git curl
mkdir -p /home/pi/codebase && cd /home/pi/codebase

# 2. Download updater logic and execute production sparse sync
curl -sSL --create-dirs [https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh](https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh) -o tools/updater/rasp_repo.sh
chmod +x tools/updater/rasp_repo.sh
bash tools/updater/rasp_repo.sh

# 3. Execute system bootstrap (apt packages, Python .venv, systemd service)
./tools/setup/raspberry_pi/setup_rasp.sh
```

## Clean Reset & Reinstallation (Docker Simulator)

If you encounter an incomplete installation or need to start completely fresh:

* **Reset inside active Docker shell:**
  ```bash
  rm -rf /home/pi/codebase/* /home/pi/codebase/.* 2>/dev/null || true
  ```

* **Full container wipe (from host PC terminal):**
  ```bash
  docker rm -f simulated_rpi5 && ./tools/launcher/simulation/run_docker_pi.sh
  ```