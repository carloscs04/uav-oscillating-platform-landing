# First-Time System Setup & Provisioning

Execute these commands inside your physical Raspberry Pi terminal or inside an active Docker container shell.

---

## 1. System Installation & Provisioning

### Option A: Interactive Step-by-Step Installation (Recommended)

```bash
# 1. Install base OS package dependencies and prepare project target directory
SUDO=$(command -v sudo || true)
$SUDO apt-get update &&$SUDO apt-get install -y git curl
mkdir -p /home/pi/uav-opl && cd /home/pi/uav-opl

# 2. Fetch lightweight updater logic to /tmp and pull clean production codebase
curl -sSL https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh -o /tmp/rasp_repo.sh
chmod +x /tmp/rasp_repo.sh
/tmp/rasp_repo.sh

# 3. Execute system bootstrap (Python .venv, editable compat install, systemd service)
./tools/setup/raspberry_pi/setup_rasp.sh
```

---

### Option B: One-Line Automated Installation

For rapid deployment on fresh Raspberry Pi OS installs or automated Docker setups:

```bash
SUDO=$(command -v sudo || true) && $SUDO apt-get update &&$SUDO apt-get install -y git curl && mkdir -p /home/pi/codebase && cd /home/pi/codebase && curl -sSL [https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh](https://raw.githubusercontent.com/carloscs04/uav-oscillating-platform-landing/main/tools/updater/rasp_repo.sh) -o /tmp/rasp_repo.sh && chmod +x /tmp/rasp_repo.sh && /tmp/rasp_repo.sh && ./tools/setup/raspberry_pi/setup_rasp.sh
```

---

## 2. Physical Raspberry Pi System Purge & Clean Reset

To stop active services, remove all project files, and return an existing Raspberry Pi back to a fresh state before reinstalling:

```bash
# Stop and disable systemd service
SUDO=$(command -v sudo || true)
[ -x "$(command -v systemctl)" ] && $SUDO systemctl stop drone.service 2>/dev/null || true
[ -x "$(command -v systemctl)" ] && $SUDO systemctl disable drone.service 2>/dev/null \vert{}\vert{} true$SUDO rm -f /etc/systemd/system/drone.service
[ -x "$(command -v systemctl)" ] && $SUDO systemctl daemon-reload 2>/dev/null || true

# Purge codebase, venv, and temporary setup scripts
$SUDO rm -rf /home/pi/codebase /tmp/rasp_repo.sh
```

---

## 3. Docker Simulator Container Reset

To destroy your active container instance on the host machine and launch a completely bare Raspberry Pi OS simulator from scratch:

```bash
docker rm -fv simulated_rpi5 2>/dev/null || true && ./tools/launcher/simulation/run_docker_pi.sh
```

