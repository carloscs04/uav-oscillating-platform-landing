# Linux Environment Setup Tools

This directory contains setup scripts to configure Python virtual environments and ROS 2 workspaces on Linux operating systems.

---

## 1. `setup_python.sh`

### Purpose
Initializes a standardized Python virtual environment (`.venv`) for Linux developers and installs the codebase as an editable package.

### What It Does
- Checks if `.venv` exists at the project root; creates it if missing.
- Activates `.venv` and upgrades `pip` to the latest version.
- Installs required Python dependencies listed in `requirements.txt`.
- Executes `pip install -e .` at the codebase root. This mounts the codebase in **editable mode**, allowing imports (e.g., `from precision_landing...`) to work universally across all project subfolders without modifying `sys.path`.

### Usage
```bash
source tools/setup/linux/setup_python.sh
```

