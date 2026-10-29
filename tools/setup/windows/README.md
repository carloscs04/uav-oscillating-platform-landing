# Windows Environment Setup Tools

This directory contains the setup script for Windows developers working on Python-based algorithms, testing, or high-level codebase development.

---

## `setup_python.bat`

### Purpose
Configures a standardized Python virtual environment on Windows, ensuring Windows developers use the exact same import structures as Linux developers without needing ROS 2 or Gazebo installed locally.

### OS Compatibility
* **Supported OS:** Windows 10 / 11 (Command Prompt or PowerShell).

### What It Does
1. Navigates relative to the project root directory using Windows path resolution (`%~dp0`).
2. Checks for an existing `.venv` directory at the project root; creates a new virtual environment if none exists.
3. Activates the virtual environment (`.venv\Scripts\activate.bat`).
4. Upgrades `pip` and installs dependencies from `requirements.txt`.
5. Executes `pip install -e .` to install the project in **editable mode**. This allows Windows users to use identical package import paths as Linux users without needing hardcoded `sys.path` modifications.

### Usage
Double-click `setup_python.bat` in Windows File Explorer, or execute it from Command Prompt / PowerShell:
```cmd
tools\setup\windows\setup_python.bat