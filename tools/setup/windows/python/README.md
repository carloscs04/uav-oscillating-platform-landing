# Windows Environment Setup Tools

This directory contains the setup script for Windows developers working on Python-based algorithms, testing, or high-level codebase development.

---

## `setup_python.bat`

### Purpose
Configures a standardized Python virtual environment on Windows, ensuring Windows developers use the exact same import structures as Linux developers without needing ROS 2 or Gazebo installed locally.

### OS Compatibility
- **Supported OS:** Windows 10 / 11 (Command Prompt or PowerShell).

### What It Does
1. Navigates relative to the project root directory using Windows path resolution (`%~dp0`).
2. Checks for an existing `.venv` directory at the project root; creates a new virtual environment if none exists.
3. Activates the virtual environment (`.venv\Scripts\activate.bat`).
4. Upgrades `pip` and installs dependencies from `requirements.txt`.
5. Executes `pip install -e . --config-settings editable_mode=compat` to install the project in **portable editable mode**. This creates standard `.pth` path links inside `.venv`, ensuring identical package import paths as Linux without hardcoded `sys.path` or `PYTHONPATH` modifications.

### Usage
Execute the script from the VS Code integrated terminal (preferred), PowerShell, or by double-clicking `setup_python.bat` in Windows File Explorer.
```cmd
tools\setup\windows\setup_python.bat
```

**Note on Double-Clicking:** Double-clicking the script runs the setup inside a temporary Command Prompt window. Once the script finishes and the window closes, the temporary session ends.

### Daily Development Workflow
After running the setup script once, activate `.venv` whenever you open a new terminal session for daily work:

- **Command Prompt (CMD):**
  ```cmd
  .venv\Scripts\activate.bat
  ```

- **PowerShell:**
  ```powershell
  .\.venv\Scripts\Activate.ps1
  ```