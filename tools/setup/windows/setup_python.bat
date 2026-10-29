@echo off
setlocal enabledelayedexpansion

:: Get root directory relative to script (3 levels up from tools\setup\windows)
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%..\..\.."
set "ROOT_DIR=%CD%"

echo === Setting up Python Environment (Windows) ===

:: 1. Create .venv if it does not exist
if not exist "%ROOT_DIR%\.venv" (
    echo Virtual environment not found. Creating .venv...
    python -m venv "%ROOT_DIR%\.venv"
) else (
    echo Existing .venv directory found.
)

:: 2. Activate virtual environment
echo Activating virtual environment (.venv)...
call "%ROOT_DIR%\.venv\Scripts\activate.bat"

:: 3. Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip

:: 4. Install requirements if file exists
if exist "%ROOT_DIR%\requirements.txt" (
    echo Installing Python dependencies from requirements.txt...
    pip install -r "%ROOT_DIR%\requirements.txt"
) else (
    echo requirements.txt not found, skipping...
)

:: 5. Install codebase in editable mode
if exist "%ROOT_DIR%\setup.py" (
    echo Installing project root in editable mode (pip install -e .)...
    pip install -e .
) else (
    echo Root setup.py not found, skipping...
)

echo === Python Setup Complete! Virtual environment is active. ===
pause