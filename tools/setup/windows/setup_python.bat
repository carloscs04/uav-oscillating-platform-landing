@echo off

REM Get root directory relative to script (3 levels up from tools\setup\windows)
set "SCRIPT_DIR=%~dp0"
cd /d "%SCRIPT_DIR%..\..\.."
set "ROOT_DIR=%CD%"

echo === Setting up Python Environment Windows ===

REM 1. Create .venv explicitly using Python 3.12
if not exist "%ROOT_DIR%\.venv" (
    echo Creating .venv with Python 3.12...
    py -3.12 -m venv "%ROOT_DIR%\.venv"
) else (
    echo Existing .venv directory found.
)

REM 2. Activate virtual environment into current terminal session
echo Activating virtual environment...
call "%ROOT_DIR%\.venv\Scripts\activate.bat"

REM 3. Upgrade pip
echo Upgrading pip...
python -m pip install --upgrade pip

REM 4. Install requirements if file exists
if exist "%ROOT_DIR%\requirements.txt" (
    echo Installing Python dependencies from requirements.txt...
    pip install -r "%ROOT_DIR%\requirements.txt"
) else (
    echo requirements.txt not found, skipping...
)

REM 5. Install codebase in editable mode
if exist "%ROOT_DIR%\setup.py" (
    echo Installing project root in editable mode...
    pip install -e . --config-settings editable_mode=compat
) else (
    echo Root setup.py not found, skipping...
)

REM 6. Clean build artifacts (.egg-info)
echo Cleaning up .egg-info directories...
for /d %%i in ("%ROOT_DIR%\*.egg-info") do rmdir /s /q "%%i"

echo === Python Setup Complete! Virtual environment is active. ===