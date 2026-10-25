@echo off
title Project Backup to OneDrive

:: Automatically detect the root project folder (Jarvis) 2 levels up from codebase/tools/
set SCRIPT_DIR=%~dp0
for %%I in ("%SCRIPT_DIR%..\..") do set PROJECT_ROOT=%%~fI
for %%I in ("%PROJECT_ROOT%") do set PROJECT_NAME=%%~nxI

:: Set Rclone executable (Use "rclone" if in system PATH, or set absolute path to executable)
set RCLONE=rclone
set DESTINATION=onedrive:Projects/%PROJECT_NAME%

:: Exclusions (Prevents sensitive keys, virtual environments, and heavy build artifacts from uploading)
set EXCLUDES=--exclude ".env*" --exclude "*venv/**" --exclude ".git/**" --exclude "__pycache__/**" --exclude "node_modules/**" --exclude "logs/**" --exclude "temp/**" --exclude "build/**" --exclude "install/**" --exclude "log/**" --exclude "*.tmp" --exclude "*.log"

set OPTIONS=--fast-list --timeout 30s --contimeout 15s --retries 2 -P

echo ==========================================
echo RESPALDO DEL PROYECTO: %PROJECT_NAME%
echo Origen: %PROJECT_ROOT%
echo Destino: %DESTINATION%
echo ==========================================
echo.

%RCLONE% sync "%PROJECT_ROOT%" "%DESTINATION%" %EXCLUDES% %OPTIONS%

echo.
echo ==========================================
echo Respaldo de %PROJECT_NAME% completado exitosamente.
echo ==========================================
pause