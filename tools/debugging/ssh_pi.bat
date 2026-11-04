@echo off
set "PI_USER=pi"
set "PI_IP=10.22.246.2"

echo === Connecting to Raspberry Pi via SSH (%PI_USER%@%PI_IP%) ===
ssh -t -o StrictHostKeyChecking=no %PI_USER%@%PI_IP%