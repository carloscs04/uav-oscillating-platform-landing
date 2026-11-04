#!/bin/bash

SUBNET="${1:-10.22.246.0/24}"

echo "=== Scanning Subnet: $SUBNET for active Pi hosts ==="

if command -v nmap &> /dev/null; then
    nmap -sn "$SUBNET" | grep -B 2 "Raspberry Pi"
elif command -v arp-scan &> /dev/null; then
    sudo arp-scan --localnet | grep -iE "b8:27:eb|dc:a6:32|e4:5f:01"
else
    PREFIX=$(echo "$SUBNET" | cut -d'.' -f1-3)
    for i in {1..254}; do
        ping -c 1 -W 1 "$PREFIX.$i" &> /dev/null && echo "[+] Host active: $PREFIX.$i" &
    done
    wait
fi