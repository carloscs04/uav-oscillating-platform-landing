#!/bin/bash

PI_USER="${PI_USER:-pi}"        #[cite: 4, 8]
PI_PASS="${PI_PASS:-123}"       #[cite: 4, 8]
PI_IP="${PI_IP:-10.22.246.2}"   #[cite: 4, 8]

echo "=== Connecting to Raspberry Pi via SSH ($PI_USER@$PI_IP) ==="

if command -v sshpass &> /dev/null; then
    sshpass -p "$PI_PASS" ssh -t -o StrictHostKeyChecking=no "$PI_USER@$PI_IP"
else
    ssh -t -o StrictHostKeyChecking=no "$PI_USER@$PI_IP"
fi