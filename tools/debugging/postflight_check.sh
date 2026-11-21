#!/bin/bash
# Post-flight power / USB / link check for the Raspberry Pi.
#
# Run ON THE PI, right after landing, BEFORE unplugging the flight battery
# (a reboot or power-cycle wipes the kernel log and resets the undervoltage flags).
#
#   preflight :  bash tools/debugging/postflight_check.sh pre
#   postflight:  bash tools/debugging/postflight_check.sh
#
# Output is printed and saved to ~/flight_checks/check_<date>.txt

MODE="${1:-post}"
OUT_DIR="$HOME/flight_checks"
mkdir -p "$OUT_DIR"
STAMP="$(date +%Y-%m-%d_%H-%M-%S)"
OUT="$OUT_DIR/check_${STAMP}_${MODE}.txt"
MARK_FILE="$OUT_DIR/.last_preflight_boot"

{
echo "=== ${MODE^^}FLIGHT CHECK  $(date)  (laptop time = Pi time - your timezone offset) ==="
BOOT="$(uptime -s)"
echo "Boot time      : $BOOT"
echo "Throttled      : $(vcgencmd get_throttled 2>/dev/null)"
echo "5V input       : $(vcgencmd pmic_read_adc EXT5V_V 2>/dev/null | awk '{print $2}')"
echo "USB max current: $(vcgencmd get_config usb_max_current_enable 2>/dev/null)"
echo "Temperature    : $(vcgencmd measure_temp 2>/dev/null)"
echo
echo "How to read 'throttled': 0x0 = OK | 0x50000 = undervoltage happened earlier"
echo "                         0x50005 = undervoltage NOW | 0x80000/0x80008 = overheating"

if [ "$MODE" = "pre" ]; then
    echo "$BOOT" > "$MARK_FILE"
    echo "=== PREFLIGHT $(date) ===" | sudo tee /dev/kmsg > /dev/null
    echo
    echo "Preflight marker written to the kernel log."
else
    echo
    if [ -f "$MARK_FILE" ]; then
        if [ "$(cat "$MARK_FILE")" != "$BOOT" ]; then
            echo "!!! Boot time changed since preflight ($(cat "$MARK_FILE") -> $BOOT):"
            echo "!!! the Pi REBOOTED during the session (brownout?). Previous-boot kernel log:"
            journalctl -k -b -1 --no-pager 2>/dev/null | grep -iE "usb|cp210|ttyUSB|voltage|over-current|disconnect" | tail -40 \
                || echo "    (no persistent journal: create /var/log/journal to keep logs across reboots)"
        else
            echo "Pi stayed on since preflight (no reboot)."
        fi
    else
        echo "(no preflight marker found: run with 'pre' before the next flight)"
    fi

    echo
    echo "--- Kernel: USB / serial / power events since the preflight marker ---"
    sudo dmesg -T | sed -n '/=== PREFLIGHT/,$p' | grep -iE "PREFLIGHT|usb|cp210|ttyUSB|voltage|over-current|disconnect" | tail -60
    echo
    echo "--- Onboard software: MAVLink link events (last 30 min) ---"
    journalctl -u drone --since "30 min ago" --no-pager 2>/dev/null \
        | grep -iE "Disconnected|Connecting|Connected \(ONLINE\)|SensorFeed|ERR|WARN" | tail -40 \
        || echo "(drone.service not used: check the terminal where main.py runs)"
    echo
    echo "What it means:"
    echo "  'cp210x ... disconnected' / 'USB disconnect'  -> Pi lost the USB-serial adapter to the Pixhawk"
    echo "  'Undervoltage detected'                         -> 5V supply sagging (BEC / wiring)"
    echo "  'over-current'                                  -> USB port current limit hit"
    echo "  only '[MAVLink] Disconnected' in software log   -> USB stayed, MAVLink data stopped (Pixhawk side)"
fi
} 2>&1 | tee "$OUT"

echo
echo "Saved to: $OUT   (copy it into experiments/<date>/ next to the flight CSV)"
