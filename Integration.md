# **Physical Integration Notes**

Author: Paty González


## Raspberry Pi 5

**Specs:** Raspberry Pi 5 Model B, 4 GB RAM, active cooler fitted.

**OS:** Debian 13 (trixie), 64-bit. The system Python is 3.13; the project `.venv` uses Python 3.12 (created with `uv venv --python 3.12 --seed .venv` before running `tools/setup/linux/setup_python.sh`).

**ROS 2:** not available on this OS (ROS 2 Jazzy only ships packages for Ubuntu 24.04). Until the Pi runs Ubuntu 24.04 or a ROS Docker container, the onboard code runs as plain Python from `src/`.

**Power:**
- Needs 5 V / 5 A (27 W). With a weaker supply, the Pi limits USB current and may throttle.
- On the drone, power it from a dedicated 5 V BEC rated ≥ 5 A, not from the Pixhawk.
- Check for undervoltage with `vcgencmd get_throttled` (`0x0` = OK).


## Camera

**Hardware:**


**Status (2026-10-31):** not detected (`rpicam-hello --list-cameras` → "No cameras available").


**Arrival checklist:**
1. `rpicam-hello --list-cameras` shows the sensor.
2. `rpicam-still -o test.jpg --nopreview`: check focus, exposure and orientation.
3. Measure frame rate at 640×480.
4. Calibrate (chessboard) and save the intrinsics.
5. Detect a printed ArUco marker at a known distance and compare the measured distance.


## Pixhawk

**Hardware:** Pixhawk 1 (2.4.8-style board: TELEM 1, TELEM 2, SERIAL 4/5, GPS, I2C, CAN, SPI ports), running PX4.

**Companion link (Pixhawk TELEM 2 → CP2102 USB-serial adapter → Pi USB port), 921600 baud:**

| TELEM 2 pin | Signal | Connect to (CP2102) |
|---|---|---|
| 1 | 5 V | **Do not connect** |
| 2 | TX (out) | RXD |
| 3 | RX (in) | TXD |
| 4 / 5 | CTS / RTS | Not used |
| 6 | GND | GND |

- TX and RX cross over. Leave the CP2102's 5V and 3V3 pins unconnected; the Pixhawk has its own power.
- The adapter appears on the Pi as `/dev/ttyUSB*`. Use its stable path instead, since the number can change between boots and the rangefinder is also a USB serial device:
  ```bash
  ls -l /dev/serial/by-id/     # e.g. usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0
  ```
- Serial port permission: `sudo usermod -aG dialout $USER`, then log out and back in.
- Quick link test: `mavproxy.py --master=/dev/serial/by-id/<cp2102> --baudrate 921600` (or `tests/SIL/PX4-mavlink_test.py` pointed at the port) should show a heartbeat.
- If messages drop at 921600 (long or loose jumper wires), lower `SER_TEL2_BAUD` and `config.yaml` to 115200 or 57600.

**PX4 parameters (set in QGroundControl):**

| Parameter | Value |
|---|---|
| `MAV_1_CONFIG` | TELEM 2 |
| `MAV_1_MODE` | Onboard |
| `SER_TEL2_BAUD` | 921600 |

Reboot the Pixhawk after changing them.

**Project config:** `src/interfaces/hardware/mavlink/config.yaml` → `HARDWARE` currently says `/dev/ttyAMA0` (the Pi's GPIO UART). Change it to the CP2102's `/dev/serial/by-id/...` path.

**Notes:**
- Before the first flight test: verify the link with propellers removed (heartbeat received, telemetry streaming, arm/disarm from the Pi), and keep a pilot on the RC transmitter able to take manual control.
