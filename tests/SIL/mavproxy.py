#!/usr/bin/env python3
"""
Pixhawk <-> Raspberry Pi MAVLink link test.

- Waits for the Pixhawk heartbeat
- Sends a companion-computer heartbeat once per second
- Requests ATTITUDE at a fixed rate and prints roll/pitch/yaw + measured rate
- Prints any STATUSTEXT messages (PreArm warnings, etc.)

Usage:
    python mav_test.py                                  # /dev/ttyUSB0 @ 921600
    python mav_test.py --port /dev/ttyUSB0 --baud 115200
    python mav_test.py --hz 50
"""
import argparse
import math
import time

from pymavlink import mavutil


def set_message_interval(m, msg_id, hz):
    interval_us = int(1e6 / hz)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        msg_id, interval_us, 0, 0, 0, 0, 0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--port", default="/dev/ttyUSB0")
    p.add_argument("--baud", type=int, default=921600)
    p.add_argument("--hz", type=float, default=20.0, help="ATTITUDE rate to request")
    args = p.parse_args()

    print(f"Opening {args.port} @ {args.baud}...")
    m = mavutil.mavlink_connection(
        args.port, baud=args.baud,
        source_system=1,
        source_component=mavutil.mavlink.MAV_COMP_ID_ONBOARD_COMPUTER)

    print("Waiting for heartbeat (10 s)...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("No heartbeat. Check TX/RX crossing, GND, SERIAL2_* params and baud rate.")
        return
    print(f"Heartbeat from system {m.target_system}, component {m.target_component}")
    print(f"Mode: {m.flightmode}   Armed: {m.motors_armed()}\n")

    set_message_interval(m, mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, args.hz)

    last_hb_sent = 0.0
    window_start = time.monotonic()
    count = 0
    last = None

    try:
        while True:
            now = time.monotonic()

            # Companion heartbeat, 1 Hz
            if now - last_hb_sent >= 1.0:
                m.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                last_hb_sent = now

            msg = m.recv_match(type=["ATTITUDE", "STATUSTEXT"],
                               blocking=True, timeout=0.5)
            if msg is None:
                continue

            if msg.get_type() == "STATUSTEXT":
                print(f"[FC] {msg.text}")
                continue

            count += 1
            last = msg
            if now - window_start >= 1.0:
                rate = count / (now - window_start)
                print(f"roll {math.degrees(last.roll):6.1f}°  "
                      f"pitch {math.degrees(last.pitch):6.1f}°  "
                      f"yaw {math.degrees(last.yaw):6.1f}°   "
                      f"{rate:5.1f} Hz   mode {m.flightmode}")
                window_start = now
                count = 0
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()