# src/interfaces/MAVLink/manager.py

import os
import sys
import time
import threading
import yaml
from typing import Any, Optional
from pymavlink import mavutil

from src.interfaces.MAVLink.rx_cmd.driver_rx import MAVLinkRXDriver
from src.interfaces.MAVLink.tx_cmd.driver_tx import MAVLinkTXDriver


class MAVLinkManager:
    """Lifecycle controller: Provisions serial link and coordinates RX & TX sub-drivers."""

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        self.config_path = config_path
        self.config = self._load_config()

        self.master: Optional[Any] = None
        self.rx_driver = MAVLinkRXDriver()
        self.tx_driver = MAVLinkTXDriver()

    def _load_config(self) -> dict:
        with open(self.config_path, "r") as f:
            return yaml.safe_load(f)["mavlink"]

    def start(self, mode: str = "HARDWARE") -> bool:
        """Opens serial/UDP channels and binds them to RX and TX sub-drivers."""
        mode = mode.upper()
        target_info = self.config["targets"].get(mode)

        if not target_info:
            print(f"[Manager Error] Mode '{mode}' not found in config.yaml")
            return False

        uri = target_info["uri"]
        baud = target_info.get("baud_rate", 921600)
        timeout = self.config.get("settings", {}).get("heartbeat_timeout", 5.0)

        try:
            if "udp:" in uri:
                self.master = mavutil.mavlink_connection(uri)
            else:
                self.master = mavutil.mavlink_connection(uri, baud=baud)
        except Exception as e:
            print(f"[Manager Error] Connection failed to {uri}: {e}")
            return False

        # Pass shared MAVLink connection instance to both sub-drivers
        self.rx_driver.set_master(self.master)
        self.tx_driver.set_master(self.master)

        # Validate link via RX driver heartbeat wait
        hb = self.rx_driver.wait_for_heartbeat(timeout=timeout)
        if hb:
            print(f"[Manager] MAVLink driver started successfully on {uri} ({mode} mode)")
            self.tx_driver.request_telemetry_streams(rate_hz=10)
            return True

        self.stop()
        return False

    def is_healthy(self) -> bool:
        return self.master is not None

    def stop(self):
        """Gracefully stops telemetry streams (turns off hardware LED) and closes serial port."""
        if self.master:
            print("[Manager] Shutting down MAVLink streams...")
            self.tx_driver.stop_telemetry_streams()
            time.sleep(0.1)  # Buffer flush delay

            print("[Manager] Closing serial channel...")
            self.master.close()
            self.master = None
            self.rx_driver.set_master(None)
            self.tx_driver.set_master(None)
            print("[Manager] MAVLink Manager stopped cleanly.")


def _run_interactive_cli(manager: MAVLinkManager):
    """Runs high-frequency RX loop with throttled console logging and an interactive CLI prompt."""
    running = True
    display_interval = 1.0  # Console refresh rate in seconds (Throttled visualization)
    last_display_time = 0.0

    # High-frequency background reader thread & RC pulse emitter
    def _rx_worker():
        nonlocal running
        last_rc_time = 0.0
        while running:
            # 1. High speed telemetry parsing
            manager.rx_driver.read_telemetry()

            # 2. Continuous 2Hz RC Override pulse to prevent Radio Failsafe
            now = time.time()
            if now - last_rc_time >= 0.5:
                manager.tx_driver.send_rc_override(pitch=1500, roll=1500, throttle=1500, yaw=1500)
                last_rc_time = now

            time.sleep(0.005)  # Fast 200Hz loop

    rx_thread = threading.Thread(target=_rx_worker, daemon=True)
    rx_thread.start()

    print("\n=======================================================")
    print("      MAVLink Interactive CLI & Dynamic Inspector      ")
    print("=======================================================")
    print(" Available Commands:")
    print("   arm              - Arm drone motors")
    print("   disarm           - Disarm drone motors")
    print("   mode <MODE>      - Change flight mode (GUIDED, LAND, LOITER, etc.)")
    print("   takeoff <ALT>    - Initiate automatic takeoff (e.g., takeoff 1.5)")
    print("   watch            - Display active vs available unwatched packets")
    print("   add <PACKET>     - Add packet to active visualization (e.g., add HIGHRES_IMU)")
    print("   remove <PACKET>  - Remove packet from visualization (e.g., remove VFR_HUD)")
    print("   rate <SEC>       - Set visualization print interval in seconds (default 1.0)")
    print("   force_arm        - Force arm motors bypassing pre-arm checks")
    print("   quit / exit      - Stop manager and exit")
    print("=======================================================\n")

    # Command input handler loop in main thread
    while running:
        try:
            now = time.time()
            # Throttled formatted display printing
            if now - last_display_time >= display_interval:
                last_display_time = now
                watched = manager.rx_driver.get_watched_packets()
                t_stamp = time.strftime("%H:%M:%S")
                print(f"\n--- [TELEMETRY SUMMARY @ {t_stamp}] (Refresh Rate: {display_interval}s) ---")
                
                for pkt_type in watched:
                    raw_data = manager.rx_driver.latest_telemetry_cache.get(pkt_type)
                    if raw_data:
                        formatted_str = manager.rx_driver.format_packet_summary(pkt_type, raw_data)
                        print(f" > [{pkt_type}]: {formatted_str}")
                    else:
                        print(f" > [{pkt_type}]: Awaiting packet stream...")

            # Non-blocking interactive command prompt
            cmd_input = input("\nMAV-CLI> ").strip()
            if not cmd_input:
                continue

            parts = cmd_input.split()
            cmd = parts[0].lower()

            if cmd in ["quit", "exit"]:
                running = False
                break
            elif cmd == "arm":
                manager.tx_driver.arm()
            elif cmd == "disarm":
                manager.tx_driver.disarm()
            elif cmd == "mode" and len(parts) > 1:
                manager.tx_driver.set_mode(parts[1])
            elif cmd in ["disable_checks", "nochecks"]:
                manager.tx_driver.disable_arming_checks()
            elif cmd == "takeoff":
                alt = float(parts[1]) if len(parts) > 1 else 1.5
                manager.tx_driver.takeoff(alt)
            elif cmd in ["force_arm", "farm"]:
                manager.tx_driver.force_arm()
            elif cmd in ["disarm", "fdisarm", "force_disarm"]:
                manager.tx_driver.force_disarm()
            elif cmd == "land":
                manager.tx_driver.set_mode("LAND")
            elif cmd == "watch":
                print(f"Active Watched: {manager.rx_driver.get_watched_packets()}")
                print(f"Available Unwatched: {manager.rx_driver.get_available_unwatched_packets()}")
            elif cmd == "add" and len(parts) > 1:
                manager.rx_driver.add_watched_packet(parts[1])
                print(f"[CLI] Added '{parts[1].upper()}' to watch list.")
            elif cmd == "remove" and len(parts) > 1:
                manager.rx_driver.remove_watched_packet(parts[1])
                print(f"[CLI] Removed '{parts[1].upper()}' from watch list.")
            elif cmd == "rate" and len(parts) > 1:
                display_interval = max(0.1, float(parts[1]))
                print(f"[CLI] Visualization print interval updated to {display_interval}s.")
            else:
                print(f"[CLI Error] Unknown command or missing parameters: '{cmd_input}'")

        except (KeyboardInterrupt, EOFError):
            running = False
            break

    manager.stop()


if __name__ == "__main__":
    # When executed directly, launch Manager and run Interactive CLI Prompt Shell
    manager_instance = MAVLinkManager()
    if manager_instance.start(mode="HARDWARE"):
        _run_interactive_cli(manager_instance)