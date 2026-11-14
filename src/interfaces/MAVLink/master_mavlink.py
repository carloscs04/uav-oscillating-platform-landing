import os
import sys
import time
import threading
import yaml
from typing import Any, Optional
from pymavlink import mavutil


from src.interfaces.MAVLink.state.vehicle_state import VehicleState
from src.interfaces.MAVLink.rx.listener import MAVLinkRXListener
from src.interfaces.MAVLink.rx.telemetry_ui import TelemetryUI
from src.interfaces.MAVLink.tx.system_cmd import SystemCommands
from src.interfaces.MAVLink.tx.flight_cmd import FlightCommands
from src.interfaces.MAVLink.tx.heartbeat import MAVLinkHeartbeatEmitter


class MAVLinkManager:
    """Main Facade API & Lifecycle Manager for MAVLink communications with concise logging."""

    def __init__(self, config_path: Optional[str] = None):
        if config_path is None:
            config_path = os.path.join(os.path.dirname(__file__), "config.yaml")

        self.config_path = config_path
        self.config = self._load_config()

        self.master: Optional[Any] = None
        self.connected: bool = False
        self.running: bool = False
        self._reconnect_thread: Optional[threading.Thread] = None

        self.state = VehicleState()
        self.rx_listener = MAVLinkRXListener(self.state)
        self.telemetry_ui = TelemetryUI(self.state)

        self.system_cmd = SystemCommands()
        self.flight_cmd = FlightCommands()
        self.heartbeat_emitter = MAVLinkHeartbeatEmitter()

    def _load_config(self) -> dict:
        with open(self.config_path, "r") as f:
            return yaml.safe_load(f)["mavlink"]

    def start(self, mode: str = "HARDWARE", timeout: float = 3.0) -> bool:
        """Starts background auto-reconnect supervisor loop."""
        if self.running:
            return self.connected

        self.running = True
        self._reconnect_thread = threading.Thread(
            target=self._connection_supervisor,
            args=(mode.upper(),),
            daemon=True
        )
        self._reconnect_thread.start()

        start_wait = time.time()
        while time.time() - start_wait < timeout:
            if self.connected:
                return True
            time.sleep(0.1)

        return self.connected

    def _connection_supervisor(self, mode: str):
        """Background thread monitoring hardware health with clean single-line logs."""
        target_info = self.config["targets"].get(mode)
        if not target_info:
            print(f"[MAVLink ERR] Mode '{mode}' missing in config.yaml")
            self.running = False
            return

        raw_uri = target_info["uri"]
        baud = target_info.get("baud_rate", 921600)
        hb_timeout = self.config.get("settings", {}).get("heartbeat_timeout", 3.0)

        attempt_logged = False

        while self.running:
            now = time.time()

            # 1. WATCHDOG: Detect link disconnect
            if self.connected:
                attempt_logged = False
                time_since_last = now - self.rx_listener.last_recv_time
                if time_since_last > hb_timeout:
                    print("[MAVLink] Disconnected due to port disconnection. Reconnecting...")
                    self._cleanup_partial_connection()

            # 2. AUTO-RECONNECT: Retry connection silently
            if not self.connected and self.running:
                if not attempt_logged:
                    print("[MAVLink] Connecting...")
                    attempt_logged = True

                try:
                    if "udp:" in raw_uri:
                        self.master = mavutil.mavlink_connection(raw_uri)
                    else:
                        self.master = mavutil.mavlink_connection(raw_uri, baud=baud)

                    self.rx_listener.set_master(self.master)
                    self.system_cmd.set_master(self.master)
                    self.flight_cmd.set_master(self.master)
                    self.heartbeat_emitter.set_master(self.master)

                    hb = self.rx_listener.wait_for_heartbeat(timeout=hb_timeout)
                    if hb:
                        print("[MAVLink] Connected (ONLINE)")
                        self.connected = True
                        attempt_logged = False
                        self.rx_listener.start()
                        self.heartbeat_emitter.start()
                        self.system_cmd.request_telemetry_streams(rate_hz=10)
                    else:
                        self._cleanup_partial_connection()

                except Exception:
                    self._cleanup_partial_connection()

            # Responsive delay loop
            for _ in range(30):
                if not self.running:
                    break
                time.sleep(0.1)

    def _cleanup_partial_connection(self):
        """Resets internal handles on connection drop."""
        self.connected = False
        if hasattr(self, "rx_listener") and self.rx_listener:
            self.rx_listener.stop()
        if hasattr(self, "heartbeat_emitter") and self.heartbeat_emitter:
            self.heartbeat_emitter.stop()

        if self.master:
            try:
                self.master.close()
            except Exception:
                pass
            self.master = None

    def is_healthy(self) -> bool:
        return self.connected

    def stop(self):
        """Gracefully halts supervisor, turns off streams, and terminates connections."""
        self.running = False
        self.connected = False

        if self.master:
            try:
                self.system_cmd.stop_telemetry_streams()
                time.sleep(0.05)
            except Exception:
                pass

            self.rx_listener.stop()
            self.heartbeat_emitter.stop()

            try:
                self.master.close()
            except Exception:
                pass

            self.master = None
            print("[MAVLink] Node stopped cleanly.")

    # --- FACADE WRAPPERS ---

    def arm(self) -> bool:
        self.state.clear_transient_packets()
        return self.system_cmd.arm()

    def disarm(self) -> bool:
        self.state.clear_transient_packets()
        return self.system_cmd.disarm()

    def set_mode(self, mode_name: str) -> bool:
        self.state.clear_transient_packets()
        return self.system_cmd.set_mode(mode_name)

    def takeoff(self, altitude_m: float = 1.5) -> bool:
        self.state.clear_transient_packets()
        return self.flight_cmd.takeoff(altitude_m)

    def land(self) -> bool:
        self.state.clear_transient_packets()
        return self.flight_cmd.land()

    def send_velocity(self, vx: float, vy: float, vz: float):
        self.flight_cmd.send_velocity(vx, vy, vz)

    def debug_force_arm(self) -> bool:
        self.state.clear_transient_packets()
        return self.system_cmd.debug_force_arm()

    def debug_force_disarm(self) -> bool:
        self.state.clear_transient_packets()
        return self.system_cmd.debug_force_disarm()

# Global stream filter to suppress hardcoded pymavlink "Device ... is dead" terminal prints
class SuppressDeadDeviceFilter:
    def __init__(self, original_stream):
        self._original = original_stream

    def write(self, text):
        if "is dead" in text:
            return
        self._original.write(text)

    def flush(self):
        self._original.flush()

    def __getattr__(self, name):
        return getattr(self._original, name)


sys.stdout = SuppressDeadDeviceFilter(sys.stdout)
sys.stderr = SuppressDeadDeviceFilter(sys.stderr)

def _run_interactive_cli(manager: MAVLinkManager):
    """Interactive CLI shell synchronized to confirmed HEARTBEAT state changes."""
    running = True
    display_interval = 1.0
    last_display_time = 0.0

    print("\n=======================================================")
    print("      MAVLink Modular Interactive CLI Inspector        ")
    print("=======================================================")
    print(" Commands: arm, disarm, mode <MODE>, force_arm, force_disarm")
    print("           watch, add <PKT>, remove <PKT>, rate <SEC>, quit")
    print("=======================================================\n")

    while running:
        try:
            now = time.time()
            if now - last_display_time >= display_interval:
                last_display_time = now
                manager.telemetry_ui.print_summary(display_interval)

            cmd_input = input("\nMAV-CLI> ").strip()
            if not cmd_input:
                continue

            parts = cmd_input.split()
            cmd = parts[0].lower()
            target_arm_state: Optional[bool] = None

            if cmd in ["quit", "exit"]:
                running = False
                break
            elif cmd == "arm":
                manager.arm()
                target_arm_state = True
            elif cmd == "disarm":
                manager.disarm()
                target_arm_state = False
            elif cmd == "mode" and len(parts) > 1:
                manager.set_mode(parts[1])
                target_arm_state = None
            elif cmd == "takeoff":
                alt = float(parts[1]) if len(parts) > 1 else 1.5
                manager.takeoff(alt)
            elif cmd == "land":
                manager.land()
            elif cmd in ["force_arm", "farm"]:
                manager.debug_force_arm()
                target_arm_state = True
            elif cmd in ["force_disarm", "fdisarm"]:
                manager.debug_force_disarm()
                target_arm_state = False
            elif cmd == "watch":
                print(f"Active Watched: {manager.state.get_watched_packets()}")
                print(f"Available Unwatched: {manager.state.get_available_unwatched_packets()}")
            elif cmd == "add" and len(parts) > 1:
                manager.state.add_watched_packet(parts[1])
                print(f"[CLI] Added '{parts[1].upper()}' to watch list.")
            elif cmd == "remove" and len(parts) > 1:
                manager.state.remove_watched_packet(parts[1])
                print(f"[CLI] Removed '{parts[1].upper()}' from watch list.")
            elif cmd == "rate" and len(parts) > 1:
                display_interval = max(0.1, float(parts[1]))
                print(f"[CLI] Print interval updated to {display_interval}s.")
            else:
                print(f"[CLI Error] Unknown command or missing parameters: '{cmd_input}'")

            if target_arm_state is not None or cmd == "mode":
                start_wait = time.time()
                while time.time() - start_wait < 1.5:
                    ack = manager.state.get_packet("COMMAND_ACK")
                    st = manager.state.get_packet("STATUSTEXT")

                    if ack and ack.get("result", 0) != 0:
                        break
                    if st:
                        break

                    if target_arm_state is True and manager.state.is_armed():
                        break
                    if target_arm_state is False and not manager.state.is_armed():
                        break

                    time.sleep(0.03)

                manager.telemetry_ui.print_summary(display_interval)
                last_display_time = time.time()

        except (KeyboardInterrupt, EOFError):
            running = False
            break

    manager.stop()


if __name__ == "__main__":
    manager_instance = MAVLinkManager()
    if manager_instance.start(mode="HARDWARE"):
        _run_interactive_cli(manager_instance)