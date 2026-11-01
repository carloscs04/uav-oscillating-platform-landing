import time
import threading
from interfaces.wifi_telemetry.tel_gateway import WifiTelemetryGateway


class MasterNode:
    """
    Central logic controller. 
    Launches and manages telemetry as a service using start/stop/update methods.
    """
    def __init__(self):
        self.is_running = False
        self._logic_thread = None

        # Telemetry service initialized with command callback
        self.telemetry = WifiTelemetryGateway(command_callback=self.handle_command)

    def handle_command(self, command: str):
        """Callback invoked by telemetry when a Wi-Fi command arrives."""
        print(f"[MasterNode] Ground Trigger Received: {command}")
        if command == "START":
            print("[MasterNode] Transitioning to MISSION_ACTIVE state.")
        elif command == "ABORT":
            print("[MasterNode] Emergency ABORT triggered!")

    def start(self):
        """Launches telemetry background loops and starts primary logic engine."""
        print("[MasterNode] Starting subsystems...")
        self.is_running = True

        # 1. Telemetry launches its own continuous internal background threads
        self.telemetry.start()

        # 2. Master Node launches its own control logic loop
        self._logic_thread = threading.Thread(target=self._run_logic_loop, daemon=True)
        self._logic_thread.start()

        print("[MasterNode] All onboard subsystems running.")

    def _run_logic_loop(self):
        """Main state machine loop (50 Hz)."""
        while self.is_running:
            # 1. Gather status snapshot
            snapshot = self._get_telemetry_snapshot()

            # 2. Push snapshot buffer to telemetry (Telemetry streams it on its own loop)
            self.telemetry.update_payload(snapshot)

            time.sleep(0.02)  # 50 Hz loop timing

    def _get_telemetry_snapshot(self) -> dict:
        return {
            "state": "HOVER",
            "altitude": 1.2,
            "system_ok": True
        }

    def stop(self):
        """Stops logic and delegates stop call to telemetry."""
        print("[MasterNode] Initiating shutdown sequence...")
        self.is_running = False
        self.telemetry.stop()
        print("[MasterNode] Shutdown complete.")