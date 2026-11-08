import sys
import time
from src.interfaces.MAVLink.master_mavlink import MAVLinkManager
from src.interfaces.Bridge_WIFI.master_wifi import MasterWIFI


def main():
    """Main entry point for the Raspberry Pi onboard system with self-healing nodes."""
    mavlink_node = None
    bridge_wifi = None

    print("[MAIN] Starting self-healing subsystem nodes...")

    # 1. Instantiate and start MAVLink hardware node (Supervised non-blocking reconnect)
    mavlink_node = MAVLinkManager()
    mavlink_node.start(mode="HARDWARE")

    try:
        print("[MAIN] Starting MasterWIFI telecom bridge...")

        # 2. Instantiate MasterWIFI passing live nodes
        bridge_wifi = MasterWIFI(
            handshake_port=5007,
            mavlink_node=mavlink_node
        )

        # 3. Start WiFi telecom orchestrator (Blocks main thread for system logs)
        bridge_wifi.start()

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt (Ctrl+C) detected. Initiating shutdown...")
    except Exception as e:
        print(f"\n[MAIN Error] Unexpected system execution error: {e}")
    finally:
        print("[MAIN] Safely terminating all active onboard subsystem nodes...")

        if bridge_wifi is not None:
            try:
                bridge_wifi.stop()
                print("[MAIN] MasterWIFI telecom bridge stopped cleanly.")
            except Exception as e:
                print(f"[MAIN Warning] Exception during MasterWIFI shutdown: {e}")

        if mavlink_node is not None:
            try:
                mavlink_node.stop()
                print("[MAIN] MAVLink hardware manager node stopped cleanly.")
            except Exception as e:
                print(f"[MAIN Warning] Exception during MAVLink shutdown: {e}")

        print("[MAIN] All onboard nodes terminated safely. System offline.")


if __name__ == "__main__":
    main()