# File: src/main.py

import time
from src.interfaces.Bridge_WIFI.master_wifi import MasterWIFI
from src.logic.master.master_node import MasterNode


def main():
    master_node = None
    bridge_wifi = None

    print("[MAIN] Booting UAV Master Supervisor...")

    # 1. Instantiate and start MasterNode (starts all subsystem nodes)
    master_node = MasterNode(mavlink_mode="HARDWARE", enable_local_stdin=True)
    master_node.start(auto_start_all=True)

    try:
        # 2. Start MasterWIFI bridge linked to MasterNode's subsystem instances
        bridge_wifi = MasterWIFI(
            handshake_port=5007,
            mavlink_node=master_node.mavlink_node,
            camera_node=master_node.camera_state,
            vision_node=master_node.vision_state,
            distance_node=master_node.distance_state,
            control_node=master_node.control_state,
            master_node=master_node
        )
        bridge_wifi.start()

    except KeyboardInterrupt:
        print("\n[MAIN] KeyboardInterrupt detected. Shutting down...")
    finally:
        # Stop MasterNode FIRST so shutdown logs transmit over UDP before Wi-Fi closes
        if master_node is not None:
            master_node.stop()
            time.sleep(0.25)
        if bridge_wifi is not None:
            bridge_wifi.stop()
        print("[MAIN] System offline.")


if __name__ == "__main__":
    main()