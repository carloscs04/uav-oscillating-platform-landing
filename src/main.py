import time
import signal
import sys
from logic.master.master_node import MasterNode


def main():
    print("=== [LAUNCHER] Booting Onboard Raspberry Pi Stack ===")

    # Instantiate and start Master Node supervisor
    master = MasterNode()
    master.start()

    # Graceful shutdown handler
    def handle_shutdown(sig, frame):
        print("\n=== [LAUNCHER] Shutdown signal caught. Stopping Master Node... ===")
        master.stop()
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_shutdown)
    signal.signal(signal.SIGTERM, handle_shutdown)

    # Launcher thread stays alive while master node is active
    while master.is_running:
        time.sleep(1.0)


if __name__ == "__main__":
    main()