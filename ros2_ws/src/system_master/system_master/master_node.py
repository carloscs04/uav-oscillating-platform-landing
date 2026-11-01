#!/usr/bin/env python3
import os
import sys
import threading
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool


class DroneSupervisor(Node):
    """
    Master Supervisor Node:
    - Acts as central mission manager & state machine.
    - Monitors critical node heartbeats (PX4, Camera, Perception).
    - Provides interactive terminal CLI for operator commands.
    - Triggers mission sequences (Hover -> Align -> Land).
    """

    def __init__(self):
        super().__init__('uav_supervisor')

        # Mode Resolution
        self.mode = os.environ.get('UAV_MODE', 'HARDWARE')
        self.get_logger().info(f"==========================================")
        self.get_logger().info(f"   UAV SUPERVISOR INITIALIZED [{self.mode}]   ")
        self.get_logger().info(f"==========================================")

        # Node Health Status Tracking
        self.node_status = {
            'px4_mavlink': False,
            'camera_driver': False,
            'aruco_perception': False,
            'distance_sensor': False,
            'control_node': False
        }

        # Critical nodes required for mission
        self.critical_nodes = ['px4_mavlink', 'camera_driver', 'aruco_perception']

        # Mission State
        self.mission_state = "IDLE"  # IDLE, READY, HOVER, ALIGN, LANDING, FINISHED, EMERGENCY

        # Subscriptions to monitor sub-node health / telemetry
        self.telemetry_sub = self.create_subscription(
            String, '/uav/state', self._telemetry_callback, 10
        )

        # Service to notify Bringup if Emergency Stop or Shutdown is needed
        self.shutdown_publisher = self.create_publisher(String, '/uav/supervisor/system_command', 10)

        # Start Interactive CLI Thread
        self.cli_thread = threading.Thread(target=self._run_cli_loop, daemon=True)
        self.cli_thread.start()

    def _telemetry_callback(self, msg: String):
        # MAVLink telemetry is active
        self.node_status['px4_mavlink'] = True

    def is_critical_data_ready(self) -> bool:
        """Verifies if all critical modules are active before enabling mission."""
        for node in self.critical_nodes:
            if not self.node_status.get(node, False):
                return False
        return True

    def _print_status(self):
        print("\n" + "=" * 50)
        print(f"       UAV SYSTEM STATUS [{self.mode}]       ")
        print("=" * 50)
        print(f" Mission State: {self.mission_state}")
        print(" Module Health Status:")
        for node, active in self.node_status.items():
            critical_flag = "[CRITICAL]" if node in self.critical_nodes else "          "
            status_str = "READY" if active else "OFFLINE"
            print(f"   - {node:<20} {critical_flag} : {status_str}")
        print("=" * 50 + "\n")

    def _run_cli_loop(self):
        """Interactive Terminal Window Operator Interface."""
        time.sleep(1)
        print("\nDrone Master Supervisor Console Active.")
        print("Type 'help' for available commands.\n")

        while rclpy.ok():
            try:
                cmd = input("[Master-Supervisor] > ").strip().lower()

                if cmd == "help":
                    print("\nAvailable Commands:")
                    print("  status    - Display health status of all modules")
                    print("  start     - Check critical data and initiate landing mission")
                    print("  emergency - Trigger immediate emergency landing")
                    print("  stop      - Shutdown all nodes and terminate system")
                    print("  help      - Show this menu\n")

                elif cmd == "status":
                    self._print_status()

                elif cmd == "start":
                    if not self.is_critical_data_ready():
                        print("[ERROR] Cannot start mission: Critical nodes are OFFLINE!")
                        self._print_status()
                    else:
                        print("[ACTION] Critical data verified! Activating Control Node...")
                        self.mission_state = "HOVER"
                        # Signal Control Node to begin sequence

                elif cmd == "emergency":
                    print("[ALERT] EMERGENCY LANDING TRIGGERED!")
                    self.mission_state = "EMERGENCY"
                    msg = String()
                    msg.data = "EMERGENCY_LAND"
                    self.shutdown_publisher.publish(msg)

                elif cmd == "stop":
                    print("[ACTION] Requesting Bring-up to stop all nodes...")
                    msg = String()
                    msg.data = "SHUTDOWN_ALL"
                    self.shutdown_publisher.publish(msg)
                    sys.exit(0)

                else:
                    if cmd:
                        print(f"Unknown command: '{cmd}'. Type 'help' for options.")

            except (KeyboardInterrupt, EOFError):
                break


def main(args=None):
    rclpy.init(args=args)
    node = DroneSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()