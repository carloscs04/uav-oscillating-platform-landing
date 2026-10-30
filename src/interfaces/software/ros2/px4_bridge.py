import sys
import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry

# Path routing to root /src
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '../../../../')))
from src.core.drone_state import DroneState
from src.modules.precision_landing.controller import LandingController

class PX4BridgeNode(Node):
    def __init__(self):
        super().__init__('px4_bridge_node')
        self.state = DroneState()
        self.controller = LandingController(target_altitude=1.0)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        self.create_subscription(
            VehicleOdometry, 
            '/fmu/out/vehicle_odometry', 
            self._odom_cb, 
            qos
        )
        self.create_timer(0.1, self._loop)

    def _odom_cb(self, msg: VehicleOdometry):
        self.state.timestamp = msg.timestamp / 1e6
        self.state.x, self.state.y, self.state.z = msg.position[0], msg.position[1], -msg.position[2]

    def _loop(self):
        cmd = self.controller.compute_command(self.state)
        self.get_logger().info(f"Alt: {self.state.z:.2f}m | Cmd Vz: {cmd['cmd_vz']:.2f}")

def main():
    rclpy.init()
    node = PX4BridgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()