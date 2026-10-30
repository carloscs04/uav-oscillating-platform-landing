import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleOdometry, TrajectorySetpoint
from geometry_msgs.msg import PoseStamped

# Import pure Python controllers from root /src
from src.core.drone_state import DroneState
from src.modules.control.pid_controller import PlatformTrackingController


class ControlNode(Node):
    def __init__(self):
        super().__init__('uav_opl_control_node')

        # Core pure Python instances
        self.state = DroneState()
        self.controller = PlatformTrackingController(kp_xy=0.8, kp_z=0.5)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Subscriptions
        self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos
        )
        self.create_subscription(
            PoseStamped,
            '/uav_opl/target_pose',
            self.target_callback,
            10
        )

        # Publisher for PX4 Trajectory Setpoints
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos
        )

        self.target_pose = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.create_timer(0.05, self.control_loop)  # 20Hz Control Loop
        self.get_logger().info("Control Node initialized successfully.")

    def odom_callback(self, msg: VehicleOdometry):
        self.state.timestamp = msg.timestamp / 1e6
        self.state.x = float(msg.position[0])
        self.state.y = float(msg.position[1])
        self.state.z = -float(msg.position[2])  # Convert NED to ENU frame

    def target_callback(self, msg: PoseStamped):
        self.target_pose['x'] = msg.pose.position.x
        self.target_pose['y'] = msg.pose.position.y
        self.target_pose['z'] = msg.pose.position.z

    def control_loop(self):
        # Calculate control actions via pure Python logic in /src
        cmd = self.controller.compute_velocities(
            drone_z=self.state.z,
            target_x=self.target_pose['x'],
            target_y=self.target_pose['y']
        )

        # Package numbers into PX4 message format
        setpoint = TrajectorySetpoint()
        setpoint.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        setpoint.velocity = [cmd['vx'], cmd['vy'], cmd['vz']]

        self.setpoint_pub.publish(setpoint)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()