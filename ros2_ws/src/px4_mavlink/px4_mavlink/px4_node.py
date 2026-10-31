import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

# Import Manager from pure Python core
from src.interfaces.hardware.mavlink.manager import MAVLinkManager


class PX4Node(Node):
    def __init__(self):
        super().__init__('px4_node')

        # 1. Declare ROS 2 parameter with default "SIMULATION"
        self.declare_parameter('mode', 'SIMULATION')
        self.mode = self.get_parameter('mode').get_parameter_value().string_value.upper()

        self.get_logger().info(f"Starting PX4 Node in [{self.mode}] mode...")

        # 2. Manager handles starting driver for the given mode
        self.manager = MAVLinkManager()
        if not self.manager.start(mode=self.mode):
            self.get_logger().error(f"Failed to start MAVLink Manager in {self.mode} mode.")
            raise RuntimeError("MAVLink connection failed")

        self.driver = self.manager.driver

        # 3. Publishers
        self.state_pub = self.create_publisher(String, '/uav/state', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/uav/pose', 10)

        # 4. Telemetry Loop
        self.timer = self.create_timer(0.02, self.telemetry_loop)

    def telemetry_loop(self):
        if not self.manager.is_healthy():
            return

        data = self.driver.read_telemetry()
        if data:
            msg = String()
            msg.data = str(data)
            self.state_pub.publish(msg)

    def destroy_node(self):
        self.manager.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PX4Node()
        rclpy.spin(node)
    except Exception as e:
        print(f"PX4 Node stopped: {e}")
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()