import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped

# Import pure Python tracker from root /src (via editable pip install)
from src.modules.vision.tracker import TargetTracker


class VisionNode(Node):
    def __init__(self):
        super().__init__('uav_opl_vision_node')

        # Instantiate pure Python logic from /src
        self.tracker = TargetTracker()

        # ROS Communication Interface
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.target_pub = self.create_publisher(
            PoseStamped,
            '/uav_opl/target_pose',
            10
        )
        self.get_logger().info("Vision Node initialized successfully.")

    def image_callback(self, msg: Image):
        # Extract image dimensions or raw bytes
        # Pass data directly into pure Python algorithm from /src
        target_coords = self.tracker.process_frame(msg.data, msg.width, msg.height)

        # Publish target coordinates over ROS topic
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = float(target_coords.get('x', 0.0))
        pose_msg.pose.position.y = float(target_coords.get('y', 0.0))
        pose_msg.pose.position.z = float(target_coords.get('z', 0.0))

        self.target_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = VisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()