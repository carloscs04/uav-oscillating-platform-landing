import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge

# Import pure Python tracker algorithm from root src/
# from src.perception.aruco_tracker import ArucoTracker


class ArucoNode(Node):
    def __init__(self):
        super().__init__('aruco_node')
        
        self.bridge = CvBridge()
        # self.tracker = ArucoTracker()
        
        self.pose_pub = self.create_publisher(PoseStamped, '/target/pose', 10)
        self.image_sub = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        
        self.get_logger().info("ArUco Perception Node started listening on /camera/image_raw")

    def image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        target_pose = self.tracker.process_frame(frame)
        
        if target_pose is not None:
            pose_msg = PoseStamped()
            pose_msg.header = msg.header
            pose_msg.pose.position.x = float(target_pose['x'])
            pose_msg.pose.position.y = float(target_pose['y'])
            pose_msg.pose.position.z = float(target_pose['z'])
            self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()