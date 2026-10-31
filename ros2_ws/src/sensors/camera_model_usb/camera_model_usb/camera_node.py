import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import pure Python camera reader from root src/
#from src.sensors.camera_stream import UniversalCamera


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        
        self.declare_parameter('device_id', 0)
        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        
        # self.camera = UniversalCamera(source=device_id)
        self.bridge = CvBridge()
        
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.timer = self.create_timer(1.0 / 30.0, self.publish_frame)  # 30 FPS
        
        self.get_logger().info(f"Camera Node started capturing on device index {device_id}")

    def publish_frame(self):
        frame = self.camera.get_frame()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera_optical_frame'
            self.image_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.camera.release()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()