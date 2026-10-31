import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range

# Import pure Python driver from root src/
#from src.sensors.rangefinder_driver import RangefinderDriver


class RangefinderNode(Node):
    def __init__(self):
        super().__init__('rangefinder_node')
        
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value
        
        #self.driver = RangefinderDriver(port=port)
        self.range_pub = self.create_publisher(Range, '/sensor/distance', 10)
        self.timer = self.create_timer(0.05, self.read_sensor)  # 20 Hz
        
        self.get_logger().info(f"Distance Sensor Node initialized on port {port}")

    def read_sensor(self):
        distance = self.driver.get_distance()
        if distance is not None:
            msg = Range()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'rangefinder_link'
            msg.radiation_type = Range.INFRARED
            msg.field_of_view = 0.05
            msg.min_range = 0.1
            msg.max_range = 12.0
            msg.range = float(distance)
            self.range_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RangefinderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()