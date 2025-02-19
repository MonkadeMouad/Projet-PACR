import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import time

class LinePublisher(Node):
    def __init__(self):
        super().__init__('line_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(1.0, self.publish_line)  # Publish every second

    def publish_line(self):
        marker = Marker()
        marker.header.frame_id = "map"  # Change to "base_link" or another frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "line"
        marker.id = 0
        marker.type = Marker.LINE_STRIP  # Line type
        marker.action = Marker.ADD

        # Line color (RGBA)
        marker.color.r = 1.0  # Red
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully opaque

        # Line width
        marker.scale.x = 0.1  # Thickness of the line

        # Define points for the line
        point1 = Point()
        point1.x = 0.0
        point1.y = 0.0
        point1.z = 0.0

        point2 = Point()
        point2.x = 2.0
        point2.y = 2.0
        point2.z = 0.0

        point3 = Point()
        point3.x = 4.0
        point3.y = 0.0
        point3.z = 0.0

        marker.points.append(point1)
        marker.points.append(point2)
        marker.points.append(point3)

        # Publish marker
        self.publisher.publish(marker)
        self.get_logger().info("Published Line Marker")

def main(args=None):
    rclpy.init(args=args)
    node = LinePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
