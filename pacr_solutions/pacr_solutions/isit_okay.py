#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PointStamped

# Adjust these imports to your actual package structure
from pacr_simulation.map_utils import GridMap, Point

class ClickedPointChecker(Node):
    def __init__(self):
        super().__init__('clicked_point_checker')

        # We'll create a QoSProfile for the map subscription so we can get latched (transient_local) data
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # 1) Subscribe to the OccupancyGrid
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile
        )

        # 2) Subscribe to /clicked_point (published by RViz when you click '2D Pose Estimate' or 'Point' tool)
        self.point_sub = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10
        )

        self.grid_map = None
        self.get_logger().info("ClickedPointChecker node up. Waiting for /map and /clicked_point...")

    def map_callback(self, msg: OccupancyGrid):
        """
        Store the map as a GridMap using your map_utils.
        """
        self.grid_map = GridMap.from_msg(msg)
        self.get_logger().info("Received and stored map.")

    def clicked_point_callback(self, msg: PointStamped):
        """
        When a clicked point is received, check if that location is free or occupied.
        """
        if self.grid_map is None:
            self.get_logger().warn("No map available yet.")
            return

        # Extract x,y from the PointStamped
        x = msg.point.x
        y = msg.point.y

        # Use the GridMap.get(...) method.
        # By default, the occupancy threshold for "occupied" is often around 50-100 in ROS maps.
        # We'll say <50 is free, >=50 is occupied.
        cell_value = self.grid_map.get(Point(x, y))

        # Decide a threshold
        # If cell_value >= 50, it's presumably an obstacle or unknown
        # If cell_value <  50, it's presumably free
        if cell_value < 50:
            self.get_logger().info(
                f"Clicked point ({x:.2f}, {y:.2f}) is free (cell value={cell_value})."
            )
        else:
            self.get_logger().info(
                f"Clicked point ({x:.2f}, {y:.2f}) is occupied or unknown (cell value={cell_value})."
            )

def main(args=None):
    rclpy.init(args=args)
    node = ClickedPointChecker()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
