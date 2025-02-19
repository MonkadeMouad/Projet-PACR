import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import os

class MapSubscriber(Node):
    def __init__(self):
        super().__init__('map_listener')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.listener_callback,
            10)
        self.subscription  # Prevent unused variable warning
        self.map_saved = False  # Ensure we save only once (optional)

    def listener_callback(self, msg):
        if not self.map_saved:
            self.get_logger().info(f'Received map with size: {len(msg.data)}')

            # Convert map data to numpy array
            width = msg.info.width
            height = msg.info.height
            resolution = msg.info.resolution
            origin = msg.info.origin.position

            # Convert OccupancyGrid data to numpy array
            grid = np.array(msg.data, dtype=np.int8).reshape((height, width))

            # Convert -1 (unknown) to 127 (gray), 100 (occupied) to 0 (black), and 0 (free) to 255 (white)
            grid = np.where(grid == -1, 127, grid)  # Unknown -> Gray
            grid = np.where(grid == 100, 0, grid)   # Occupied -> Black
            grid = np.where(grid == 0, 255, grid)   # Free -> White

            # Save as PGM image
            map_filename = "saved_map.pgm"
            cv2.imwrite(map_filename, grid)

            # Save YAML metadata
            yaml_data = {
                "image": map_filename,
                "resolution": resolution,
                "origin": [origin.x, origin.y, origin.z],
                "negate": 0,
                "occupied_thresh": 0.65,
                "free_thresh": 0.196
            }

            yaml_filename = "saved_map.yaml"
            with open(yaml_filename, 'w') as yaml_file:
                yaml.dump(yaml_data, yaml_file, default_flow_style=False)

            self.get_logger().info(f'Map saved as {map_filename} and {yaml_filename}')
            self.map_saved = True  # Prevent multiple saves (optional)

def main(args=None):
    rclpy.init(args=args)
    node = MapSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
