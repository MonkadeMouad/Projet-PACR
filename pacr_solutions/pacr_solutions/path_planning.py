import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import OccupancyGrid, Path
from pacr_interfaces.srv import GetPath
from pacr_simulation.map_utils import GridMap
from .planner import PathPlanner
import numpy as np

# Utilitaire pour convertir OccupancyGrid en GridMap
class GridMapUtils:
    """Classe utilitaire pour convertir un OccupancyGrid en GridMap."""
    @staticmethod
    def from_msg(msg: OccupancyGrid) -> GridMap:
        resolution = msg.info.resolution
        width, height = msg.info.width, msg.info.height
        origin = (msg.info.origin.position.x, msg.info.origin.position.y)

        # Transformation des données en numpy array
        data = np.array(msg.data, dtype=np.int8).reshape((height, width), order="F")

        return GridMap(origin, resolution, (width, height), data)

class PathPlanningNode(Node):
    def __init__(self):
        super().__init__('path_planning')
        self.get_logger().info("Path Planning Node Started")

        # Planificateur
        self.planner = PathPlanner(method="A*")

        # Souscription à la carte (costmap)
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, 'costmap', self.costmap_cb, qos_profile
        )

        # Publication du chemin planifié
        self.path_pub = self.create_publisher(Path, 'planned_path', 10)

        # Service de planification
        self.path_service = self.create_service(GetPath, 'get_path', self.handle_get_path)

    def costmap_cb(self, msg):
        """Callback de réception de la carte des obstacles (costmap)."""
        self.get_logger().info("Costmap received")
        grid_map = GridMapUtils.from_msg(msg)
        self.planner.set_map(grid_map)

    def handle_get_path(self, request, response):
        start = (request.start.x, request.start.y)
        goal = (request.goal.x, request.goal.y)

        self.get_logger().info(f"Planning path from {start} to {goal}")

        path = self.planner.plan_path(start, goal)

        if path:
            response.path = self.planner.convert_to_ros_path(path)
            self.path_pub.publish(response.path)
            return response  # ✅ Valid response
        else:
            self.get_logger().error("No path found")
            return None  # ❌ Invalid! Causes TypeError


def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
