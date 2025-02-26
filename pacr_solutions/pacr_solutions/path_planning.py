#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import Pose, PoseStamped

# Import the custom service
from pacr_interfaces.msg import Pose2D, Point2D
from pacr_interfaces.srv import GetPath


# Your map utility classes
from pacr_simulation.map_utils import GridMap

# Your RRT* implementation
from pacr_solutions.rrt_star import RRTStar

class PathPlanningService(Node):
    def __init__(self):
        super().__init__('path_planning_service')

        # QoS so we can correctly latch onto the map
        qos_profile = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # Subscriber to the map
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.map_callback,
            qos_profile
        )

        # Internal storage of the grid map
        self.grid_map = None

        # Create the service server
        self.srv = self.create_service(
            GetPath,              # Service type
            'get_path',           # Service name
            self.get_path_callback
        )

        self.get_logger().info("PathPlanningService node is up. Waiting for map and service calls...")

    def map_callback(self, msg: OccupancyGrid):
        # Convert OccupancyGrid to GridMap
        self.grid_map = GridMap.from_msg(msg)
        self.get_logger().info("Map received and processed.")

    def get_path_callback(self, request, response):
        """
        Service callback for 'get_path'.
        The request contains:
            request.start (Pose2D),
            request.goal (Pose2D).
        The response must be a list of Point2D[] named 'path'.
        """

        # 1) Check if we have a valid map yet
        if self.grid_map is None:
            self.get_logger().error("No map available yet. Unable to plan.")
            response.path = []
            return response

        # 2) Extract start/goal from request
        start_x = request.start.x
        start_y = request.start.y
        start_theta = request.start.theta  # unused, but available

        goal_x = request.goal.x
        goal_y = request.goal.y
        goal_theta = request.goal.theta    # unused, but available

        self.get_logger().info(
            f"Received request to plan path from ({start_x}, {start_y}) to ({goal_x}, {goal_y})."
        )

        # 3) Initialize and run the RRT* planner
        planner = RRTStar(
            start=(start_x, start_y),
            goal=(goal_x, goal_y),
            grid_map=self.grid_map,
            step_size=1.0,
            goal_radius=2.0,
            max_iterations=1000,
            neighbor_radius=3.0
        )
        path_coords = planner.plan()  # This should be a list of (x, y)

        # 4) If path_coords is empty, return an empty list of Point2D
        if not path_coords:
            self.get_logger().warn("No path found. Returning empty path.")
            response.path = []
            return response

        # 5) Convert each (x, y) into a Point2D
        path_points = []
        for (px, py) in path_coords:
            point_msg = Point2D()
            point_msg.x = px
            point_msg.y = py
            path_points.append(point_msg)

        # 6) Fill the response with the list of Point2D
        response.path = path_points

        self.get_logger().info(f"Path found with {len(path_points)} waypoints.")
        return response

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanningService()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()