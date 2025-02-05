import numpy as np
import heapq
import random
from pacr_simulation.map_utils import GridMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

# ============================================================================
# Custom point class to enable arithmetic and attribute access.
# ============================================================================
class Point2D:
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __sub__(self, other):
        return Point2D(self.x - other.x, self.y - other.y)

    def __truediv__(self, scalar):
        return Point2D(self.x / scalar, self.y / scalar)

    def __add__(self, other):
        return Point2D(self.x + other.x, self.y + other.y)

    def __eq__(self, other):
        if not isinstance(other, Point2D):
            return False
        return self.x == other.x and self.y == other.y

    def __hash__(self):
        return hash((self.x, self.y))

    def as_tuple(self):
        return (self.x, self.y)

    def __repr__(self):
        return f"Point2D({self.x}, {self.y})"


# ============================================================================
# PathPlanner class uses A* (or RRT*) and works with Point2D objects.
# ============================================================================
class PathPlanner:
    def __init__(self, method="A*"):
        self.method = method
        self.grid_map = None  # Will hold a GridMap instance

    def set_map(self, grid_map: GridMap):
        """Initialize the map for planning.
        
        Converts grid_map.origin (if not already a Point2D) to a Point2D.
        """
        if not isinstance(grid_map.origin, Point2D):
            grid_map.origin = Point2D(grid_map.origin[0], grid_map.origin[1])
        self.grid_map = grid_map

    def plan_path(self, start, goal):
        """Plan a path using the chosen method.
        
        `start` and `goal` are expected as (x, y) tuples.
        """
        if self.grid_map is None:
            return None  # No map available

        if self.method == "A*":
            return self._a_star(start, goal)
        elif self.method == "RRT*":
            return self._rrt_star(start, goal)
        else:
            raise ValueError("Unsupported planning method")

    # --------------------------------------------------------------------------
    # A* implementation using Point2D.
    # --------------------------------------------------------------------------
    def _a_star(self, start, goal):
        """A* algorithm implementation.
        
        Converts start and goal into Point2D and generates neighbors accordingly.
        """
        def heuristic(a, b):
            return np.linalg.norm(np.array([a.x, a.y]) - np.array([b.x, b.y]))

        # Convert start and goal to Point2D objects
        start_pt = Point2D(start[0], start[1])
        goal_pt = Point2D(goal[0], goal[1])

        # Define 8-connected neighborhood
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0),
                     (1, 1), (-1, -1), (1, -1), (-1, 1)]

        open_set = []
        heapq.heappush(open_set, (0, start_pt))
        came_from = {}
        g_score = {start_pt: 0}
        f_score = {start_pt: heuristic(start_pt, goal_pt)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal_pt:
                # Reconstruct path
                path = []
                while current in came_from:
                    path.append(current.as_tuple())
                    current = came_from[current]
                path.append(start_pt.as_tuple())
                return path[::-1]

            for dx, dy in neighbors:
                neighbor = Point2D(current.x + dx, current.y + dy)
                # The coll_free method (in map_utils.py) will use:
                #   (p - grid_map.origin) / grid_map.resolution
                # so grid_map.origin is already a Point2D.
                if not self.grid_map.coll_free(current, neighbor):
                    continue

                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal_pt)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # No path found

    # --------------------------------------------------------------------------
    # RRT* implementation using Point2D (optional, similar treatment as A*).
    # --------------------------------------------------------------------------
    def _rrt_star(self, start, goal, max_iter=1000, step_size=5, goal_sample_rate=0.1):
        """RRT* algorithm implementation using Point2D objects."""
        # Convert start and goal to Point2D objects
        start_pt = Point2D(start[0], start[1])
        goal_pt = Point2D(goal[0], goal[1])

        class Node:
            def __init__(self, pos, parent=None):
                self.pos = pos
                self.parent = parent
                self.cost = 0 if parent is None else parent.cost + np.linalg.norm(
                    np.array([pos.x, pos.y]) - np.array([parent.pos.x, parent.pos.y])
                )

        def get_random_point():
            if random.random() < goal_sample_rate:
                return goal_pt
            # Use grid_map dimensions (if available) or fallback
            if hasattr(self.grid_map, 'dimensions'):
                width, height = self.grid_map.dimensions
            else:
                # Fallback: infer dimensions from the data array
                height, width = self.grid_map.data.shape
            return Point2D(random.randint(0, width - 1), random.randint(0, height - 1))

        def nearest_node(nodes, point):
            return min(nodes, key=lambda node: np.linalg.norm(
                np.array([node.pos.x, node.pos.y]) - np.array([point.x, point.y])
            ))

        def steer(from_node, to_point, step_size):
            direction = np.array([to_point.x - from_node.pos.x, to_point.y - from_node.pos.y])
            distance = np.linalg.norm(direction)
            if distance < step_size:
                return to_point
            new_coords = np.array([from_node.pos.x, from_node.pos.y]) + (direction / distance) * step_size
            return Point2D(new_coords[0], new_coords[1])

        nodes = [Node(start_pt)]
        for _ in range(max_iter):
            rand_point = get_random_point()
            nearest = nearest_node(nodes, rand_point)
            new_pos = steer(nearest, rand_point, step_size)

            if not self.grid_map.coll_free(nearest.pos, new_pos):
                continue

            new_node = Node(new_pos, nearest)

            # Rewire step
            for node in nodes:
                dist = np.linalg.norm(np.array([node.pos.x, node.pos.y]) - np.array([new_node.pos.x, new_node.pos.y]))
                if dist < step_size * 2:
                    cost = node.cost + dist
                    if cost < new_node.cost:
                        new_node.parent = node
                        new_node.cost = cost

            nodes.append(new_node)

            if np.linalg.norm(np.array([new_node.pos.x, new_node.pos.y]) - np.array([goal_pt.x, goal_pt.y])) < step_size:
                # Reconstruct the path
                path = []
                current = new_node
                while current is not None:
                    path.append(current.pos.as_tuple())
                    current = current.parent
                return path[::-1]

        return None  # No path found

    # --------------------------------------------------------------------------
    # Convert a planned path (list of (x, y) tuples) into a ROS Path message.
    # --------------------------------------------------------------------------
    def convert_to_ros_path(self, path):
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x = p[0]
            pose.pose.position.y = p[1]
            ros_path.poses.append(pose)
        return ros_path
