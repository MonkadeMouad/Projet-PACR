import numpy as np
import heapq
import random
from pacr_simulation.map_utils import GridMap
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathPlanner:
    def __init__(self, method="A*"):
        self.method = method
        self.grid_map = None  # Stocke la carte

    def set_map(self, grid_map: GridMap):
        """Initialise la carte pour la planification."""
        self.grid_map = grid_map

    def plan_path(self, start, goal):
        """Calcule un chemin en fonction de la méthode choisie."""
        if self.grid_map is None:
            return None  # Pas de carte

        if self.method == "A*":
            return self._a_star(start, goal)
        elif self.method == "RRT*":
            return self._rrt_star(start, goal)
        else:
            raise ValueError("Méthode non supportée")

    def _a_star(self, start, goal):
        """Implémentation de l'algorithme A*."""
        def heuristic(a, b):
            return np.linalg.norm(np.array(a) - np.array(b))

        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0), (1, 1), (-1, -1), (1, -1), (-1, 1)]

        open_set = []
        heapq.heappush(open_set, (0, start))
        came_from = {}
        g_score = {start: 0}
        f_score = {start: heuristic(start, goal)}

        while open_set:
            _, current = heapq.heappop(open_set)

            if current == goal:
                path = []
                while current in came_from:
                    path.append(current)
                    current = came_from[current]
                path.append(start)
                return path[::-1]

            for dx, dy in neighbors:
                neighbor = (current[0] + dx, current[1] + dy)

                if not self.grid_map.coll_free(current, neighbor):
                    continue

                tentative_g_score = g_score[current] + heuristic(current, neighbor)

                if neighbor not in g_score or tentative_g_score < g_score[neighbor]:
                    came_from[neighbor] = current
                    g_score[neighbor] = tentative_g_score
                    f_score[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                    heapq.heappush(open_set, (f_score[neighbor], neighbor))

        return None  # Aucun chemin trouvé

    def _rrt_star(self, start, goal, max_iter=1000, step_size=5, goal_sample_rate=0.1):
        """Implémentation de RRT*."""
        class Node:
            def __init__(self, pos, parent=None):
                self.pos = pos
                self.parent = parent
                self.cost = 0 if parent is None else parent.cost + np.linalg.norm(np.array(pos) - np.array(parent.pos))

        def get_random_point():
            if random.random() < goal_sample_rate:
                return goal
            return (random.randint(0, self.grid_map.width), random.randint(0, self.grid_map.height))

        def nearest_node(nodes, point):
            return min(nodes, key=lambda node: np.linalg.norm(np.array(node.pos) - np.array(point)))

        def steer(from_node, to_point, step_size):
            direction = np.array(to_point) - np.array(from_node.pos)
            distance = np.linalg.norm(direction)
            if distance < step_size:
                return to_point
            return tuple(np.array(from_node.pos) + (direction / distance) * step_size)

        nodes = [Node(start)]
        for _ in range(max_iter):
            rand_point = get_random_point()
            nearest = nearest_node(nodes, rand_point)
            new_pos = steer(nearest, rand_point, step_size)

            if not self.grid_map.coll_free(new_pos):
                continue

            new_node = Node(new_pos, nearest)

            # Rewire step (RRT*)
            for node in nodes:
                if np.linalg.norm(np.array(node.pos) - np.array(new_node.pos)) < step_size * 2:
                    if node.cost + np.linalg.norm(np.array(node.pos) - np.array(new_node.pos)) < new_node.cost:
                        new_node.parent = node
                        new_node.cost = node.cost + np.linalg.norm(np.array(node.pos) - np.array(new_node.pos))

            nodes.append(new_node)

            if np.linalg.norm(np.array(new_node.pos) - np.array(goal)) < step_size:
                path = []
                while new_node:
                    path.append(new_node.pos)
                    new_node = new_node.parent
                return path[::-1]

        return None  # Aucun chemin trouvé

    def convert_to_ros_path(self, path):
        """Convertit un chemin en message ROS Path."""
        ros_path = Path()
        ros_path.header.frame_id = "map"
        for p in path:
            pose = PoseStamped()
            pose.pose.position.x, pose.pose.position.y = p
            ros_path.poses.append(pose)
        return ros_path
