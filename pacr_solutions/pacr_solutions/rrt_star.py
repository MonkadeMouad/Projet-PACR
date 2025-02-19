#!/usr/bin/env python3
import math
import random
from typing import List, Tuple

# Import the GridMap and Point from your map_utils
from pacr_simulation.map_utils import GridMap, Point


class Node:
    """
    Simple Node structure to hold the tree info:
      x, y    -> coordinates
      parent  -> reference to another Node
      cost    -> cumulative cost from start
    """
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0


class RRTStar:
    """
    Minimal RRT* algorithm using the methods from GridMap:
      - sample_free() for random free-space sampling
      - coll_free(p1, p2) for collision checking between two points

    The 'plan' method returns a list of (x, y) points if a path is found,
    or an empty list otherwise.
    """

    def __init__(
        self,
        start: Tuple[float, float],
        goal: Tuple[float, float],
        grid_map: GridMap,
        step_size: float = 1.0,
        goal_radius: float = 2.0,
        max_iterations: int = 1000,
        neighbor_radius: float = 3.0
    ):
        """
        :param start: (start_x, start_y)
        :param goal:  (goal_x, goal_y)
        :param grid_map: Instance of your GridMap class
        :param step_size: Maximum distance to "steer" from one node toward a sample
        :param goal_radius: Distance threshold to consider the goal "reached"
        :param max_iterations: How many times we expand the tree before giving up
        :param neighbor_radius: Radius for local neighborhood when re-wiring
        """
        self.start = Node(start[0], start[1])
        self.goal = Node(goal[0], goal[1])
        self.grid_map = grid_map

        self.step_size = step_size
        self.goal_radius = goal_radius
        self.max_iterations = max_iterations
        self.neighbor_radius = neighbor_radius

        # Our tree is simply a list of nodes, with each node referencing its parent
        self.nodes = [self.start]

        # You could seed the random generator if you want reproducible results
        self.rng = random.Random()

    def plan(self) -> List[Tuple[float, float]]:
        """
        Main loop of RRT*:
          1) Sample a random point from free space
          2) Find nearest existing node
          3) Steer (limit the new node to 'step_size' away)
          4) Collision check
          5) Find near nodes within 'neighbor_radius'
          6) Choose best parent among near nodes (lowest cost + collision-free)
          7) Rewire near nodes to potentially lower their costs
          8) Check if we are within 'goal_radius' of goal
             -> if yes and collision-free to goal, return final path
        """
        for _ in range(self.max_iterations):
            # 1) Sample a free point in the map (in world coordinates)
            sample_pt = self.grid_map.sample_free(rng=self.rng)

            # 2) Nearest node
            nearest_node = self.get_nearest(sample_pt)

            # 3) Steer to get a new node (limiting distance to step_size)
            new_node = self.steer(nearest_node, sample_pt)

            # 4) Collision check from nearest to new_node
            if not self.grid_map.coll_free(Point(nearest_node.x, nearest_node.y),
                                           Point(new_node.x, new_node.y)):
                continue  # If collision, skip

            # 5) Find nodes near 'new_node' within a 'neighbor_radius'
            near_nodes = self.get_near_nodes(new_node)

            # 6) Choose the best parent among near_nodes
            new_node = self.choose_best_parent(new_node, near_nodes)

            # Add new_node to our tree
            self.nodes.append(new_node)

            # 7) Rewire the near_nodes to see if we can lower their cost
            self.rewire(new_node, near_nodes)

            # 8) Check goal
            if self.distance(new_node, self.goal) < self.goal_radius:
                # Check collision from new_node directly to goal
                if self.grid_map.coll_free(Point(new_node.x, new_node.y),
                                           Point(self.goal.x, self.goal.y)):
                    return self.extract_path(new_node)

        # If no path found after max_iterations, return an empty list
        return []

    def steer(self, from_node: Node, to_point: Point) -> Node:
        """
        Return a new node in the direction of 'to_point' but limited to 'step_size'.
        """
        dist = math.dist((from_node.x, from_node.y), (to_point.x, to_point.y))
        if dist < self.step_size:
            # If it's already within step_size, create a node at to_point
            new_x, new_y = to_point.x, to_point.y
        else:
            # Otherwise, move 'step_size' in that direction
            theta = math.atan2(to_point.y - from_node.y, to_point.x - from_node.x)
            new_x = from_node.x + self.step_size * math.cos(theta)
            new_y = from_node.y + self.step_size * math.sin(theta)

        new_node = Node(new_x, new_y)
        # The new node's parent is (by default) the from_node
        new_node.parent = from_node
        # Its cost is parent's cost plus the distance traveled
        new_node.cost = from_node.cost + min(dist, self.step_size)
        return new_node

    def get_nearest(self, pt: Point) -> Node:
        """
        Return the node from self.nodes that is closest to 'pt'.
        """
        # Using math.dist to measure the Euclidean distance
        nearest_node = min(self.nodes, key=lambda node: math.dist((node.x, node.y), (pt.x, pt.y)))
        return nearest_node

    def get_near_nodes(self, new_node: Node) -> List[Node]:
        """
        Return nodes within 'neighbor_radius' of new_node.
        """
        near = []
        for node in self.nodes:
            if self.distance(node, new_node) < self.neighbor_radius:
                near.append(node)
        return near

    def choose_best_parent(self, new_node: Node, near_nodes: List[Node]) -> Node:
        """
        Among 'near_nodes', pick the one that yields the lowest total cost
        while being collision-free.
        """
        best_parent = new_node.parent
        best_cost = new_node.cost

        for near_node in near_nodes:
            # cost if we connect new_node to near_node
            dist = self.distance(near_node, new_node)
            potential_cost = near_node.cost + dist

            if potential_cost < best_cost:
                # Check collision from near_node to new_node
                if self.grid_map.coll_free(Point(near_node.x, near_node.y),
                                           Point(new_node.x, new_node.y)):
                    best_parent = near_node
                    best_cost = potential_cost

        new_node.parent = best_parent
        new_node.cost = best_cost
        return new_node

    def rewire(self, new_node: Node, near_nodes: List[Node]) -> None:
        """
        Attempt to rewire the 'near_nodes' through new_node if it improves cost.
        """
        for near_node in near_nodes:
            dist = self.distance(new_node, near_node)
            potential_cost = new_node.cost + dist
            # If going from new_node -> near_node is cheaper
            if potential_cost < near_node.cost:
                # and collision-free
                if self.grid_map.coll_free(Point(new_node.x, new_node.y),
                                           Point(near_node.x, near_node.y)):
                    near_node.parent = new_node
                    near_node.cost = potential_cost

    def distance(self, n1: Node, n2: Node) -> float:
        """Simple Euclidean distance between two Nodes."""
        return math.dist((n1.x, n1.y), (n2.x, n2.y))

    def extract_path(self, node: Node) -> List[Tuple[float, float]]:
        """
        Traverse from 'node' up to the start, building the path in reverse.
        Then reverse it so it goes start -> ... -> goal.
        We also append the actual goal (since we tested collision to goal).
        """
        path = [(self.goal.x, self.goal.y)]
        current = node
        while current is not None:
            path.append((current.x, current.y))
            current = current.parent
        path.reverse()
        return path

