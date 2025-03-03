#!/usr/bin/env python3
"""
controller_node.py
Provides a ROS 2 service named "get_cmd" (GetCmd.srv)
which computes a velocity command (linear, angular)
to move the robot along a given path toward a goal.

Message definitions (in pacr_interfaces):
 - Pose2D: x, y, theta
 - Velocity2D: linear, angular
 - Point2D: x, y
 - GetCmd.srv
"""

import math
import rclpy
from rclpy.node import Node

# Service import (généré après compilation de pacr_interfaces)
from pacr_interfaces.srv import GetCmd
from pacr_interfaces.msg import Pose2D, Velocity2D, Point2D

class LocalControllerNode(Node):
    def __init__(self):
        super().__init__('local_controller_node')
        # ...
        # Variable pour stocker la "vraie" path
        self.current_path = []
        self.path_received = False  # Savoir si on a déjà reçu un path
        # Exemple de paramètres (facultatif)
        self.declare_parameter('max_lin_speed', 0.5)
        self.declare_parameter('max_ang_speed', 1.0)

        self.max_lin_speed = self.get_parameter('max_lin_speed').get_parameter_value().double_value
        self.max_ang_speed = self.get_parameter('max_ang_speed').get_parameter_value().double_value

        # Création du service "get_cmd"
        self.srv_ = self.create_service(
            GetCmd,             # Le type de service
            'get_cmd',          # Le nom ROS du service
            self.get_cmd_callback
        )

        self.get_logger().info('LocalControllerNode lancé : service "/get_cmd" prêt.')

    def get_cmd_callback(self, request, response):
        if not self.path_received:
            self.current_path = list(request.path)  # on copie
            self.path_received = True

        # 2) On travaille toujours sur self.current_path
        if len(self.current_path) == 0:
            response.finished = True
            response.command.linear = 0.0
            response.command.angular = 0.0
            return response

        # 3) On vise le 1er waypoint
        target = self.current_path[0]
        dx = target.x - request.pose.x
        dy = target.y - request.pose.y
        dist = math.sqrt(dx*dx + dy*dy)

        distance_tolerance = 0.1
        if dist < distance_tolerance:
            # On enlève ce waypoint
            self.current_path.pop(0)
            # On s'arrête ce cycle
            response.finished = (len(self.current_path) == 0)
            response.command.linear = 0.0
            response.command.angular = 0.0
            return response

        # 4) Sinon, calculer l'angle, etc. 
        desired_angle = math.atan2(dy, dx)
        angle_error = desired_angle - request.pose.theta
        # ... normalisation ...
        angle_error = math.atan2(math.sin(angle_error), math.cos(angle_error))

        # Tourner sur place si ...
        angle_tolerance = 0.1
        if abs(angle_error) > angle_tolerance:
            v = 0.0
            w = 0.5 * angle_error
        else:
            v = 0.2
            w = 0.0

        # saturation ...
        v = max(min(v, self.max_lin_speed), -self.max_lin_speed)
        w = max(min(w, self.max_ang_speed), -self.max_ang_speed)

        response.finished = False
        response.command.linear = v
        response.command.angular = w
        return response


def main(args=None):
    rclpy.init(args=args)
    node = LocalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
