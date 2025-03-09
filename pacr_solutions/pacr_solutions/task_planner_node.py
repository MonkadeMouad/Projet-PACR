# task_planner_node.py

import rclpy
from rclpy.node import Node

from pacr_interfaces.srv import GetAction
from geometry_msgs.msg import Pose2D

from pacr_solutions.MDP import (
    build_mdp,
    value_iteration,
    WAIT
)

class TaskPlannerNode(Node):
    def __init__(self):
        super().__init__('task_planner_node')

        # Construire et résoudre le MDP
        self.mdp = build_mdp()
        self.policy, _ = value_iteration(self.mdp, epsilon=0.01)
        self.get_logger().info("MDP initialisé et politique optimale calculée.")

        # Service get_action
        self.srv = self.create_service(
            GetAction,
            'get_action',
            self.get_action_callback
        )

    def get_action_callback(self, request, response):
        """
        Calcule l'action pour l'état (robot_location, object_state).
        """
        state = (request.controlled_robot_location, request.object_state)
        self.get_logger().info(f"Requête reçue : état actuel = {state}")

        # Cherche l'action dans la policy
        action = self.policy.get(state, WAIT)

        # Met l'action dans la réponse
        response.action = action
        # S'il s'agit de GOTO_OTHER, on pourrait fixer response.target_pose
        return response

def main(args=None):
    rclpy.init(args=args)
    node = TaskPlannerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
