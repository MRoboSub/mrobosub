"""
mrobosub_lib: Currently this package doesn't do much other than print the node is starting.
"""

import rclpy
from rclpy.node import Node as RosNode
from typing import Callable

class Node(RosNode):
    def __init__(self, node_name: str, *args, **kawrgs):
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")
        # TODO: what about ros params?? we were parsing them here in ros1 but are now parsing them nowhere

    def run(self):
        pass
