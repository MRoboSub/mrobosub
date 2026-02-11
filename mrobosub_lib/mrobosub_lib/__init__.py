"""
mrobosub_lib: Currently this package doesn't do much other than print the node is starting.
"""

from typing import Any
from rclpy.node import Node as RosNode


class Node(RosNode):
    def __init__(self, node_name: str, *args: Any, **kawrgs: Any) -> None:
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")

    def run(self) -> None:
        pass
