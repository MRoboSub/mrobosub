"""
mrobosub_lib: Currently this package doesnt do much other than start the node and
call run on a separate thread
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
        """
        This function is intended to be called on a separate thread for an event loop

        If `mrobosub_lib.main` is called, this function will be automatically executed while the node is being spun
        Otherwise, self._start_thread() must be called to run this function in a separate thread
        """
        pass
