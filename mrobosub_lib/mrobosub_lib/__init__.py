"""
mrobosub_lib: Currently this package doesnt do much other than start the node and
call run on a separate thread
"""

import rclpy
from rclpy.node import Node as RosNode
from typing import Callable

import threading


class Node(RosNode):
    def __init__(self, node_name: str, *args, **kawrgs):
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")

    def _start_thread(self):
        self.__thread = threading.Thread(target=self.run, daemon=True)
        self.__thread.start()

    def run(self):
        """
        This function is intended to be called on a separate thread for an event loop

        If `mrobosub_lib.main` is called, this function will be automatically executed while the node is being spun
        Otherwise, self._start_thread() must be called to run this function in a separate thread
        """
        pass

    def cleanup(self):
        self.destroy_node()
        self.__thread.join()


def main(constructor: type[Node], *args, **kwargs):
    """
    Main entry point for constructing and spinning mrobosub nodes
    args and kwargs are passed to node constructor
    """
    rclpy.init()

    node = constructor(*args, **kwargs)

    node._start_thread()
    rclpy.spin(node)

    node.cleanup()
    rclpy.shutdown()
