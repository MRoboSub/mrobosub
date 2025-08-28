import rclpy
from rclpy.node import Node as RosNode
from typing import Callable


class Node(RosNode):
    def __init__(self, node_name: str, *args, **kawrgs):
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")

    def run(self):
        pass

    def cleanup(self):
        self.destroy_node()


def main(constructor: Callable[[], Node]):
    rclpy.init()

    node = constructor()
    node.run()
    node.cleanup()

    rclpy.shutdown()
