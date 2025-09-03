import rclpy
from rclpy.node import Node as RosNode
from typing import Callable

# TODO: Is adding threading in this way the most robust way to do this?
import threading

class Node(RosNode):
    def __init__(self, node_name: str, *args, **kawrgs):
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")

        self._thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        # muskaan note: why are we using daemon = True here?
        self._thread.start()

    def run(self):
        pass

    def cleanup(self):
        self.destroy_node()
        self._thread.join()


def main(constructor: Callable[[], Node]):
    rclpy.init()

    node = constructor()
    
    node.run()
    node.cleanup()

    rclpy.shutdown()
    
