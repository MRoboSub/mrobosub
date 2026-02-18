"""
mrobosub_lib: this package prints that the node is starting
and parses params + takes care of updating the member variable values at runtime whenever any of the param values are changed
"""

from typing import Any, List
from rclpy.node import Node as RosNode
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor

class Param:
    """
    Param: used to pass in list of params from individual nodes to the superclass Node to declare and set params
    
    Allowed parameter types in ROS2: bool, int64, float64, string, byte[], bool[], int64[], float64[] or string[]
    """
    name: str
    type: Parameter.Type
    description: ParameterDescriptor

    def __init__(self, name: str, type: Parameter.Type, description: str):
        self.name = name
        self.type = type
        self.description = ParameterDescriptor(description=description)

class Node(RosNode):
    def __init__(self, node_name: str, *args: Any, **kawrgs: Any) -> None:
        super().__init__(node_name, *args, **kawrgs)
        self.get_logger().info(f"starting node {node_name}...")

    def set_params(self, _params = None):
        for param in self.param_list:
            setattr(self, param.name, self.get_parameter(param.name).value)

    def declare_params(self, param_list: List[Param]):
        self.param_list = param_list

        for param in param_list:
            self.declare_parameter(param.name, param.type, param.description)

        self.set_params()

        # whenever a parameter value changes, set_params will be called as a callback
        # to update all the class variable values based on latest param values
        self.add_post_set_parameters_callback(self.set_params)

    def run(self) -> None:
        pass
