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
    Param: Used to pass a list of parameters from child nodes into the Node parent class
        to be declared as ROS params and set as member variables

    Allowed parameter types in ROS2:

        ROS 2 IDL type	    rclpy type (actually used in code)

        bool	            BOOL
        int64	            INTEGER
        float64	            DOUBLE
        string	            STRING
        byte[]	            BYTE_ARRAY
        bool[]	            BOOL_ARRAY
        int64[]	            INTEGER_ARRAY
        float64[]	        DOUBLE_ARRAY
        string[]	        STRING_ARRAY
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
