from rclpy.parameter import Parameter
from typing import Dict, Type

PARAMETER_TYPE_TO_NATIVE: Dict[Parameter.Type, Type] = {
    Parameter.Type.BOOL: bool,
    Parameter.Type.INTEGER: int,
    Parameter.Type.DOUBLE: float,
    Parameter.Type.STRING: str,
    Parameter.Type.BYTE_ARRAY: bytes,
    Parameter.Type.BOOL_ARRAY: list,
    Parameter.Type.INTEGER_ARRAY: list,
    Parameter.Type.DOUBLE_ARRAY: list,
    Parameter.Type.STRING_ARRAY: list,
    Parameter.Type.NOT_SET: type(None)
}

