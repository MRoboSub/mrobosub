import rclpy
from rclpy.parameter import Parameter

from mrobosub_lib import Node, Param
from serial import Serial
from serial.serialutil import SerialException
from mrobosub_msgs.msg import MotorState
from mrobosub_msgs.msg import PololuCommands
from std_srvs.srv import SetBool
from typing import Optional

import numpy as np


NUM_MOTORS = 8


class ThrusterController(Node):
    """
    Provides /emergency_stop_motors service
    and requires /motor_output topic
    """

    def __init__(self):
        super().__init__("thruster_controller")
        self.get_logger().info("Launched thruster_controller node")
        self.emergency_stop = False
        self.motor_outputs = [0] * NUM_MOTORS

        self.object_position_service = self.create_service(
            SetBool, "emergency_stop_motors", self.handle_emergency_stop
        )
        self.motor_sub = self.create_subscription(
            MotorState, "/motor_output", self.motor_callback, 1
        )
        self.pololu_pub = self.create_publisher(
            PololuCommands, "/pololu_commands", qos_profile=1
        )

        self.declare_parameter(
            "thruster_reverse",
            rclpy.Parameter.Type.BOOL_ARRAY,
            descriptor=ParameterDescriptor(
                description="List of booleans indicating whether each thruster is reversed"
            ),
        )

        self.declare_parameter(
            "thruster_motor_id",
            rclpy.Parameter.Type.INTEGER_ARRAY,
            descriptor=ParameterDescriptor(
                description="List of motor IDs for each thruster on the thruster controller"
            ),
        )
        params = [Param('thruster_reverse', rclpy.Parameter.Type.BOOL_ARRAY, "List of booleans indicating whether each thruster is reversed"), 
                  Param('thruster_motor_id', rclpy.Parameter.Type.INTEGER_ARRAY, "List of motor IDs for each thruster on the thruster controller")
                  ]
        
        self.declare_params(params)


    def handle_emergency_stop(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        if req.data:
            self.emergency_stop = True
        else:
            self.emergency_stop = False
        res.success = True
        return res


    # in case of invalid PWM or motor number parameters, does not send any updated signal to the motor controller
    def publish_motor_outputs(self, msg: MotorState) -> int:
        message_valid = [False]*12
        message_output = [0.0]*12
        
        if self.emergency_stop:
            for motor in range(NUM_MOTORS):

                motor_pin = (self.thruster_motor_id[motor])

                message_valid[motor_pin] = True
                message_output[motor_pin] = 0
        else:
            for motor in range(NUM_MOTORS):
                motor_pin = (self.thruster_motor_id[motor])

                message_valid[motor_pin] = True

                if self.thruster_reverse[motor]:  # type: ignore
                    message_output[motor_pin] = -msg.motors[motor]
                else:
                    message_output[motor_pin] = msg.motors[motor]


        msg = PololuCommands()
        message_output = np.array(message_output, dtype=np.float32)
        msg.pins = message_output
        msg.valid = message_valid

        self.pololu_pub.publish(msg)
        # self.get_logger().info(f"Thruster controller: sent pwm value {pwm_val} to motor {motor}")

        return 0

    def motor_callback(self, msg: MotorState):
        self.publish_motor_outputs(msg)

def main():
    rclpy.init()
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
