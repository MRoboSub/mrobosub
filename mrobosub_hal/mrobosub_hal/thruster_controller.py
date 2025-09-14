#!/usr/bin/env python

import rclpy
from rcl_interfaces.msg import ParameterDescriptor

from mrobosub_lib import Node
from serial import Serial
from serial.serialutil import SerialException
from mrobosub_msgs.msg import MotorState
from std_srvs.srv import SetBool
from typing import Optional


NUM_MOTORS = 8


class ThrusterController(Node):
    """
    Provides /emergency_stop_motors service
    and requires /motor_output topic
    """

    def __init__(self):
        super().__init__("thruster_controller")
        self.get_logger().info("Launched thruster_controller node")
        self.port = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00467345-if00"
        self.emergency_stop = False
        self.motor_outputs = [0] * NUM_MOTORS
        self.serial: Serial | None = None
        self.connect()
        self.get_errors()  # clear errors at the start

        self.object_position_service = self.create_service(
            SetBool, "emergency_stop_motors", self.handle_emergency_stop
        )
        self.motor_sub = self.create_subscription(
            MotorState, "/motor_output", self.motor_callback, 1
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

        self.timer = self.create_timer(1.0/50, self.loop)

    def connect(self) -> bool:
        try:
            self.serial = Serial(self.port, timeout=0.5, write_timeout=0.5)
        except SerialException as e:
            self.get_logger().info(f"Could not connect to mini maestro: {e}")
            return False
        return True

    def write(self, data: bytearray) -> bool:
        if self.serial is None:
            success = self.connect()
            if not success or self.serial is None:
                return False
        try:
            self.serial.write(data)
            return True
        except SerialException as e:
            self.get_logger().info(f"write error: {e}")
            self.serial.close()
            self.connect()
        return False

    def read(self, len: int) -> Optional[bytes]:
        if self.serial is None:
            success = self.connect()
            if not success or self.serial is None:
                return None
        try:
            return self.serial.read(len)
        except SerialException as e:
            self.get_logger().info(f"read error: {e}")
            self.serial.close()
            self.connect()
        return None

    def handle_emergency_stop(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        self.emergency_stop = True
        self.get_errors()
        self.write(bytearray([0xAA, 0x0C, 0x22]))
        res.success = True
        return res

    # pwm_raw should be in [-1, 1]
    # pwm_val should be in [4000, 8000]
    def convert_pwm_signal(self, pwm_raw: float) -> Optional[int]:
        if pwm_raw < -1 or pwm_raw > 1:
            self.get_logger().info(
                f"Thruster Controller [ERROR]: PWM value {pwm_raw} out of range (should be in [-1, 1])"
            )
            return None
        return int((pwm_raw * 1600) + 6000)

    # in case of invalid PWM or motor number parameters, does not send any updated signal to the motor controller
    def send_signal(self, motor: int, pwm_raw: float) -> int:
        if self.get_parameter("thruster_reverse").get_parameter_value().bool_array_value[motor]:  # type: ignore
            pwm_raw *= -1
        pwm_val = self.convert_pwm_signal(pwm_raw)
        if pwm_val is None:
            return -1

        if motor < 0 or motor >= NUM_MOTORS:
            self.get_logger().info(
                f"ERROR: motor number {motor} out of range (should be in [0, {NUM_MOTORS-1}])"
            )
            return -1

        motor = (
            self.get_parameter("thruster_motor_id")
            .get_parameter_value()
            .integer_array_value[motor]
        )

        LSBs = pwm_val % (2**7)
        MSBs = int(pwm_val / (2**7))

        self.get_errors()
        self.write(bytearray([0xAA, 0x0C, 0x04, motor, LSBs, MSBs]))
        # self.get_logger().info(f"Thruster controller: sent pwm value {pwm_val} to motor {motor}")

        return 0

    def motor_callback(self, msg: MotorState):
        if not self.emergency_stop:
            for i in range(NUM_MOTORS):
                motor_name = f"motor{i}"
                self.motor_outputs[i] = getattr(msg, motor_name)

    def get_errors(self):
        # gets errors from thruster controller hardware (which automatically clears the errors too)
        self.write(bytearray([0xAA, 0x0C, 0x21]))
        error = self.read(2)
        if error is None:
            return
        error_code = int.from_bytes(error, "little")
        if error_code != 0:
            self.get_logger().info(f"Thruster controller: error code = {error_code}")
            # eg: error_code 16 means 00010000 which is the 5th error bit set

    def loop(self):
        for i in range(NUM_MOTORS):
            self.send_signal(i, self.motor_outputs[i])


def main():
    rclpy.init()
    node = ThrusterController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
