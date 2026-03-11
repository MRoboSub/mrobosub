# in-progress
import rclpy
from rcl_interfaces.msg import ParameterDescriptor

from mrobosub_lib import Node
from serial import Serial
from serial.serialutil import SerialException
from mrobosub_msgs.msg import PololuCommands
from std_srvs.srv import SetBool
from typing import Optional


NUM_PINS = 12
FREQUENCY = 50


class Pololu(Node):
    """
    Provides requires /pololu_commands topic
    """

    def __init__(self):
        super().__init__("pololu")
        self.get_logger().info("Launched pololu node")
        self.port = "/dev/serial/by-id/usb-Pololu_Corporation_Pololu_Mini_Maestro_12-Channel_USB_Servo_Controller_00467345-if00"
        self.pololu_outputs = [0] * NUM_PINS
        self.serial: Serial | None = None
        self.connect()
        self.get_errors()  # clear errors at the start

        self.pololu_sub = self.create_subscription(
            PololuCommands, "/pololu_commands", self.pololu_commands_callback, 1
        )
        self.timer = self.create_timer(1.0/FREQUENCY, self.loop)

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

    # pwm_raw should be in [-1, 1]
    # pwm_val should be in [4000, 8000]
    def convert_pwm_signal(self, pwm_raw: float) -> Optional[int]:
        if pwm_raw < -1 or pwm_raw > 1:
            self.get_logger().info(
                f"Pololu [ERROR]: PWM value {pwm_raw} out of range (should be in [-1, 1])"
            )
            return None
        return int((pwm_raw * 1600) + 6000)

    # in case of invalid PWM or pin number parameters, does not send any updated signal to the pin controller
    def send_signal(self, pin: int, pwm_raw: float) -> int:
        pwm_val = self.convert_pwm_signal(pwm_raw)
        if pwm_val is None:
            return -1

        if pin < 0 or pin >= NUM_PINS:
            self.get_logger().info(
                f"ERROR: pin number {pin} out of range (should be in [0, {NUM_PINS-1}])"
            )
            return -1

        LSBs = pwm_val % (2**7)
        MSBs = int(pwm_val / (2**7))

        self.get_errors()
        self.write(bytearray([0xAA, 0x0C, 0x04, pin, LSBs, MSBs]))

        return 0

    def pololu_commands_callback(self, msg: PololuCommands):
        for i in range(NUM_PINS):
            if msg.valid[i]:
                self.pololu_outputs[i] = msg.pins[i]

    def get_errors(self):
        self.write(bytearray([0xAA, 0x0C, 0x21]))
        error = self.read(2)
        if error is None:
            return
        error_code = int.from_bytes(error, "little")
        if error_code != 0:
            self.get_logger().info(f"Pololu: error code = {error_code}")
            # eg: error_code 16 means 00010000 which is the 5th error bit set

    def loop(self):
        for i in range(NUM_PINS):
            self.send_signal(i, self.pololu_outputs[i])


def main():
    rclpy.init()
    node = Pololu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
