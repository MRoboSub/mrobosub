import rclpy
from rcl_interfaces.msg import ParameterDescriptor

from std_srvs.srv import SetBool

from mrobosub_lib import Node
from mrobosub_msgs.msg import ThrusterCommands

from typing import Optional
from serial import Serial
from serial.serialutil import SerialException
import sys

# Constants
NUM_PINS = 8
FREQUENCY = 50

class ESP32_Thruster(Node):
    """
    Provides requires /esp32/thruster_commands topic
    """

    def __init__(self):
        super().__init__("esp32_thruster")

        if len(sys.argv) < 2:
            raise ValueError("Must give a device id as argument.")


        self.get_logger().info("Launched esp32_thruster node")
        
        # Params make more sense for parameters that can change dynamically with 
        # the program running. This is a static argument that should be loaded 
        # once on start up.
        self.port = sys.argv[1]

        self.thruster_outputs = [0] * NUM_PINS
        self.serial: Serial | None = None
        self.connect()

        self.esp32_thruster_sub = self.create_subscription(
            ThrusterCommands, "/esp32/thruster_commands", self.eps32_thruster_commands_callback, 1
        )
        self.emergency_stop_service = self.create_service(
            SetBool, "emergency_stop_motors", self.handle_emergency_stop
        )
        self.timer = self.create_timer(1.0/FREQUENCY, self.loop)


    def connect(self) -> bool:
        try:
            self.serial = Serial(self.port, timeout=0, write_timeout=0.5)
            self.enable_all()
        except SerialException as e:
            self.get_logger().info(f"Could not connect to esp32_thruster: {e}")
            return False
        return True

    def write(self, data: str) -> bool:
        data = data + "\n"
        if self.serial is None:
            success = self.connect()
            if not success or self.serial is None:
                self.get_logger().error("Serial is not connected. AHHHH!")
                return False
        try:
            self.serial.write(data.encode())
            return True
        except SerialException as e:
            self.get_logger().info(f"write error: {e}")
            self.serial.close()
            self.connect()
        return False


    # percent_raw should be in [-1, 1]
    # percent_val should be in [-100, 100]
    def convert_percent(self, percent_raw: float) -> Optional[int]:
        if percent_raw < -1 or percent_raw > 1:
            self.get_logger().info(
                f"ESP32 [ERROR]: Raw input value {percent_raw} out of range (should be in [-1, 1])"
            )
            return None
        return int(percent_raw * 100)

    # in case of invalid percent or pin number parameters, does not send any updated signal to the pin controller
    def send_power(self, pin: int, percent_raw: float) -> int:
        percent_val = self.convert_percent(percent_raw)
        if percent_val is None:
            return False

        if pin < 0 or pin >= NUM_PINS:
            self.get_logger().info(
                f"ERROR: pin number {pin} out of range (should be in [0, {NUM_PINS-1}])"
            )
            return False

        msg = "POWER:" + str(pin) + "," + str(percent_val)

        self.write(msg)

        return True


    def send_enable_disable(self, pin: int, enable: bool) -> int:
        self.get_logger().info(f"Send enable {enable} to pin {pin}")
        if(enable):
            msg = "ENABLE:TRUE"
        else:
            msg = "ENABLE:FALSE"

        if pin < 0 or pin >= NUM_PINS:
            self.get_logger().info(
                f"ERROR: pin number {pin} out of range (should be in [0, {NUM_PINS-1}])"
            )
            return False

        self.write(msg)
        return True

    def enable_all(self):
        for i in range(NUM_PINS):
            self.send_enable_disable(i, True)


    def send_estop(self):
        msg = "ESTOP:"
        self.write(msg)


    def handle_emergency_stop(
        self, req: SetBool.Request, res: SetBool.Response
    ) -> SetBool.Response:
        if req.data:
            self.send_estop()
        res.success = True
        return res

    def eps32_thruster_commands_callback(self, msg: ThrusterCommands):
        for i in range(NUM_PINS):
            if msg.valid[i]:
                self.thruster_outputs[i] = msg.pins[i]
            if msg.pins[i] != 0:
                self.get_logger().info(f"The motor at index {i} is running at value {msg.pins[i]}")

    def loop(self):
        for i in range(NUM_PINS):
            self.send_power(i, self.thruster_outputs[i])

        if self.serial is not None:
            data = self.serial.read_all()
            if data is not None:
                pass
                # self.get_logger().info(f"data read from esp32: {str(data)}")


def main():
    rclpy.init()
    node = ESP32_Thruster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
