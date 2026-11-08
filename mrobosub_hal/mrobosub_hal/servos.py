import rclpy
from serial import Serial
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import FluidPressure
import time
import math

from mrobosub_lib import Node
from std_srvs.srv import SetBool

BAUD_RATE = 9600
CONNECTION_NAME = "/dev/ttyACM1"

class Servo(Node):
    def __init__(self):
        super().__init__("servo")

       

        #self.write_timer = self.create_timer(1/FREQUENCY, self.writer)
        #self.read_timer = self.create_timer(1/FREQUENCY, self.serialConnection)
        
        self.serial = Serial(CONNECTION_NAME, BAUD_RATE, timeout=0.1)

        time.sleep(2) # Give some time for the serial connection to be made.

        self.send_command("ESTOP:ON")

        self.angle_sub = self.create_subscription(
            Float32,
            "/servo/angle",
            self.angle_callback,
            1
        )

        self.enable_sub = self.create_subscription(
                    Bool,
                    "/servo/enable",
                    self.angle_callback,
                    1
        )

        self.estop_sub = self.create_subscription(
                    Bool,
                    "/servo/estop",
                    self.angle_callback,
                    1
        )

        self.get_logger().info("Servo node initialized!")
        
    
    def send_command(self, command):
        message = command + "\n"
        self.serial.write(message.encode("utf-8"))
        self.serial.flush()

        self.get_logger().info(f"Sent: {command}")

    # Receive target angle
    def angle_callback(self, msg):

        angle = msg.data

        if angle < 30 or angle > 160:
            self.get_logger().warn(
                f"Angle {angle} outside valid range (0-180)"
            )
            return

        command = f"SET: {int(round(angle))}"
        self.send_command(command)

    
    def estop_callback(self, msg):

        if msg.data:
            self.send_command("ESTOP:ON")
            self.get_logger().warn("SERVO EMERGENCY STOP ACTIVATED")

        else:
            self.send_command("ESTOP:OFF")
            self.get_logger().info("Servo emergency stop cleared")

    # Enable / disable servo
    def enable_callback(self, msg):

        if msg.data:
            self.send_command("ENABLE:TRUE")

        else:
            self.send_command("ENABLE:FALSE")


    def destroy_node(self):

        # Close serial connection when ROS node shuts down
        if self.serial.is_open:
            self.serial.close()

        super().destroy_node()


def main():

    rclpy.init()

    node = Servo()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()