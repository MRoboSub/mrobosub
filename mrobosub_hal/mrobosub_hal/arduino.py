import rclpy
from serial import Serial
from std_msgs.msg import Float32, Bool
from mrobosub_msgs.msg import LedState
from sensor_msgs.msg import FluidPressure
import time
import struct

from mrobosub_lib import Node
from std_srvs.srv import SetBool
import sys

FREQUENCY = 120 # times per second
BAUD_RATE = 9600

class Arduino(Node):
    def __init__(self):
        super().__init__("arduino")

        if len(sys.argv) < 2:
            raise ValueError("Must provide port")

        self.charm_pub = self.create_publisher(
            Bool, "/buttons/charm", qos_profile=1
        )
        self.strange_pub = self.create_publisher(
            Bool, "/buttons/strange", qos_profile=1
        )
        self.depth_pub = self.create_publisher(
            Float32, "/depth", qos_profile=1
        )
        self.zero_srv = self.create_service(
            SetBool, "/depth/zero", self.zero_srv_callback
        )

        self.charm_state_sub = self.create_subscription(
            LedState,
            f"/leds",
            self.led_state_callback,
            qos_profile=1,
        )
   
        # Create serial connection
        self.serial = Serial(sys.argv[1], BAUD_RATE, timeout=0)

        time.sleep(2) # Give some time for the serial connection to be made.

        if self.serial is None:
            self.get_logger().error("Serial is none")
            exit()

        # In case there is data pending in the Serial buffer that is OLD, clear this out.
        self.serial.reset_input_buffer()

        # Init values
        self.numHeaderBytes = 0
        self.offset = 0.0
        self.dataBytes = []

        # start timer
        self.read_timer = self.create_timer(1/FREQUENCY, self.serialLoop)
        
    
    def serialLoop(self):
        self.handleRead()
        # self.handleWrite()

    
    def handleRead(self):
        if self.serial.in_waiting < 1:
            return None

        # printing in the if statement should be paired with one out of the if statement.
        
        bytes_recd = self.serial.read(1)

        if self.numHeaderBytes < 4:
            if bytes_recd == b'\xff':
                self.numHeaderBytes += 1
            else:
                self.numHeaderBytes = 0
        else:
            self.dataBytes.append(bytes_recd)
            if len(self.dataBytes) == 6:
                ard_data = b''.join(self.dataBytes)

                unpacked_data = struct.unpack('<cfc', ard_data)
                strange = unpacked_data[0]
                depth = unpacked_data[1]
                charm = unpacked_data[2]

                self.numHeaderBytes = 0
                self.dataBytes = []

                self.charm_pub.publish(Bool(data=(charm == b'\x01')))
                self.strange_pub.publish(Bool(data=(strange == b'\x01')))
                
                offsetted_pressure = depth - self.offset
                self.depth_pub.publish(Float32(data=offsetted_pressure))


    # def handleWrite(self):
    #     self.serial.write(b'\xFF')
    #     self.serial.write(b'\x01' if self.charm_state else '\x00')
    #     self.serial.write(b'\x01' if self.strange_state else '\x00')

    def led_state_callback(self, msg: LedState):
        self.charm_state = msg.charm_state
        self.strange_state = msg.strange_state

    def zero_srv_callback(self, req, res):
        self.offset = self.depth 
        res.success = True       
        return res

def main():
    rclpy.init()
    node = Arduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
