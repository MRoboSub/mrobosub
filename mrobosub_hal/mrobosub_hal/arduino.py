import rclpy
from serial import Serial
from std_msgs.msg import Float32, Bool
from sensor_msgs.msg import FluidPressure
import time
import struct

from mrobosub_lib import Node
from std_srvs.srv import SetBool

FREQUENCY = 60 # times per second
BAUD_RATE = 9600
CONNECTION_NAME = "/dev/ttyACM1"

class Arduino(Node):
    def __init__(self):
        super().__init__("arduino")

        self.charm_pub = self.create_publisher(
            Bool, "/buttons/charm", qos_profile=1
        )
        self.strange_pub = self.create_publisher(
            Bool, "/buttons/strange", qos_profile=1
        )
        self.depth_pub = self.create_publisher(
            FluidPressure, "/depth", qos_profile=1
        )
        self.zero_srv = self.create_service(
            SetBool, "/depth/zero", self.zero_srv_callback
        )

        #self.write_timer = self.create_timer(1/FREQUENCY, self.writer)
        self.read_timer = self.create_timer(1/FREQUENCY, self.serialConnection)
        
        self.serial = Serial(CONNECTION_NAME, BAUD_RATE, timeout=1)

        time.sleep(2) # Give some time for the serial connection to be made.

        self.numHeaderBytes = 0
        self.charm = 0
        self.strange = 0
        self.depth = 0.0
        self.offset = 0.0
        self.dataBytes = []

        self.zero = False
        
    
    def serialConnection(self):
        if self.serial.in_waiting < 1:
            return None
        
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
                self.strange = unpacked_data[0]
                self.depth = unpacked_data[1]
                self.charm = unpacked_data[2]

                self.numHeaderBytes = 0
                self.dataBytes = []

                self.charm_pub.publish(Bool(data=(self.charm == b'\x01')))
                self.strange_pub.publish(Bool(data=(self.strange == b'\x01')))
                
                offsetted_pressure = self.depth - self.offset
                self.depth_pub.publish(FluidPressure(fluid_pressure=offsetted_pressure))

    def zero_srv_callback(self, req, res):
        self.zero = req.data
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
