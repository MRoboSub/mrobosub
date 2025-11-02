import rclpy
from std_msgs.msg import Float32, Bool
from serial import Serial
from struct import Struct
import time


from mrobosub_lib import Node
from mrobosub_msgs.msg import ImuINS, ImuPIMU

FREQUENCY = 60 # times per second
BAUD_RATE = 9600
CONNECTION_NAME = "/dev/ttyACM0"
HALL_EFFECT_ON = 0 # By default (with no magnet) the hall effect is 1. So it is on when it is 0.
PADDING_BYTES = 4
BUFFER_SIZE   = 6

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
            Float32, "/depth", qos_profile=1
        )

        self.timer = self.create_timer(1/FREQUENCY, self.timer_callback)
        
        self.serial = Serial(CONNECTION_NAME, BAUD_RATE, timeout=1)

        time.sleep(2) # Give some time for the serial connection to be made.


    def timer_callback(self):
        data_to_publish = self.read_aligned_bytes()
        if data_to_publish is not None:
            (charm, strange, depth) = data_to_publish
            self.charm_pub.publish(Bool(data=charm))
            self.strange_pub.publish(Bool(data=strange))
            self.depth_pub.publish(Float32(data=depth))
    

    def read_aligned_bytes(self) -> tuple[bool, bool, float] | None:
        # We need some bytes to grab.
        if self.serial.in_waiting < 1:
            return None
        
        # Read all bytes available. 
        byte_recd = self.serial.read(self.serial.in_waiting)

        # We need to count PADDING_BYTES 0xFF at the beginning to align to the data.
        ff_bytes_count = 0 
        buffer = []
        for b in [int(b) for b in list(byte_recd)]:
            if ff_bytes_count < PADDING_BYTES:
                if b == 0xFF:
                    ff_bytes_count += 1
                else:
                    ff_bytes_count = 0
            else:
                buffer.append(b)
                
            if len(buffer) == BUFFER_SIZE:
                return self.bytes_to_data(buffer)
        
        return None
        

    def bytes_to_data(self, buffer) -> tuple[bool, bool, float]:
        self.get_logger().info(f"buffer: {buffer}")
        bytes_object = bytes(buffer)
        buffer_format = Struct("<ccf") # two single chars for the hall effects then a float  
        (charm, strange, depth) = buffer_format.unpack(bytes_object)
        self.get_logger().info(f"{charm=} {strange=} {depth=}")
        return (int.from_bytes(charm, byteorder='little') == HALL_EFFECT_ON, int.from_bytes(strange, byteorder='little') == HALL_EFFECT_ON, depth) 


def main():
    rclpy.init()
    node = Arduino()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == "__main__":
    main()
