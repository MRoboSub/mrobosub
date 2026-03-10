import socket
import rclpy
from mrobosub_msgs.msg import Dvl
import numpy as np
from mrobosub_lib import Node


# todo: parameterize this in the launch file
UDP_IP = "0.0.0.0"
# UDP_IP = "192.168.2.9"
HOST_IP = b"192.168.2.3"
UDP_PORT = 27000


class DVLPublisher (Node):
    """
    Provides /dvl/raw_dvl topic
    """

    def __init__(self):
        super().__init__('dvl_publisher')
        self.pub = self.create_publisher(Dvl, "/dvl/raw_dvl", qos_profile=1)

        # connect to socket containing the DVL information
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setblocking(True)  # False
        self.sock.settimeout(0.2)
        self.sock.bind((UDP_IP, UDP_PORT))
        self.timer = self.create_timer(1.0/50, self.loop)

        # set the host address and port on the dvl
        message = b"HOST-ADDRESS %b:%b" % (HOST_IP, str(UDP_PORT).encode('utf-8'))
        self.sock.sendto(message, (UDP_IP, UDP_PORT))

    def destroy_node(self):
        self.sock.close()
        super().destroy_node()

    def loop(self):
        try:
            data, addr = self.sock.recvfrom(1024)
            data_str = data.decode()

            if not data_str.startswith("$DVKFC"):
                return

            # parse according to spec here https://docs.ceruleansonar.com/c/dvl-50/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvkfc-kalman-filter-raw-data-support-message
            data_list = data_str.split(",")
            map_list = [*map(float, data_list[10 : 24 + 1 : 7])]
            self.pub.publish(Dvl(velocity=map_list))

        except socket.timeout:
            self.get_logger().info("DVL UDP connection timing out, no data recieved from DVL")

def main():
    rclpy.init()
    node = DVLPublisher()
    rclpy.spin(node)

if __name__ == "__main__":
    main()
