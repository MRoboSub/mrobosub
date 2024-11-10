import socket

import rospy
from mrobosub_msgs.msg import Dvl


UDP_IP = ...
UDP_PORT = ...


class DVLPublisher():
    def __init__(self):
        self.pub = rospy.Publisher('/dvl/raw_dvl', Dvl, queue_size=10)

    def publisher(self):
        rate = rospy.Rate(50)

        # connect to socket containing the DVL information
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((UDP_IP, UDP_PORT))
 
        while not rospy.is_shutdown():
            data, addr = sock.recvfrom(1024)
            data_str = data.decode()

            if not data_str.startswith("$DVKFC"):
                rate.sleep()
                continue

            # parse according to spec here https://docs.ceruleansonar.com/c/dvl-50/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvkfc-kalman-filter-raw-data-support-message
            data_list = data_str.split(',')
            self.pub.publish(Dvl(*map(lambda i: float(data_list[i]), (10, 17, 24))))
            rate.sleep()