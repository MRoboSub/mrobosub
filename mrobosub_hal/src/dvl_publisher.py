#!/usr/bin/env python

import socket

import rospy
from mrobosub_msgs.msg import Dvl
import numpy as np


# todo: parameterize this in the launch file
UDP_IP = "0.0.0.0"
# UDP_IP = "192.168.2.9"
UDP_PORT = 50000


class DVLPublisher():
    def __init__(self):
        rospy.init_node("dvl_publisher")
        self.pub_raw = rospy.Publisher('/dvl/raw_dvl', Dvl, queue_size=1)
        self.pub_translational = rospy.Publisher('dvl/translational_data', Dvl, queue_size=1) #TODO edit DVL type to be translational
        # cone unit vectors
        self.coneA = np.array([np.cos(70*np.pi/180), 0, np.sin(70*np.pi/180)])
        self.coneB = np.array([-np.cos(70*np.pi/180)*np.sin(30*np.pi/180), np.cos(70*np.pi/180)*np.cos(30*np.pi/180), np.sin(70*np.pi/180)])
        self.coneC = np.array([-np.cos(70*np.pi/180)*np.sin(30*np.pi/180), -np.cos(70*np.pi/180)*np.cos(30*np.pi/180), np.sin(70*np.pi/180)])

    def run(self):
        rate = rospy.Rate(50)

        # connect to socket containing the DVL information
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setblocking(False)
        sock.settimeout(0.2)
        sock.bind((UDP_IP, UDP_PORT))
 
        while not rospy.is_shutdown():
            try:
                data, addr = sock.recvfrom(1024)
                data_str = data.decode()

                if not data_str.startswith("$DVKFC"):
                    rate.sleep()
                    continue

                # parse according to spec here https://docs.ceruleansonar.com/c/dvl-50/communicating-with-the-tracker-650/outgoing-message-formats-tracker-650-to-host/usddvkfc-kalman-filter-raw-data-support-message
                data_list = data_str.split(',')
                self.pub_raw.publish(Dvl(*map(float, data_list[10:24+1:7])))

                # transform from cone axis to tranlational axis 
                cone_A_vel = float(data_list[10])
                cone_B_vel = float(data_list[17])
                cone_C_vel = float(data_list[24])
                velocities = cone_A_vel * self.coneA + cone_B_vel * self.coneB + cone_C_vel * self.coneC
                trans_response = Dvl()
                trans_response.velocityA = float(velocities[0]) #along surge axis
                trans_response.velocityB = float(velocities[1]) #along sway axis
                trans_response.velocityC = float(velocities[2]) #along heave axis (positive down)
                self.pub_translational.publish(trans_response)
                rate.sleep()
            except socket.timeout:
                print("DVL UDP connection timing out, no data recieved from DVL")
        sock.close()

if __name__ == "__main__":
    DVLPublisher().run()
