#!/usr/bin/env python

import inertial_sense_ros.msg as imu_msgs
import rospy

from mrobosub_lib.lib import Node
from mrobosub_msgs.msg import Imu_INS, Imu_PIMU

class IMU(Node):
    def __init__(self):
        super().__init__('imu')
        print("Launched imu node")
        self.did_ins_sub = rospy.Subscriber("/ins_eul_uvw_ned", imu_msgs.DID_INS1, self.did_ins_callback) # for DID_INS1
        self.pimu_sub = rospy.Subscriber("/pimu", imu_msgs.PIMU, self.did_pimu_sub) # for DID_PIMU
        self.did_ins_pub = rospy.Publisher("/imu_INS", Imu_INS, queue_size=1)
        self.pimu_pub = rospy.Publisher("/imu_PIMU", Imu_PIMU, queue_size=1)

    def did_ins_callback(self, msg):
        self.did_ins_pub.publish(msg.theta)


    def did_pimu_sub(self, msg):

        m = Imu_PIMU()
        m.dtheta = msg.dtheta
        m.dvel = msg.dvel
        m.dt = msg.dt
        
        self.pimu_pub.publish(m)

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    IMU().run()

