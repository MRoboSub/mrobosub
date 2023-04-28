#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64, Bool
from mrobosub_lib.lib import Node, Param
from dynamic_reconfigure.server import Server
from mrobosub_localization.cfg import collision_paramsConfig
from math import exp

class CollisionDetector(Node):
    threshold: float
    rc: float

    def __init__(self):
        super().__init__("collision")
        self.sub = rospy.Subscriber('/imu/data', Imu, self.filter_callback)
        self.pub_filtered = rospy.Publisher('/collision/filtered_accel', Float64, queue_size = 10)
        self.pub_collision = rospy.Publisher('/collision/collision', Bool, queue_size = 10)
        self.last_value = 0
        self.last_time = 0

    def filter_callback(self, msg):
        if self.last_time == 0:
            self.pub_filtered.publish(msg.linear_acceleration.x)
            self.last_value = msg.linear_acceleration.x
            self.last_time = msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9
            return
        next_time = (msg.header.stamp.secs + msg.header.stamp.nsecs / 1e9)
        delta_time = next_time - self.last_time
        
        a = exp(-delta_time / self.rc)

        output = (self.last_value - msg.linear_acceleration.x) * a + msg.linear_acceleration.x
        self.pub_filtered.publish(output)
        self.pub_collision.publish(-output > self.threshold)
        self.last_time = next_time
        self.last_value = output

    def reconfigure_callback(self, config, level):
        self.rc = config["rc"]
        self.threshold = config["threshold"] 
        return config
    
    def run(self):
        srv = Server(collision_paramsConfig, self.reconfigure_callback)
        rospy.spin()

if __name__ == "__main__":
    CollisionDetector().run()
