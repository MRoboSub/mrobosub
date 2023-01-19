#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

from mrobosub_lib.lib import Node, Param

from typing import Optional, Final

class DepthTeleop(Node):
    """
    Subscribers
    - /depth/raw_depth
    """

    """
    Publishers
    - /pose/heave
    """

    def __init__(self):
        super().__init__('depth_teleop')
        
        self.twist_pub = rospy.Publisher('/target_twist/heave', Float64, queue_size=1)
        self.pose_pub = rospy.Publisher('/target_pose/heave', Float64, queue_size=1)

    def run(self): 
        while True:
            print("Which input would you like: \nTwist \nPose\n")
            input_node = input().upper()
            
            curr_node = None
            if input_node[0] == "T":
                curr_node = self.twist_pub
            elif input_node[0] == "P":
                curr_node = self.pose_pub
            else:
                print("Invalid input node\n")
                continue
                    
            while True:     
                print("Input your desired value (or change to change node): ")
                desired_target = input()
                
                try:
                    desired_target = float(desired_target)
                    
                    curr_node.publish(desired_target)
                except:
                    break
        
    def cleanup(self):
        self.twist_pub.publish(0)

if __name__ == '__main__':
    DepthTeleop().run()
