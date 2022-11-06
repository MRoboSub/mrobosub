#!/usr/bin/env python

import rospy

def perception_init():
    rospy.init.node('perception_node',anonymous=False)
    
    # Subscribe to the client/Topic. Need to figure out client. Once decided need to edit this. 
    # Publish to the topics that communicate with other perception nodes. 

    rospy.spin()

if __name__ == '__main__':
    perception_init()
    

