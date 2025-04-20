from ast import Global
from time import sleep
from PathFinding import PathFinding
from Nodes import Node, Params
from OccupancyGrid import OccupancyGrid
from std_msgs.msg import Float64
from ..state_machine.periodic_io import PIO 

import math
import rospy

class PathFollowing(PathFinding):
    target_heave = 2;   

    def goToTarget(self, node:Node):
        # RTR (rotate translate rotate) version of motion control
        # 1. rotate
        while not self.turned():
            self.setCurrentNode();
            self.setTargetYaw(node, self.currentNode)
            PIO.set_target_pose_yaw(self.target_yaw)
        
        # 2. translate
        while not self.arrivedAtGoal(node):
            # Note, the arrivedAtGoal() currently uses the x,y coordinates to determine whether the sub has arrived or not
            # However, this does not interact directly with the surge, which is a separate entity in this system.
            # TODO, determine how to directly tie the surge and the x,y coordinates.
            self.setCurrentNode()
            self.setTargetSurge(node, self.currentNode)
            PIO.set_target_pose_surge(self.target_surge)


    def arrivedAtGoal(self, goalNode:Node) -> bool:
        return (goalNode.x - self.currentNode.x) < 0.2 & (goalNode.y - self.currentNode.y) < 0.2
    
    def turned(self) -> bool:
        return (self.target_yaw  - self.sub_yaw) < 0.2