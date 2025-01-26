import Nodes as nd
import numpy as np
import heapq as hq
import rospy
from std_msgs.msg import Float64, Float32
from OccupancyGrid import OccupancyGrid


class PathFinding:
    """
    Subscribers
    - /pose/x_pos
    - /pose/y_pos

    Publishers
    - /path
    """
    sub_pos = None
    def __init__(self):
        rospy.Subscriber('/pose/x_pos', Float64, self.pose_callback)
        rospy.Subscriber('/pose/y_pos', Float64, self.pose_callback)
    
    def pose_callback(self, msg):
        self.sub_pos = [msg.x_pos, msg.y_pos]

    #find path using basic A* algorithm    
    def findPath(self, startNode, goalNode, distanceGrid, searchParams):
        path = {}
        path_found = False
        open_list = []
        #TODO - configure the heap to use the lowest f_cost first
        hq.heappush(open_list, startNode)

        closed_list = []
        nextNode = None;

        while not open_list == False:
            nextNode = open_list.pop()
            closed_list.append(nextNode)

            if nextNode == goalNode:
                goalNode.setParent(nextNode.getParent())
                path_found = True
                break
            
            #TODO - implement getNeighbors
            childNodes = nextNode.getNeighbors()
            for child in childNodes:
                if child != None:
                    #check if child is in closed list or open list
                    if child not in closed_list and child not in open_list:
                        #undiscovered node, compute new cost and add to open list
                        #TODO - put setParent into the expand node function
                        child.setCost(self.g_cost(child, startNode), self.h_cost(child, goalNode))
                        hq.heappush(open_list, child)
                   
                    #if child is in one of the lists, check if the cost is less than the current cost and update
                    else:
                        #discovered node, check if the cost is less than the current cost
                        new_cost = self.g_cost(child, startNode) + self.h_cost(child, goalNode)
                        if new_cost > child.getCost():
                            child.setCost(self.g_cost(child, startNode), self.h_cost(child, goalNode))
                            child.setParent(nextNode)

        print("path has been found")

        if(path_found == True):
            nodePath = self.extract_node_path(goalNode, startNode)
            prunedNodePath = self.prune_node_path(nodePath)
            path = self.extract_pose_path(prunedNodePath, distanceGrid)
    
        else:
            print("A* Didn't find a path yet")
            return 0

        return path

    #simple heuristic for now
    def h_cost(self, current, goal):
        return abs(current.x - goal.x) + abs(current.y - goal.y)
    
    #simple g cost for now
    #TODO - complete gcost
    def g_cost(self, current, start):
        return abs(current.x - start.x) + abs(current.y - start.y)
    
    #TODO - complete extract_node_path
    def extract_node_path(self, goalNode, startNode):
        path = {}
        return path
    
    #TODO - complete prune_node_path
    def prune_node_path(self, path):
        return path
    
    #TODO - complete extract_pose_path
    def extract_pose_path(self, path, distanceGrid):
        return path