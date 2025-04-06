import array
import math
from Nodes import Node
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
        #  Initialize with the x-pose and the y-pose from the ekf.
        rospy.Subscriber('/pose/x_pos', Float64, self.pose_callback)
        rospy.Subscriber('/pose/y_pos', Float64, self.pose_callback)
        # TODO This is where we will make the grid
    
    def pose_callback(self, msg):
        self.sub_pos = [msg.x_pos, msg.y_pos]

    #find path using basic heuristic  
    def findPath(self, startNode: Node, goalNode: Node, OccupancyGrid: OccupancyGrid) -> "list[Node]":

        # start by creating a new empty path, open list, and closed list. and setting path_found flag to false.
        path = []
        open_list = []
        closed_list = []

        # set path_found to false, nextNode to none, and push the start node to the openlist.
        # Open list should be a priority queue, with nodes with the least cost being at the top.
        path_found = False
        nextNode = startNode
        hq.heappush(open_list, startNode)
        

        
        # While the open list has nodes inside of it, we want to iterate through nodes and add them to the closed list, starting with the startNode.
        # If the next node, which is popped from the top of the open list, is the goal node, we have found the path and we may break out of this loop after setting
        # the path_found flag to true.
        # If the next node is not the goal node, we then find its "child nodes," which can be seen in the getNeighbors function below. For each child node, we then
        # Check if they are already in either of the lists. If they are already in one of the lists, and their current cost is lower than the cost already associated
        # with the node, the cost should be updated and the parent should be changed to "nextNode". If undiscovered, compute the cost and add it to the open list to be 
        # explored later. This while loop will run until the open list is empty or the goal_node is found.
        while len(open_list) > 0 and path_found == False:
            nextNode = open_list.pop()
            closed_list.append(nextNode)

            if nextNode == goalNode:
                goalNode.setParent(nextNode.getParent())
                path_found = True
                break
            
            childNodes = self.getNeighbors(nextNode, OccupancyGrid)
            for child in childNodes:
                if child != None:
                    #check if child is in closed list or open list
                    if child not in closed_list and child not in open_list:
                        #undiscovered node, compute new cost and add to open list
                        child.setCost(self.h_cost(child, goalNode))
                        hq.heappush(open_list, child)
                   
                    #if child is in one of the lists, check if the new cost is less than the current cost and update
                    else:
                        #discovered node, check if the cost is less than the current cost
                        new_cost = self.h_cost(child, goalNode)
                        if new_cost < child.getCost():
                            child.setCost(self.h_cost(child, goalNode))
                            child.setParent(nextNode)


        # If the path is found, we now wll extract the path, which can be seen in detail below. We will then prune the path,
        # deleting any unnecesary nodes and increasing the possible speed. Finally, we will extract the pose path.
        if(path_found == True):
            # extract the path from the goal node to the start node using the parents
            nodePath = self.extract_node_path(goalNode, startNode)
            # prune the path to remove unnecessary nodes and increase speed
            prunedNodePath = self.prune_node_path(nodePath)
            # convert the node path to a pose path, which can then be used with PIDs 
            path = self.extract_pose_path(prunedNodePath, OccupancyGrid)
    
        else:
            print("A* Didn't find a path yet")
            return []

        return path

    
    def h_cost(self, current, goal):
        # This is the function that calculates the cost of each node. The costs in our case can be determined by using the equation
        # cost = H x (dX+dY) + (D - 2 * H) x min(dx,dy), where:
        # H is a horizontal cost constant, D is the diagnal cost constant, and dx,dy are the errors in the x and y direction. In this instance
        # Because we want the cost to coorelate to the distance from the target, we set our costs to be 1 meter per cell in the horizontal direction, 
        # and the diagnal cost to be sqrt(2), which is found using pythagoreans theorem.
        h_cost = 0.0;
        hor_cost = 1;
        dia_cost = 2**0.5;

        dx = abs(current.x - goal.x);
        dy = abs(current.y - goal.y);
        h_cost = hor_cost * (dx + dy) + (dia_cost - 2 * hor_cost) * min(dx, dy);
        
        return h_cost
    
 
    def getNeighbors(self, current, distanceGrid):
        # This function finds and stores all of the nodes surrounding the current node in an array. It does this by taking the "detlas" around the node,
        # or the indices of each node on the map around the current node. One way of doing this is by setting x,y delta pair, which you can add to the 
        # index of the current node. Once the index of a child node is found, we add the node to the array if it is confirmed that the node lies within the map.
        # We then also set the parent of the child node to be the current node.        
        xDeltas = [-1, 1, -1, 0, 1, -1, 0, 1]
        yDeltas = [0, 0, -1, -1, -1, 1, 1, 1]
        neighbors = []
        for i in range(8):
            x = current.x + xDeltas[i]
            y = current.y + yDeltas[i]
            if distanceGrid.isCellInGrid(x, y):
                child = Node([x, y])
                child.setParent(current)
                neighbors.append(child)
        return neighbors
    
    #TODO - complete extract_node_path
    def extract_node_path(self, goalNode, startNode):
        # This function finds the actual path and puts it in order from start to finish. In this function, you start at the goal node 
        # and find the path by getting the parent of the goal node, then the parent of the parent of the goal node, etc until the startNode is found.
        # Add all of the goal nodes to the path array as you go, and reverse the order of the array to ensure the final path is from the start to the end.

        path = []
        path.append(goalNode)
        while goalNode != startNode:
            goalNode = goalNode.getParent()
            path.append(goalNode)
        
        path.reverse()
        return path
    
    #TODO - complete extract_pose_path
    def extract_pose_path(self, path, distanceGrid) -> list:
        # The purpose of this function is to convert from the node path to pose path, such that the paths are put in the real world from
        # using x and y distance coordinates from a global start point. This coordinate frame is more useful for us because we can use the
        # coordinates to find target directions and distances for PID control.
         
        new_path = []
        i = 0
        for node in path:
            delta_y, delta_x, theta = 0, 0, 0
            current = grid_position_to_global_position(node.x, node.y, distanceGrid)

            if (i == 0):
                delta_y = current[1] #y
                delta_x = current[0] #x
                i += 1
            
            else:
                delta_y = current[1] - new_path[i-1].y
                delta_x = current[0] - new_path[i-1].x
            
            theta = math.atan2(delta_y, delta_x)
            #TODO - instead of pushing nodes, I should make a separate 
            # pose class and keep them separate. || or, I could make the node contain both the pose and the cell
            new_path.append(Node([current[0], current[1], theta])) # add new node to list using pose
            
        return new_path
    
    # ! - this may lead to a slow path, refine this method to remove more nodes.
    def prune_node_path(self, path):
        new_path = []
        new_path.append(path[0])
        for i in range(0, len(path) - 2):
            slope2, slope1, dx1, dx1, dx2, dy2 = 0, 0, 0, 0, 0, 0
            while path[i+2] != path[path.length]:
                dx2 = np.abs(path[i].x - path[i+2].x)
                dy2 = np.abs(path[i].y - path[i+2].y)
                dx1 = np.abs(path[i].x - path[i+1].x)
                dy1 = np.abs(path[i].y - path[i+1].y)
                slope2 = dy2 / dx2
                slope1 = dy1 / dx1

                if slope1 != slope2:
                    new_path.append(path[i+1])
                    break
                elif path[i+2] == path[path.length]:
                    new_path.append(path[i+2])
                    break

def grid_position_to_global_position(x, y, distanceGrid):
    global_x = distanceGrid.globalOrigin_[0] + x / distanceGrid.cellsPerMeter_
    global_y = distanceGrid.globalOrigin_[1] + y / distanceGrid.cellsPerMeter_
    return [global_x, global_y]

def global_position_to_grid_position(x, y, distanceGrid):
    grid_x = (x - distanceGrid.globalOrigin_[0]) * distanceGrid.cellsPerMeter_
    grid_y = (y - distanceGrid.globalOrigin_[1]) * distanceGrid.cellsPerMeter_
    return [grid_x, grid_y]