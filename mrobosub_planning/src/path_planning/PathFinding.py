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

    def h_cost(self, current, goal):
        h_cost = 0.0;
        hor_cost = 1;
        dia_cost = np.sqrt(2);

        dx = abs(current.x - goal.x);
        dy = abs(current.y - goal.y);
        h_cost = hor_cost * (dx + dy) + (dia_cost - 2 * hor_cost) * min(dx, dy);
        
        return h_cost
    
    #simple g cost for now. 
    #REVIEW - do we need g_cost? hueristic alone should be fine
    #TODO - complete gcost
    def g_cost(self, current, start, distances, params):
        nearestObstDist = distances(current.x, current.y) #TODO - implement distances
        hor_cost = 1;
        dia_cost = np.sqrt(2);
        g_cost = current.g_cost;
        dx = abs(current.x - start.x);
        dy = abs(current.y - start.y);

        if (dx == 1 and dy == 1):
            g_cost += dia_cost;
        else:
            g_cost += hor_cost;

        
        return abs(current.x - start.x) + abs(current.y - start.y)
    
    def getNeighbors(self, current, distanceGrid):
        xDeltas = [-1, 0, 1, -1, 1, -1, 0, 1]
        yDeltas = [0, 0, 1, -1, 1, -1, -1, 1]
        neighbors = []
        for i in range(8):
            x = current.x + xDeltas[i]
            y = current.y + yDeltas[i]
            if distanceGrid.isCellInGrid(x, y):
                neighbors.append(distanceGrid.cells_[x, y])

        #TODO - implement getNeighbors
        return neighbors
    
    #TODO - complete extract_node_path
    def extract_node_path(self, goalNode, startNode):
        path = []
        path.append(goalNode)
        while goalNode != startNode:
            goalNode = goalNode.getParent()
            path.append(goalNode)
        
        path.reverse()
        return path
    
    #TODO - complete extract_pose_path
    def extract_pose_path(self, path, distanceGrid):
        new_path = []
        i = 0
        for node in path:
            delta_y, delta_x, theta = 0, 0, 0
            cur = grid_position_to_global_position(node.x, node.y, distanceGrid)

            if (i == 0):
                delta_y = cur.y
                delta_x = cur.x
                i += 1
            
            else:
                delta_y = cur.y - new_path[i-1].y
                delta_x = cur.x - new_path[i-1].x
            
            theta = np.arctan2(delta_y, delta_x)
            #TODO - instead of pushing nodes, I should make a separate 
            # pose class and keep them separate. || or, I could make the node contain both the pose and the cell
            new_path.append(nd.Node([cur.x, cur.y, theta])) # add new node to list using pose
            
        return path
    
    # REVIEW - this may lead to a slow path, so maybe make an alternative pruning method
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