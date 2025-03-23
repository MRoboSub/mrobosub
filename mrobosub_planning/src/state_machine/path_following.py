from ..path_planning.PathFinding import PathFinding
from ..path_planning.Nodes import Node, Params
from ..path_planning.OccupancyGrid import OccupancyGrid
from periodic_io import PIO
import math
import rospy
from std_msgs.msg import Float64

# Go to nodes produced by path finder
    # timeout: float = 150.0
    # surge_speed: float = 0.15
    # found_image_threshold = 50

# This was found in gate_states line 45. use later

# TODO - make start node, end node, distance grid, and search parameters

class PathFollowing:
    target_heave = 2;
    def __init__(self):
    # TODO check that we can do this outside of a class
        rospy.Subscriber('/pose/x_pos', Float64, self.pose_callback)
        rospy.Subscriber('/pose/y_pos', Float64, self.pose_callback)
        self.sub_pose = None
        self.startNode = None
        self.goalNode = None

    def pose_callback(self, msg):
        self.sub_pos = [msg.x_pos, msg.y_pos]
        
    def setStartNode(self):
        self.startNode = Node(self.sub_pose)

    def setGoalNode(self, x, y):
        self.goalNode = [x,y]

    def setOccupancyGrid(self, width, height, metersPerCell, CellsPerMeter, OccupiedThreshold):
        self.occupancyGrid = OccupancyGrid(width, height, metersPerCell, CellsPerMeter, OccupiedThreshold)
    
    def setTargetYaw(self, current, next):
        deltax = next.x - current.x
        deltay = next.y - current.y
        self.target_yaw = math.atan2(deltay, deltax)

    def setTargetSurge(self, current, next):
        deltax = next.x - current.x
        deltay = next.y - current.y
        self.target_surge = ((deltax * deltax) + (deltay * deltay))**(1/2)

follower = PathFollowing()
follower.setStartNode
follower.setGoalNode(10,10)

# Set the Occupancy Grid Here
# TODO How to make from square to circular
follower.setOccupancyGrid(10,10,10,010,01)

# make an instance of the path finder
pathfinder = PathFinding() 


path = pathfinder.findPath(follower.startNode, follower.goalNode, follower.occupancyGrid)

# TODO make the PID go from one node to the next.
PIO.set_target_pose_heave(follower.target_heave)
PIO.set_target_pose_yaw(follower.target_yaw) #use atan2
PIO.set_target_twist_surge(follower.target_surge)
