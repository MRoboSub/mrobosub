from time import sleep
from ..path_planning.PathFinding import PathFinding
from ..path_planning.PathFollowing import PathFollowing
from periodic_io import PIO


'''
TODO 
steps to create a path to a target pose, and travel to said pose while
keeping track of both where it is currently and where it is going on an otherwise 
empty occupancy grid.

1.) create an instance of the path follower, which is a child class to the 
path finder
2.) make the occupancy grid, and plot the target locations on the occupancy grid
3.) create a goal node that will be inserted into the path finder. the path finder will then make a list of 
    smaller target nodes, which are normally lined up. These will then be pruned to remove nodes that are in a straight line to 
    increase the overall speed.
4.) go to said targets, not stopping until the sub is within a certain range of the goal x and y coordinates
    4.5.) the sub should first determine its goal yaw, turn to that. Then determine its goal surge and go forward until it reaches
    its goal. The sub should constantly be updating its current yaw, surge, and global
5.) repeat step 4, moving onto the next target.
'''
# 1.) make an instance of the path follower
follower = PathFollowing()

# 2.) Set the Occupancy Grid
follower.setOccupancyGrid(10,10,10,010,01)

# 3.) create the goal node and insert it into the path finder
follower.setGoalNode(10.0, 10.0)
path = follower.findPath(follower.currentNode, follower.goalNode, follower.occupancyGrid)

# 4.) go through each mini-goal node until every node in the path has been visited.
for node in path:
    follower.goToTarget(node)
    # comment yo
# Starting at n1, go to n2
