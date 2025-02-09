import numpy as np
import OccupancyGrid as og
#REVIEW - should we set just the center point, or the whole area?
class Node:
    #inialize length and width in inches
    def __init__(self, location):
        self.location = location
        self.x = location[1]
        self.y = location[0]
        self.type = None
        self.parent = None
        self.occupancy = 0
        self.cost = 0
    def __lt__(self, other):
        return self.cost < other.cost
        

    def setTargetType(self,type):
        self.type = type 
        #TODO: set cells within boundary of the target to be equal to 1
        #REVIEW: how do we handle angles. Especially if the targets are large, a missed angle will result in an inaccurate map
        if self.type == "gate":
            self.width = 120 #inches
            self.height = 3  #inches
        elif self.type == "abydos":
            self.width = 12
            self.height = 16                
    
    def setParent(self, parent):
        self.parent = parent

    def setCost(self, h):
        self.cost = h
    
    def getCost(self):
        return self.cost
 