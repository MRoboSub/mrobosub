import numpy as np
import rospy
from std_msgs.msg import Float64, Float32
from Nodes import Node

#TODO - Determine size of sub in order to make approporiate configuration space
class OccupancyGrid:

    
    
    #initialize the width and height in Inches
    def __init__(self, width, height, inchesPerCell, cellsPerInch, occupiedThreshold):
        self.widthInInches_ = width
        self.heightInInches_ = height
        self.inchesPerCell_ = inchesPerCell
        self.cellsPerInch_ = cellsPerInch
        self.occupiedThreshold_ = occupiedThreshold
        self.widthInCells_ = self.widthInInches_ / self.inchesPerCell_
        self.heightInCells_ = self.widthInInches_ / self.inchesPerCell_
        self.numberOfCells_ = self.widthInCells_ * self.heightInCells_
        self.globalOrigin_ = [self.widthInCells_ / 2.0, self.heightInCells_ / 2.0]
        #cells_ is the matrix in which all data is kept track of
        self.cells_ = self.createArray()

        rospy.Subscriber('/pose/x_pos', Float64, self.pose_callback)
        rospy.Subscriber('/pose/y_pos', Float64, self.pose_callback)

        assert width > 0
        assert height > 0
        assert inchesPerCell < width
        assert inchesPerCell < height

    def pose_callback(self, msg):
        self.sub_pos = [msg.x_pos, msg.y_pos]


    def setGlobalOrigin(self, x,y):
        print("Setting Global Origin")
        self.reset()
        self.globalOrigin = [x,y]

    def reset(self):
        print("Resetting Occupancy Grid")
        grid = {}
        #create a grid of (heightInCells_) rows and (widthInCells_) columns
        for row in range(self.heightInCells_):
            for col in range(self.widthInCells_):
                grid[(row, col)] = Node([row,col])
        return grid
    
    def createArray(self):
        print("Setting Occupancy Grid")
        grid = {}
        #create a grid of (heightInCells_) rows and (widthInCells_) columns
        for row in range(self.heightInCells_):
            for col in range(self.widthInCells_):
                grid[(row, col)] = Node([row,col]) 
        return grid
        

    def getOccupancy(self, x, y):
        if self.isCellInGrid(x,y):
            return self.cells_[x,y].occupancy

    # for setting non-binary log odds | Would like to use for correcting map with onboard cameras, etc, do later
    def setLogOdds(self, x, y, cellOdds):
        if self.isCellInGrid(x,y):
            node = self.cells_[x,y]
            node.occupancy = cellOdds
            self.cells_[x, y].occupancy = cellOdds

    # for settiing binary occupancy | Used for setting targets initially
    def setOccupancy(self, x,y):
        if self.isCellInGrid(x,y):
            self.cells_[x,y].occupancy = 1

    
    def isCellOccupied(self, x, y):
        if self.isCellInGrid(x,y):
            return self.cells_[x,y].occupancy > 0
    
    def setCellTarget(self, x, y, type):
        if self.isCellInGrid(x,y):
            self.setOccupancy(x,y)
            self.cells_[x,y].setTargetType()

    def isCellInGrid(self, x, y):
        xCoordValid = (x>=0) and (x < self.widthInCells_)
        yCoordValid = (y>=0) and (y < self.heightInCells_)
        return [xCoordValid and yCoordValid]
    
