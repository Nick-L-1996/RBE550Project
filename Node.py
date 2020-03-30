import numpy as np
from TerrainCosts import *
class Node:
    def __init__(self, x, y, row, column):
        self.xcoord = x
        self.ycoord = y
        self.row = row
        self.column = column
        self.parent = None
        self.CostToTravel = 10000000000
        self.Heuristic = 10000000000
        self.PriorityQueueCost = 1000000000000
        # Ground should be concrete if you don't make it anything else
        self.Environment = "Concrete"
        self.costClass = EdgeCostConcrete()
        self.RobotDirection = np.array([1, 1]) # Need this to determine if the robot is turning or not for mud heuristic

    def isInList(self, nodeList):
        inListFlag = False
        for node in nodeList:
            if (node.xcoord == self.xcoord) and (node.xcoord == self.ycoord):
                inListFlag = True
        return inListFlag

    def setEnvironmentType(self, Type):#Takes String of environment Type. Used by map generator
        if Type == "Trees":
            self.costClass = EdgeCostTrees()
        elif Type == "Mud":
            self.costClass = EdgeCostMud()
        elif Type == "Sand":
            self.costClass = EdgeCostSand()
        elif Type == "Water":
            self.costClass = EdgeCostWater()
        elif Type == "Concrete":
            self.costClass = EdgeCostConcrete()
        self.Environment = Type

    def getNeighborEdgeCost(self, Neighbor): # pass ion the last node in the path to calculate the cost from it, to the node being expanded
        return Neighbor.costClass.getTerrainEdgeCost(self, Neighbor)


    #use this method when a node is selected by the scheduler to be added to the path
    def fullyExpandNode(self, costToTravel, Heuristic, totalCost, parentNode):# pass in the found values for heuristic and costs so that it isnt recalculated again
        self.CostToTravel = costToTravel
        self.Heuristic = Heuristic
        self.PriorityQueueCost = totalCost
        self.parent = parentNode
        self.RobotDirection = np.array([self.xcoord - parentNode.xcoord, self.ycoord - parentNode.ycoord])