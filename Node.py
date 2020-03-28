import numpy as np
from TerrainCosts import *
class Node:
    def __init__(self, x, y, row, column):
        self.xcoord = x
        self.ycoord = y
        self.row = row
        self.column = column
        self.parent = None
        self.totalCost = 10000000000
        self.Heuristic = 10000000000
        self.Environment = "None"
        self.costClass = terrainCosts()
        self.RobotDirection = np.array([1, 1]) # Need this to determine if the robot is turning or not for mud heuristic

    def isInList(self, nodeList):
        inListFlag = False
        for node in nodeList:
            if (node.xcoord == self.xcoord) and (node.xcoord == self.ycoord):
                inListFlag = True
        return inListFlag

    def setEnvironmentType(self, Type):#Takes String of environment Type. Used by map generator
        if Type == "Trees":
            self.costClass = edgeCostTrees()
        elif Type == "Mud":
            self.costClass = edgeCostMud()
        elif Type == "Sand":
            self.costClass = edgeCostSand()
        elif Type == "Water":
            self.costClass = edgeCostWater()
        elif Type == "Concrete":
            self.costClass = edgeCostConcrete()
        self.Environment = Type

    def getEdgeCost(self, CurrentNode): # pass ion the last node in the path to calculate the cost from it, to the node being expanded
        return self.costClass.getEdgeCost(CurrentNode, self)


    #use this method when a node is selected by the scheduler to be added to the path
    def fullyExpandNode(self, totalCost, Heuristic, parentNode):# pass in the found values for heuristic and costs so that it isnt recalculated again
        self.totalCost = totalCost
        self.Heuristic = Heuristic
        self.parent = parentNode
        self.RobotDirection = np.array([self.xcoord - parentNode.xcoord, self.ycoord - parentNode.ycoord])