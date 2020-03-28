import numpy as np
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
        self.cost = None
        self.RobotDirection = np.array([1, 1]) # Need this to determine if the robot is turning or not for mud heuristic

    def isInList(self, nodeList):
        inListFlag = False
        for node in nodeList:
            if (node.xcoord == self.xcoord) and (node.xcoord == self.ycoord):
                inListFlag = True
        return inListFlag