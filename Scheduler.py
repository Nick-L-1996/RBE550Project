import math
from Heuristics import *

#TODO: Make Function Headers

class Scheduler:
    #scheduler needs to know what the world looks like
    def __init__(self, map):
        self.map = map

        ## Data Structure for Heuristic Value getters
        self.heuristicGetters = {
            "Water": heuristicWater(),
            #"Mud": heuristicMud(),
            "Concrete": heuristicConcrete(),
            "Trees": heuristicTrees(),
            "Sand": heuristicSand()
        }
        
        ## Data Structure for Heuristic Chosen Counter (DTS)
        self.heuristicChosenCount = {
            "Water": 0,
            "Mud": 0,
            "Concrete": 0,
            "Trees": 0,
            "Sand": 0
        }

    def updateUnvisitedQueue(self, currentNode, visited, unvisited, endNode):
        pass

    def getEdgeCost(self, currentNode, parentNode):
        # Euclidean Distance 
        edgeCost = math.sqrt(abs(currentNode.row - parentNode.row)**2 + abs(currentNode.column - parentNode.column)**2)
        # return the edge cost
        return edgeCost

    def getNodeNeighbors(self, currentNode):
         # N,E,S,W
        neighbors = []
        row = currentNode.row
        column = currentNode.column
        mapHeight = len(self.map)
        mapWidth = len(self.map[0])

        #If we have neighbors to the north
        if(row > 0):
            neighbors.append(self.map[row - 1][column])
        # If we have neighbors to the east
        if (column < mapWidth):
            neighbors.append(self.map[row][column + 1])
        # If we have neighbors to the south
        if (row < mapHeight):
            neighbors.append(self.map[row + 1][column])
        # If we have neighbors to the west
        if (column > 0):
            neighbors.append(self.map[row][column - 1])

        #NW, NE, SW, SE
        # If we have neighbors to the north and west
        if (row > 0 and column > 0):
            neighbors.append(self.map[row - 1][column - 1])
        # north east
        if (row > 0 and column < mapWidth):
            neighbors.append(self.map[row - 1][column + 1])
        # If we have neighbors to the south west
        if (row < mapHeight and column > 0):
            neighbors.append(self.map[row + 1][column - 1])
        # south east
        if (row < mapHeight and column < mapWidth):
            neighbors.append(self.map[row + 1][column + 1])

        return neighbors


    def getHeuristicValue(self, heuristic, currentNode, neighborNode, endNode):
        #call a method on the heuristic object
        heuristicValue = self.heuristicGetters[heuristic].getHeuristic(currentNode, neighborNode, endNode)
        return heuristicValue