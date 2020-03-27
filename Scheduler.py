class Scheduler:
    #scheduler needs to know what the world looks like
    def __init__(self, map):
        self.map = map
        pass

    def updateUnvisitedQueue(self, currentNode, visited, unvisited):
        pass

    def getEdgeCost(self, currentNode, parentNode):
        #if the two have the same row but columns off by 1, cost is 1
        # if the two have the same column but row off by 1, cost is 1
        # if both column and row is off by 1, cost is sqrt(2)
        pass

    def getNodeNeighbors(self, currentNode):
         # N,E,S,W
        neighbors = []
        row = self.currentNode.row
        column = self.currentNode.column

        #If we have neighbors to the north
        if(row > 0):
            neighbors.append(self.map[row - 1][column])
        # If we have neighbors to the east
        if (column < self.nodesWide - 1):
            neighbors.append(self.map[row][column + 1])
        # If we have neighbors to the south
        if (row < self.nodesTall - 1):
            neighbors.append(self.map[row + 1][column])
        # If we have neighbors to the west
        if (column > 0):
            neighbors.append(self.map[row][column - 1])

        #NW, NE, SW, SE
        # If we have neighbors to the north and west
        if (row > 0 and column > 0):
            neighbors.append(self.map[row - 1][column - 1])
        #north east
        if (row > 0 and column < self.nodesWide - 1):
            neighbors.append(self.map[row - 1][column + 1])
        # If we have neighbors to the south west
        if (row < self.nodesTall - 1 and column > 0):
            neighbors.append(self.map[row + 1][column - 1])
        # south east
        if (row < self.nodesTall - 1 and column < self.nodesWide - 1):
            neighbors.append(self.map[row + 1][column + 1])

        return neighbors

    def getHeuristic(self, endNode):
        #call a method on the heuristic object
        pass
