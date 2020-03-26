from Scheduler import *
class RR_Scheduler(Scheduler):
    def __init__(self, graph):
        super().__init__(graph)
        pass

    #OVERLOADED FUNCTION
    def getNextNode(self, currentNode):
        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode)
        # keep track of cheapest node
        # for each of the neighbors
        # for each heuristic:
        # calculate edge-cost
        # calculate heuristic
        # cost = edge-cost + heuristic
        # if cost cheaper than cheapest
        # new cheapest node found
        pass