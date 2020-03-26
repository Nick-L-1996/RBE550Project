class Scheduler:
    #scheduler needs to know what the world looks like
    def __init__(self, graph):
        self.graph = graph
        pass

    def getNextNode(self, currentNode):
        #get neighbors of current node
        #find best neighbor to go to
        pass

    def getNodeNeighbors(self, currentNode):
        pass

    def getEdgeCost(self, currentNode, parentNode):
        pass

    def getHeuristic(self, endNode):
        pass
