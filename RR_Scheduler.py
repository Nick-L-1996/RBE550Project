from Scheduler import *
class RR_Scheduler(Scheduler):
    def __init__(self, graph):
        super().__init__(graph)
        pass


        """
         @RICH I didn't REALLY follow the pseudo-code itself (as written) from the paper, but this is how I understood it:
             1) put start node in unvisited and pop off the unvisited list
             2) current node = popped off node
             3) at current node, have scheduler determine next best node to go
             4) rewire the nodes parent if necessary
             5) add current node to visited
             6) add next node to the unvisited list, and sort the list by total cost (h + g)
             7) Go to step 2
        """
    #OVERLOADED FUNCTION
    def updateUnvisitedQueue(self, currentNode, visited, unvisited):
        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode)
        
        # get unexplored neighbors

        
        for neighbor in neighbors:
            for heuristic in heuristics:

                # get the edge cost between the current node and neighbor
                edgeCost = self.getEdgeCost(currentNode,neighbor)

                # get the hueristic value between the current node and the neighbor
                heuristicCost = self.getHeuristic(currentNode,neighbor)
                
                # calculate the would be cost to travel from current node to neighbor
                tempCost = edgeCost + heuristicCost

                # if new path cost is cheaper than another current path cost to get to neighbor, update neighbor cost
                if tempCost < neighbor.cost:
                    neighbor.cost = tempCost
                
                
        


        # keep track of cheapest node
        # for each of the neighbors
        # for each heuristic:
        # calculate edge-cost
        # calculate heuristic
        # cost = edge-cost + heuristic
        # if cost cheaper than cheapest
        # new cheapest node found

        
        pass