from SchedulersShared import *
class RR_SchedulerShared(SchedulerShared):
    def __init__(self, map, verbose = False):
        super().__init__(map, verbose)
        self.verbose = verbose

    def clearData(self):
        pass # No data to clear

    #OVERLOADED FUNCTION
    def Expand(self, currentNode, Explored, FrontierQueue, endNode, isGreedy, randomVar):
        
        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode)
        newFrontierNodes = [] #needed for GUI
        
        if(self.verbose):
            print("Current Node:", currentNode.row, currentNode.column)
        # get unexplored neighbors
        # for every neighbor find the lowest value across all heuristics
        numberExpansions = 0
        for neighbor in neighbors:
            if(self.verbose):
                print("Neighbor Node:", neighbor.row, neighbor.column)
            
            # if this neighbor was marked as visited
            if(neighbor in Explored): ## removed check for trees
                if(self.verbose):
                    print("Visited", neighbor.row, neighbor.column)
                continue
                
            terrain_type = neighbor.Environment  

            # get the edge cost between the current node and neighbor use same key as heuristic dict because the strings are the same.
            edgeCost = currentNode.getNeighborEdgeCost(neighbor, randomVar)
            
            if(self.verbose):
                print("Edge Cost:", edgeCost)

            chosenHeuristic = None
            # check a neighbor with all heuristics
            # for each key in the heuristic getters.

            for heuristic_type in self.heuristicGetters.keys():
                
                # get the hueristic value between the current node and the neighbor using specific hueristic and end goal
                heuristicCost = self.getHeuristicValue(heuristic_type, currentNode, neighbor, endNode)
                
                if(self.verbose):
                    print("Heuristic Cost:", heuristic_type, heuristicCost)

                # calculate the would be cost to travel from current node to neighbor
                #allows for this scheduler to become SMHA* or SMHGBFS depending on if Greedy or not
                if isGreedy:    
                    tempCost = heuristicCost
                else:
                    tempCost = currentNode.CostToTravel + edgeCost + heuristicCost
                # if new temp path cost is cheaper than another current path cost to get to neighbor, update neighbor cost
                if tempCost < neighbor.PriorityQueueCost:
                    chosenHeuristic = heuristic_type
                    ########NEW NICK ADDED######## updates robot direction and all other needed paramaters
                    neighbor.fullyExpandNode(currentNode.CostToTravel + edgeCost, heuristicCost, tempCost,  currentNode)
            
                numberExpansions+=1
            # if neighbor is not in the unvisited list, add it to unvisited
            if (neighbor not in FrontierQueue):
                #print ("Added Neighbor:", neighbor.row, neighbor.column, chosenHeuristic, neighbor.Environment, neighbor.CostToTravel)
                newFrontierNodes.append(neighbor)
                FrontierQueue.append(neighbor)
    

        # sort nodes in unvisited by their cost
        FrontierQueue.sort(key=lambda x: x.PriorityQueueCost)

        # return Frontier Queue, newFrontierNodes, number of expansions
        return FrontierQueue, newFrontierNodes, numberExpansions