#@TODO: How are we doing the sxploitation? Randomly or like DTS?
#       We'll just say random for now. 
from SchedulersShared import *
import random
class EISMHA_SchedulerShared(SchedulerShared):
    def __init__(self, map):
        super().__init__(map)
        pass

    #OVERLOADED FUNCTION
    def Expand(self, currentNode, Explored, FrontierQueue, endNode, isGreedy, epsilon):
        
        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode)
        newFrontierNodes = [] #needed for GUI
        
        #print("Current Node:", currentNode.row, currentNode.column)
        # get unexplored neighbors
        # for every neighbor find the lowest value across all heuristics
        numberExpansions = 0
        for neighbor in neighbors:
            #print("Neighbor Node:", neighbor.row, neighbor.column)
            
            # if this neighbor was marked as visited
            if(neighbor in Explored): ## removed check for trees

               #print("Visited", neighbor.row, neighbor.column)
               continue
                
            terrain_type = neighbor.Environment
            #if(terrain_type != "Concrete"):
            #    print("Environment:", terrain_type)    

            # get the edge cost between the current node and neighbor use same key as heuristic dict because the strings are the same.
            edgeCost = currentNode.getNeighborEdgeCost(neighbor)
            #print("Edge Cost:", edgeCost)

            # FOR NOW SET THIS HERE SO THIS RUNS
            exploiting = None
            
            """
            For each of the neighbor, apply the heuristic that pertains to its terrain. 
            A random percent of the time, it will choose another random heursitic and compare it to the one its associated with
            If the one that is lower is actually cheaper, then it gets rewarded 
            """
            #Apply the heuristic type and get the cost
            #@TODO: EXPLOTING OR EXPLORING?
            if(exploiting):
                heuristicCost = self.getHeuristicValue(neighbor.Environment, currentNode, neighbor, endNode)
            else:
                # random other heuristic
                randHeur = random.choice(list(self.heuristicGetters.keys()))
                heuristicCost = self.getHeuristicValue(randHeur, currentNode, neighbor, endNode)

    
            chosenHeuristic = None
            # check a neighbor with all heuristics
            # for each key in the heuristic getters.
            if isGreedy:
                tempCost = heuristicCost
            else:
                tempCost = currentNode.CostToTravel + edgeCost + heuristicCost
            # if new temp path cost is cheaper than another current path cost to get to neighbor, update neighbor cost
            if tempCost < neighbor.PriorityQueueCost:
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