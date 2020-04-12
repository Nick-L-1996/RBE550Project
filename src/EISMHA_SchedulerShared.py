#@TODO: How are we doing the exploitation? Randomly or like DTS?
#       We'll just say random for now. 
from SchedulersShared import *
import random
import sys
class EISMHA_SchedulerShared(SchedulerShared):
    def __init__(self, map, epsilon):
        super().__init__(map)
        self.lastHeuristicValue = sys.maxsize
        self.epsilon = epsilon
        heuristics = [HeuristicWater(),
            HeuristicMud(),
            HeuristicConcrete(),
            HeuristicTrees(),
            HeuristicSand()]
        self.TerrainPerformance = {
            "Water" : EISMHATerrainPerformance (heuristics,   [50, 1, 1, 1, 1], [1, 1, 1, 1, 1], 100),
            "Concrete" : EISMHATerrainPerformance(heuristics, [1, 50, 1, 1, 1], [1, 1, 1, 1, 1], 100),
            "Sand" : EISMHATerrainPerformance (heuristics,    [1, 1, 1, 1, 50], [1, 1, 1, 1, 1], 100),
            "Trees" : EISMHATerrainPerformance (heuristics,   [1, 1, 1, 50, 1], [1, 1, 1, 1, 1], 100),
            "Mud" : EISMHATerrainPerformance (heuristics,     [1, 50, 1, 1, 1], [1, 1, 1, 1, 1], 100)
        }

    #OVERLOADED FUNCTION
    def Expand(self, currentNode, Explored, FrontierQueue, endNode, isGreedy):
        
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

            # Random number generator to determine if exploiting
            rand = random.random()
            if (rand > self.epsilon):
                exploiting = True
            else:
                exploiting = False
           
            # Heuristic Object Type
            chosenHeuristic = self.TerrainPerformance[terrain_type].getHeuristic(exploiting)
            heuristicCost = chosenHeuristic.getHeuristic(currentNode, neighbor, endNode)

            if(heuristicCost < self.lastHeuristicValue):
                #reward that heuristic
                self.TerrainPerformance[terrain_type].UpdateMetaMethod(True)
            else:
                #punish that heuristic
                self.TerrainPerformance[terrain_type].UpdateMetaMethod(False)

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

class EISMHAHeuristicObject:
    def __init__(self, Heuristic, alpha, beta, C):
        self.C = C
        self.alpha = alpha
        self.beta = beta
        self.Heuristic = Heuristic

    def rewardHeuristic(self):
        self.alpha+=1
        self.limitMem()

    def punishHeuristic(self):
        self.beta+=1
        self.limitMem()

    def limitMem(self):
        if (self.alpha + self.beta) > self.C:
            self.alpha *= self.C / (1 + self.C)
            self.beta *= self.C / (1 + self.C)

    def betaDistribution2(self, xnew): # returns the highest probability density at a given point
        return np.power(xnew, self.alpha-1)*np.power((1-xnew), self.beta-1)#beta distribution function

class EISMHATerrainPerformance:
    def __init__(self, Heuristics, ListOfStartingAlphas, ListOfStartingBetas, C):
        #heuristicList is list of heuristic objects
        #self.heuristicList is list of EISHMAHeuristicObjects
        self.EISMHAHeuristicObjectList = []
        #self.Heuristics = Heuristics
        self.lastReturnedHeuristicIndex = 0
        for index in range(0, len(Heuristics)):
            self.EISMHAHeuristicObjectList.append(EISMHAHeuristicObject(Heuristics[index], ListOfStartingAlphas[index], ListOfStartingBetas[index], C))
    
    def getHeuristic(self, Exploiting):
        # update lastReturnedHeuristicIndex
        # Iterate through heuristics, call betaDistribution2(75%) on each object in the list
        # find the best one one, and the second best one. Store it. 

        #Type, betaDistribution
        heuristicBetaDistribution = []
        for i in range(len(self.EISMHAHeuristicObjectList)):
            heuristicBetaDistribution.append((self.EISMHAHeuristicObjectList[i].Heuristic, self.EISMHAHeuristicObjectList[i].betaDistribution2(.75), i))
        
        heuristicBetaDistribution.sort(key=lambda x: x[1])
        # bestIndex = heuristicBetaDistribution.index(max(heuristicBetaDistribution))
        # Bear with me here
        # secondBestIndex = heuristicBetaDistribution.index(max(value for value in heuristicBetaDistribution if value!=max(heuristicBetaDistribution)))
        if(Exploiting):
            # return the best Heuristic, update lastReturnedHeuristicIndex
            self.lastReturnedHeuristicIndex = heuristicBetaDistribution[0][2]
            return heuristicBetaDistribution[0][0]
        else:
            # return second best Heuristic
            self.lastReturnedHeuristicIndex = heuristicBetaDistribution[1][2]
            return heuristicBetaDistribution[1][0]

    def UpdateMetaMethod(self, didBetter):
        # If better reward, else punish
        if didBetter:
            self.EISMHAHeuristicObjectList[self.lastReturnedHeuristicIndex].rewardHeuristic()
        else:
            self.EISMHAHeuristicObjectList[self.lastReturnedHeuristicIndex].punishHeuristic()
