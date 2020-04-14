from SchedulersShared import *
import random
import sys
class EISMHA_SchedulerShared(SchedulerShared):
    def __init__(self, map, epsilon):
        super().__init__(map)
        self.lastHeuristicValue = sys.maxsize
        # Epsilon value (thershold) for probability of exploitation  
        self.epsilon = epsilon
        
        #List of heuristics available for EISMHA Terrain Performance Metric
        heuristics = [HeuristicWater(),
            HeuristicMud(),
            HeuristicConcrete(),
            HeuristicTrees(),
            HeuristicSand()]
        
        # This dictionary holds objects of type TerrainPerformance. The keys are the terrain type, and the 
        # keys are objects that hold the ability to update the performance of the heuristics for each terrain
        self.TerrainPerformance = {
            "Water" : TerrainPerformanceTracker (heuristics,   [50, 1, 1, 1, 1], [1, 1, 1, 1, 1], 100),
            "Concrete" : TerrainPerformanceTracker(heuristics, [1, 50, 1, 1, 1], [1, 1, 1, 1, 1], 100),
            "Sand" : TerrainPerformanceTracker (heuristics,    [1, 1, 1, 1, 50], [1, 1, 1, 1, 1], 100),
            "Trees" : TerrainPerformanceTracker (heuristics,   [1, 1, 1, 50, 1], [1, 1, 1, 1, 1], 100),
            "Mud" : TerrainPerformanceTracker (heuristics,     [1, 50, 1, 1, 1], [1, 1, 1, 1, 1], 100)
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
                
            #grab the terrain type
            terrain_type = neighbor.Environment

            # get the edge cost between the current node and neighbor use same key as heuristic dict because the strings are the same.
            edgeCost = currentNode.getNeighborEdgeCost(neighbor)
            #print("Edge Cost:", edgeCost)

            # Random number generator to determine if exploiting
            rand = random.random()
            if (rand > self.epsilon):
                exploiting = True
            else:
                exploiting = False
           
            # Use the terrain type to get the best heuristic as of right now
            chosenHeuristic = self.TerrainPerformance[terrain_type].getBestHeuristic(exploiting)
            
            # get the heuristic value of the neighbor using the chosen heuristic
            heuristicCost = chosenHeuristic.getHeuristic(currentNode, neighbor, endNode)

            if(heuristicCost < self.lastHeuristicValue):
                #call updateMetaMethod which will reward that heuristic
                self.TerrainPerformance[terrain_type].UpdateMetaMethod(True)
            else:
                # call updateMetaMethod which will punish that heuristic
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

class HeuristicPerformanceObject:
    """
    Class used to keep track of the performance of each individual heuristic
    
    Parameteters to Constructor: 
    Heuristic: Heuristic object
    alpha (int): number of rewards it starts with, and keeps track of times rewarded for it's performance
    beta (int): number of punishments it starts with, and keeps track of times punished for it's performance
    C (int): Memory of this object, the sum of alpha and beta cannot exceed this amount
    """
    def __init__(self, Heuristic, alpha, beta, C):
        self.C = C # history limit, number of rewards and punishes given
        self.alpha = alpha # number of times rewarded
        self.beta = beta # number of times punished
        self.Heuristic = Heuristic # heuristic object to keep track of

    # rewarding a heuristic means incrementing alpha
    def rewardHeuristic(self):
        self.alpha+=1 
        self.limitMem()
        
    # punishing a heuristic means incrementing beta
    def punishHeuristic(self):
        self.beta+=1
        self.limitMem()

    # Scale alpha and beta if they exceed limit after rewarding/punishing
    def limitMem(self):
        if (self.alpha + self.beta) > self.C: # if number of rewards and punishes are greater than history allows, then scale alpha and beta to be within limit
            self.alpha *= self.C / (1 + self.C)
            self.beta *= self.C / (1 + self.C)

    def betaDistribution2(self, xnew): # returns the highest probability density at a given point
        return np.power(xnew, self.alpha-1)*np.power((1-xnew), self.beta-1) # beta distribution function

class TerrainPerformanceTracker:
    """
    Class used to update the performance of a specific terrain and its associated heuristics
    
    Parameteters to Constructor: 
    Heuristics (List of Heuristic Objects): a list containing all possible types of heuristics
    ListOfStartingAlphas (int []): the initial alpha (number of times its been rewarded) for each heuristic, in order of the Heuristics list
                                    Here we want to "reward" heursitics that match the terrain type
    ListOfStartingBetas (int []): the initial beta (number of times its been punished) for each heuristic, in order of the Heuristics list
    C (int): The number of rewards/punishments we want to keep track of before scaling them. Basically, this is the "history".
    """
    def __init__(self, Heuristics, ListOfStartingAlphas, ListOfStartingBetas, C):
        #heuristicList is list of heuristic objects
        #self.heuristicList is list of EISHMAHeuristicObjects
        self.HeuristicPerformanceObjectList = []
        #self.Heuristics = Heuristics
        self.lastReturnedHeuristicIndex = 0
        # for every heuristic in the list, create a HeuristicPerformanceObject, which will be responsible for punishing and rewarding based on heuristic performance
        for index in range(0, len(Heuristics)):
            # We will construct the HeuristicPerformanceObject with the proper heuristic, the corresponding starting alpha and beta, and the history variable, C. 
            self.HeuristicPerformanceObjectList.append(HeuristicPerformanceObject(Heuristics[index], ListOfStartingAlphas[index], ListOfStartingBetas[index], C))
    
    def getBestHeuristic(self, Exploiting):
        """
        This method iterates through the list of EISMHAHeuristic Objects (the objects that essentially hold heuristic performance), and determines
        the best heuristic to return based on the beta distribution of the object.

        Parameters:
        Exploiting (bool): whether or not we are currently exploiting

        Returns:
        The best heuristic object according to the beta distribution
        """
        # update lastReturnedHeuristicIndex
        # Iterate through heuristics, call betaDistribution2(75%) on each object in the list
        # find the best one one, and the second best one.
        heuristicBetaDistribution = []
        betaThresh = .75
        for i in range(len(self.HeuristicPerformanceObjectList)):
            # append to our list a list of shape (HeuristicType, betaDistribtion of that heuristic, index of that heuristic)
            heuristicBetaDistribution.append((self.HeuristicPerformanceObjectList[i].Heuristic, self.HeuristicPerformanceObjectList[i].betaDistribution2(betaThresh), i))
        
        # Sort the heuristics by  so that you can easily access the 1st and 2nd best performed heuristics
        heuristicBetaDistribution.sort(key=lambda x: x[1])
       
        # Depending on whether or not we are exploiting, we want to either choose the best or second best heuristic
        if(Exploiting):
            # return the best Heuristic, update lastReturnedHeuristicIndex
            # the array structure we are grabbing from is (Heuristic Type, betaDistribution, index)
            # We use the index so we know which one to punish or reward later
            self.lastReturnedHeuristicIndex = heuristicBetaDistribution[0][2]
            return heuristicBetaDistribution[0][0]
        else:
            # return second best Heuristic, update lastReturnedHeuristicIndex
            # the array structure we are grabbing from is (Heuristic Type, betaDistribution, index)
            # We use the index so we know which one to punish or reward later
            self.lastReturnedHeuristicIndex = heuristicBetaDistribution[1][2]
            return heuristicBetaDistribution[1][0]
    
    # update heuristics based on performance
    def UpdateMetaMethod(self, didBetter):
        """
        This rewards or punishes a heuristic depending on whether or not it has preformed better or worse than the previous heuristic. 

        Parameters:
        didBetter (bool): whether or not the heuristic did better than before

        Returns:
        None
        """
        # If the heuristic performed better, then reward it
        if didBetter:
            # Access the last used Heuristic in the list and call it's rewardHeuristic function to reward it
            self.HeuristicPerformanceObjectList[self.lastReturnedHeuristicIndex].rewardHeuristic()
        # Otherwise punish it
        else:
            # Access the last used Heuristic in the list and call it's punishHeuristic function to punish it
            self.HeuristicPerformanceObjectList[self.lastReturnedHeuristicIndex].punishHeuristic()
