from DynamicThompsonSampling import *
from Heuristics import *

#changed name from IMHA* bc DTS is a greedy best first search algorithm with multiple prioritized Queues as written in the paper
# We can easily make it A* though
class IndependentQueueAlgorithm:
    # self.Algorithm_RR = IMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "DTS")
    def __init__(self, map, start, end, algorithm="DTSGreedy", verbose = False):
        self.Queues = {}
        # add all Queues to list
        self.Queues["Concrete"] = HeuristicConcrete()
        self.Queues["Mud"] = HeuristicMud()
        self.Queues["Sand"] = HeuristicSand()
        self.Queues["Water"] = HeuristicWater()
        self.Queues["Trees"] = HeuristicTrees()
        self.verbose = verbose

        self.algorithm = algorithm
        self.startNodeDict = start
        self.endNodeDict = end
        self.foundGoal = False
        # Need world object from GUI
        self.mapDict = map
        # scheduler goes here
        self.scheduler, self.isGreedy = self.getScheduler(algorithm)
        #the isGreedy object determines if the priority queue considers G + H or just H
    def clearData(self):
        self.scheduler.clearData()
    # Returns a scheduler corresponding to input. Made so we can add more in the future
    def getScheduler(self, scheduler,verbose = False):
        if scheduler == "DTSGreedy":
            return DTS_SchedulerIndependent(self.mapDict, self.Queues, verbose = verbose), True
        elif scheduler =="DTSA*":
            return DTS_SchedulerIndependent(self.mapDict, self.Queues, verbose = verbose), False
        return DTS_SchedulerIndependent(self.mapDict, self.Queues), True


    def updateParameters(self, Map, Goal,
                         otherArguements):  # algorithm needs the map and goal, and other arguements if necassary
        # For example, WA* needs an epsilon value to work so this can be added here. If multiple
        # other arguements are needed use a list or some other data structure
        self.mapDict = Map
        self.endNodeDict = Goal
        self.otherArgs = otherArguements
        self.scheduler, self.isGreedy = self.getScheduler(self.algorithm)

    def run(self, ExploredListDict, FrontierQueueDict):
        QueueEmptyDict = {}
        NewFrontierNodesDict = {}
        GoalFoundDict = {}
        CurrentNodeDict = {}
        numExpansions = 0
        for key in ExploredListDict.keys():
            QueueEmptyDict[key] = False
            NewFrontierNodesDict[key] = []
            GoalFoundDict[key] = False
            CurrentNodeDict[key] = []

        key = self.scheduler.chooseQueue() #pick the queue to expand
        CurrentNodeDict[key] = [FrontierQueueDict[key][0]]
        FrontierQueueDict[key].remove(CurrentNodeDict[key][0])
        if CurrentNodeDict[key][0] == self.endNodeDict[key]:
            if(self.verbose):
                print("found goal")
            GoalFoundDict[key] = True
        else:
            FrontierQueueDict[key], NewFrontierNodesDict[key], numExpansions = self.Expand(CurrentNodeDict[key][0], ExploredListDict[key],
                                                                                       FrontierQueueDict[key], self.endNodeDict[key],
                                                                                       self.isGreedy, self.Queues[key], self.mapDict[key])
        if (len(FrontierQueueDict[key]) == 0):
            QueueEmptyDict[key] = True

        #update the Meta of the scheduler
        self.scheduler.UpdateMetaMethod(key, FrontierQueueDict[key][0].PriorityQueueCost)
        return GoalFoundDict, NewFrontierNodesDict, CurrentNodeDict, QueueEmptyDict, FrontierQueueDict, numExpansions

    def Expand(self, currentNode, Explored, FrontierQueue, endNode, isGreedy, heuristic, map):

        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode, map)
        newFrontierNodes = []  # needed for GUI
        
        if(self.verbose):
            print("Current Node:", currentNode.row, currentNode.column)
        # get unexplored neighbors
        # for every neighbor find the lowest value across all heuristics
        numberExpansions = 0
        for neighbor in neighbors:
            # if this neighbor is in the explored list
            if (neighbor in Explored):  ## removed check for trees
                continue

            edgeCost = currentNode.getNeighborEdgeCost(neighbor)
            heuristicCost = heuristic.getHeuristic(currentNode, neighbor, endNode)
            if isGreedy:
                tempCost = heuristicCost
            else:
                tempCost = currentNode.CostToTravel + edgeCost + heuristicCost
            neighbor.fullyExpandNode(currentNode.CostToTravel + edgeCost, heuristicCost, tempCost, currentNode)
            numberExpansions += 1
            # if neighbor is not in the frontier add it
            if (neighbor not in FrontierQueue):
                if(self.verbose):
                    print("Added Neighbor:", neighbor.row, neighbor.column, neighbor.Environment,
                        neighbor.CostToTravel)
                newFrontierNodes.append(neighbor)
                FrontierQueue.append(neighbor)
        # sort nodes in frontier by their cost
        FrontierQueue.sort(key=lambda x: x.PriorityQueueCost)

        # return Frontier Queue, newFrontierNodes, number of expansions
        return FrontierQueue, newFrontierNodes, numberExpansions

    def getNodeNeighbors(self, currentNode, map):
         # N,E,S,W
        neighbors = []
        row = currentNode.row
        column = currentNode.column
        mapHeight = len(map)
        mapWidth = len(map[0])

        #If we have neighbors to the north
        if(row > 0):
            neighbors.append(map[row - 1][column])
        # If we have neighbors to the east
        if (column < mapWidth - 1):
            neighbors.append(map[row][column + 1])
        # If we have neighbors to the south
        if (row < mapHeight - 1):
            neighbors.append(map[row + 1][column])
        # If we have neighbors to the west
        if (column > 0):
            neighbors.append(map[row][column - 1])

        #NW, NE, SW, SE
        # If we have neighbors to the north and west
        if (row > 0 and column > 0):
            neighbors.append(map[row - 1][column - 1])
        # north east
        if (row > 0 and column < mapWidth - 1):
            neighbors.append(map[row - 1][column + 1])
        # If we have neighbors to the south west
        if (row < mapHeight - 1 and column > 0):
            neighbors.append(map[row + 1][column - 1])
        # south east
        if (row < mapHeight - 1 and column < mapWidth - 1):
            neighbors.append(map[row + 1][column + 1])

        return neighbors
