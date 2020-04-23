from SchedulersShared import *
from RR_SchedulerShared import *
from EISMHA_SchedulerShared import *
import time

class SharedQueueAlgorithm:
    #self.Algorithm_RR = SMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "Round Robin")
    def __init__(self, map, end, otherArguements, algorithm ="EISMHA", verbose = False):
        # algorithm needs the map and goal, and other arguements if necassary
        # For example, WA* needs an epsilon value to work so this can be added here. If multiple
        # other arguements are needed use a list or some other data structure
        self.endNode = end
        self.otherArgs = otherArguements
        # Need world object from GUI
        self.algorithm = algorithm
        self.map = map
        self.verbose = verbose
        self.scheduler, self.isGreedy = self.getScheduler(algorithm)

    #This function is needed for compatibility with GUI
    def updateParameters(self, Map, Goal, otherArguements):  # algorithm needs the map and goal, and other arguements if necassary
        # For example, WA* needs an epsilon value to work so this can be added here. If multiple
        # other arguements are needed use a list or some other data structure
        self.map = Map
        self.endNode = Goal
        self.otherArgs = otherArguements
        self.scheduler, self.isGreedy = self.getScheduler(self.algorithm)

    # Returns a scheduler corresponding to input
    def getScheduler(self, algorithm):
        if algorithm == "MHA*":
            return RR_SchedulerShared(self.map, self.verbose), False #Since this is the shared queue algorithm class, using the round robin scheduler and not being greedy (i.e A*) makes this SMHA*
        elif algorithm =="MHGBFS":
            return RR_SchedulerShared(self.map, self.verbose), True #this is Shared Multiheuristic Greedy Best First Search
        elif algorithm == "EISMHA":
            return EISMHA_SchedulerShared(self.map, self.otherArgs, self.verbose), False
    def clearData(self):
        self.scheduler.clearData()
    # This function is needed for compatibility with GUI
    #takes in the frontier and explored queues
    #returns 6 pieces of information
    #1 boolean if goal was found or not
    #2 new nodes added to the frontier queue. This is used to update the GUI\
    #3 returns the current node so that the GUI can update this node from frontier to explored
    #4 returns a boolean for if the queue is empty. Used to determine if there isnt a possible path
    #5 returns the frontier queue
    #6 number of expansions

    def run(self, ExploredList, FrontierQueue):
        QueueEmpty = False
        NewFrontierNodes = []
        GoalFound = False
        CurrentNode = FrontierQueue[0]
        FrontierQueue.remove(CurrentNode)
        numExpansions = 0
        if CurrentNode == self.endNode:
            #print ("found goal")
            GoalFound = True
        else:
        # updated unVisited queue with unvisited queue and cost of neighbors
        # sort unvisited queue by total cost
            FrontierQueue, NewFrontierNodes, numExpansions = self.scheduler.Expand(CurrentNode, ExploredList, FrontierQueue, self.endNode, self.isGreedy)
        if (len(FrontierQueue) == 0):
            QueueEmpty = True
        
        return GoalFound, NewFrontierNodes, CurrentNode, QueueEmpty, FrontierQueue, numExpansions
