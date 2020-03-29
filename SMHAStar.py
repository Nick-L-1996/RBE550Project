from SchedulersShared import *
from RR_SchedulerShared import *
import time

class SMHAStar:
    #self.Algorithm_RR = SMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "Round Robin")
    def __init__(self, map, start, end, scheduler = "EISMHA"):
        self.visited = []
        self.unVisited = []
        self.finalPath = []
        self.startNode = start
        self.endNode = end
        self.foundGoal = False
        # Need world object from GUI
        self.map = map
        # scheduler goes here, initially a string until map is updated
        self.schedulerType = scheduler
        self.scheduler = None
        pass


    # Returns a scheduler corresponding to input
    def getScheduler(self, scheduler):
        if scheduler == "Round Robin":
            return RR_SchedulerShared(self.map)

        return EISMHA_Scheduler(self.map)


    def run(self):
        print("Start:", self.startNode.row, self.startNode.column)
        print("End:", self.endNode.row, self.endNode.column)
        # intialize scheduler, now with the updated map
        self.scheduler = self.getScheduler(self.schedulerType)
        
        # add start to open set
        self.unVisited.append(self.startNode)

        # while open is not empty or the goal not found, keep searching
        while (self.foundGoal != True and len(self.unVisited) != 0):
            #curr current node  from un top of open set
            currentNode = self.unVisited.pop(0)
            print(currentNode.row, currentNode.column)
            # if the current node is the end, then break to return path
            if currentNode == self.endNode:
                print ("found goal")
                self.foundGoal = True
                break
            
            # we have visited the current node
            self.visited.append(currentNode)

            # updated unVisited queue with unvisited queue and cost of neighbors
            # sort unvisited queue by total cost
            self.unVisited = self.scheduler.updateUnvisitedQueue(currentNode, self.visited, self.unVisited, self.endNode)
            time.sleep(2)
        if(self.foundGoal):
            #Determine final path, by backtracking to source
            #return final path
            print("Found Path!")
            print(self.endNode.row, self.endNode.column)
            pass
        
        else:
            print("Could not find goal node!")
        
        pass