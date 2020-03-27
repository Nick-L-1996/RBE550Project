from Scheduler import *
class SMHAStar:
    #self.Algorithm_RR = SMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "Round Robin")
    def __init__(self, map, start, end, scheduler = "EISMHA"):
        self.visited = []
        self.unVisited = []
        self.finalPath = []
        self.startNode = start
        self.endNode = end
        self.foundGoal = False
        # Need world or graph object from GUI
        self.graph = map
        #scheduler goes here
        self.scheduler = getScheduler(scheduler)
        pass


    # Returns a scheduler corresponding to input
    def getScheduler(scheduler):
        if scheduler == "Round Robin":
            return RR_Scheduler()
        elif scheduler == "DTS":
            return DTS_Scheduler()

        return EISMHA_Scheduler()


    def run(self):
        # add start to open set
        self.unVisiting.append(self.startNode)
        # while open is not empty or the goal not found, keep searching
        while (self.foundGoal != True or len(self.unVisited) != 0)

        #curr current node  from un top ofopen set
            currentNode = self.unVisited.pop(0)

            # if the current node is the end, then break to return path
            if currentNode == self.endNode:
                print ("found goal")
                self.foundGoal = True
                break
            
            # we have visited the current node
            self.visited.append(currentNode)

            # updated unVisited queue with unvisited queue and cost of neighbors
            # sort unvisited queue by total cost
            currentNodeNeigbors = scheduler.updateUnvisitedQueue(currentNode, self.visited, self.unVisited)
        
        if(self.foundGoal):
            #Determine final path, by backtracking to source
            #return final path
            pass
        
        else:
            print("Could not find goal node!")
        
        pass