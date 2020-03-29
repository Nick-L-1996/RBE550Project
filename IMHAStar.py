from DynamicThompsonSampling import *

class IMHAStar:
    # self.Algorithm_RR = IMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "DTS")
    def __init__(self, map, start, end, scheduler="DTS"):
        self.Queues = []
        # add all Queues to list
        self.Queues.append(independentQueue(heuristicConcrete()))
        self.Queues.append(independentQueue(heuristicMud()))
        self.Queues.append(independentQueue(heuristicSand()))
        self.Queues.append(independentQueue(heuristicWater()))
        self.Queues.append(independentQueue(heuristicTrees()))
        self.startNode = start
        self.endNode = end
        self.foundGoal = False
        # Need world object from GUI
        self.map = map
        # scheduler goes here
        self.scheduler = self.getScheduler(scheduler)
        pass

    # Returns a scheduler corresponding to input. Made so we can add more in the future
    def getScheduler(self, scheduler):
        if scheduler == "DTS":
            return DTS_SchedulerIndependent(self.map, self.Queues)

        return DTS_SchedulerIndependent(self.map, self.Queues)

    def run(self):
        pass
        #Begin with Scheduler Choose Queue Method
        #Run the expansion
        #use best heuristic value in the updateMetaMethod which will update any parameters that determine the queue returned in ChooseQueue
        #run until goal is found
        #return the path

class independentQueue:
    def __init__(self, heuristic):
        self.Frontier = []
        self.Explored = []
        self.Heuristic = heuristic