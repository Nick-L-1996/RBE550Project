from DynamicThompsonSampling import *
from Heuristics import *

#changed name from IMHA* bc DTS is a greedy best first search algorithm with multiple prioritized Queues as written in the paper
# We can easily make it A* though
class IndependentQueueAlgorithm:
    # self.Algorithm_RR = IMHAStar(self.Map, self.StartPoint, self.EndPoint, scheduler = "DTS")
    def __init__(self, map, start, end, scheduler="DTSGreedy"):
        self.Queues = {}
        # add all Queues to list
        self.Queues["Concrete"] = independentQueue(HeuristicConcrete(), "Concrete")
        self.Queues["Mud"] = independentQueue(HeuristicMud(), "Mud")
        self.Queues["Sand"] = independentQueue(HeuristicSand(), "Sand")
        self.Queues["Water"] = independentQueue(HeuristicWater(), "Water")
        self.Queues["Trees"] = independentQueue(HeuristicTrees(), "Trees")

        self.startNode = start
        self.endNode = end
        self.foundGoal = False
        # Need world object from GUI
        self.map = map
        # scheduler goes here
        self.scheduler, self.isGreedy = self.getScheduler(scheduler)
        #the isGreedy object determines if the priority queue considers G + H or just H

    # Returns a scheduler corresponding to input. Made so we can add more in the future
    def getScheduler(self, scheduler):
        if scheduler == "DTSGreedy":
            return DTS_SchedulerIndependent(self.map, self.Queues, self.startNode, self.endNode), True
        elif scheduler =="DTSA*":
            return DTS_SchedulerIndependent(self.map, self.Queues, self.startNode, self.endNode), False
        return DTS_SchedulerIndependent(self.map, self.Queues, self.startNode, self.endNode), True


    def run(self):
        chosenQueue = self.Queues[self.scheduler.chooseQueue()]

        pass
        #Begin with Scheduler Choose Queue Method
        #Run the expansion
        #use best heuristic value in the updateMetaMethod which will update any parameters that determine the queue returned in ChooseQueue
        #run until goal is found
        #return the path

    #def expand(self, queue):

class independentQueue:
    def __init__(self, heuristic, ID):
        self.ID = ID
        self.Frontier = []
        self.Explored = []
        self.Heuristic = heuristic