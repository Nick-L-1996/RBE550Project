from SchedulersIndependent import *

#This is algorithm 4 from the meta A* paper
class DTS_SchedulerIndependent(SchedulerIndependent):
    def __init__(self, map, Queues):
        super().__init__(map, Queues)

        # DTS will take this Queue list and create its own queue list with alpha, beta and probability distribution values
        #TODO InitializeMetaMethod
        # For each different Queue
        #make a struct with the best heursitic value (top of the explored queue stack)
        #initialize alpha to 1
        #initialize beta to 1
        #[best heuristic, alpha, beta, Queue Object]

        #############################################################
    def chooseQueue(self):  # Todo function name from metaA* paper psuedo code
        #Calculates Beta Prob distribution for each queue and picks the queue with the highest probability of success
        pass

    def UpdateMetaMethod(self):  # Todo function name from metaA* paper psuedo code
        #Explore all neighbors of top node in the chosen queue, if a neighbor has node with a better heuristic than the previous best heuristic
        #of all the seperate Queues, reward this Queue, if not punish this queue
        #apply the memory function as described in the MetaA* paper pseduo code
        pass