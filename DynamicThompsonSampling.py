from SchedulersIndependent import *
import numpy as np
#This is algorithm 4 from the meta A* paper
#it can be adapted to be A* by passing in the best heuristic as totalcost + Heuristic rather than just Heuristic
class DTS_SchedulerIndependent(SchedulerIndependent):
    def __init__(self, map, Queues, startNode, endNode):
        super().__init__(map, Queues)
        # DTS will take this Queue list and create its own queue list with alpha, beta and probability distribution values
        self.DTSQueues = []
        queueKeys = Queues.Keys()
        for key in queueKeys:
            self.DTSQueues.append(DTSQueueObject(Queues[key], 10, startNode, endNode))


        #############################################################
    def chooseQueue(self):
        #Calculates Beta Prob distribution for each queue and picks the queue with the highest probability of success
        bestQueue = self.DTSQueues[0].Queue
        bestProb = self.DTSQueues[0].betaDistribution()
        for queue in range(1, len(self.DTSQueues)):
            prob = self.DTSQueues[queue].betaDistribution()
            if prob>bestProb:
                bestProb = prob
                bestQueue = self.DTSQueues[queue].Queue
        return bestQueue

    def UpdateMetaMethod(self, Queue, bestHeuristic):
        #Explore all neighbors of top node in the chosen queue, if a neighbor has node with a better heuristic than the previous best heuristic
        #of all the seperate Queues, reward this Queue, if not punish this queue
        #apply the memory function as described in the MetaA* paper pseduo code
        queueToUpdateIndex = 0
        for queue in range(0, len(self.DTSQueues)):
            if Queue.IDNum == self.DTSQueues[queue].Queue.IDNUM:
                queueToUpdateIndex = queue
        if bestHeuristic<self.DTSQueues[queueToUpdateIndex].bestH:
            self.DTSQueues[queueToUpdateIndex].bestH = bestHeuristic
            self.DTSQueues[queueToUpdateIndex].rewardQueue()
        else:
            self.DTSQueues[queueToUpdateIndex].punishQueue()


class DTSQueueObject:
    def __init__(self, Queue, C, startNode, EndNode):
        self.C = C
        self.alpha = 1
        self.beta = 1
        self.bestH = Queue.Heuristic.getHeuristic(startNode, startNode, EndNode) #estimate from start to goal
        self.Queue = Queue

    def rewardQueue(self):
        self.alpha+=1
        self.limitMem()
    def punishQueue(self):
        self.beta+=1
        self.limitMem()

    def limitMem(self):
        if (self.alpha + self.beta) > self.C:
            self.alpha *= self.C / (1 + self.C)
            self.beta *= self.C / (1 + self.C)

        #https://towardsdatascience.com/beta-distribution-intuition-examples-and-derivation-cf00f4db57af
    def betaDistribution(self):#returns the point where the beta distribution is highest
        highestDist = 0
        xval = 0
        for x in np.range(0, 1, .05):# I only check 20 points along the curve bc i figure that is enough
            dist = np.power(x, self.alpha-1)*np.power((1-x), self.beta-1)#beta distribution function
            if dist>highestDist:
                highestDist = dist
                xval = x
        return xval

