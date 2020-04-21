from SchedulersIndependent import *
import numpy as np
#This is algorithm 4 from the meta A* paper
#it can be adapted to be A* by passing in the best heuristic as totalcost + Heuristic rather than just Heuristic
class DTS_SchedulerIndependent(SchedulerIndependent):
    def __init__(self, map, Queues, verbose = False):
        super().__init__(map, Queues)
        # DTS will take this Queue list and create its own queue list with alpha, beta and probability distribution values
        self.DTSQueues = []
        self.verbose = verbose
        queueKeys = Queues.keys()
        for key in queueKeys:
            self.DTSQueues.append(DTSQueueObject(key, Queues[key], 10))
        #############################################################

        #returns the key of the chosen algorithm
    def chooseQueue(self):
        #Calculates Beta Prob distribution for each queue and picks the queue with the highest probability of success
        bestQueue = self.DTSQueues[0].key
        bestProb = self.DTSQueues[0].betaDistribution()
        if(self.verbose):
            print("Best Probability", bestProb)
        for queue in range(1, len(self.DTSQueues)):
            #prob = self.DTSQueues[queue].betaDistribution()
            prob = self.DTSQueues[queue].betaDistribution2(0.75) # find proability of 75% of getting rewarded again
            if(self.verbose):
                print("Probability:", prob)
            if prob>bestProb:
                bestProb = prob
                bestQueue = self.DTSQueues[queue].key
        return bestQueue
    

    def UpdateMetaMethod(self, key, bestHeuristic):
        #Explore all neighbors of top node in the chosen queue, if a neighbor has node with a better heuristic than the previous best heuristic
        #reward this Queue, if not punish this queue
        #apply the memory function as described in the MetaA* paper pseduo code
        queueToUpdateIndex = 0
        for queue in range(0, len(self.DTSQueues)):
            if key == self.DTSQueues[queue].key:
                queueToUpdateIndex = queue
                if(self.verbose):
                    print("Updated" + key)
        if bestHeuristic<self.DTSQueues[queueToUpdateIndex].bestH:
            self.DTSQueues[queueToUpdateIndex].bestH = bestHeuristic
            if(self.verbose):
                print("Rewarded Queue")
            self.DTSQueues[queueToUpdateIndex].rewardQueue()
        else:
            self.DTSQueues[queueToUpdateIndex].punishQueue()
            if(self.verbose):
                print("Punished Queue")

    # clear queues probabilites by creating a new queObject
    # def clearQueues(self):
    #     del self.DTSQueues[:]
    #     for key in self.queueKeys:
    #         self.DTSQueues.append(DTSQueueObject(key, self.Queues[key], 10))
    #     pass

class DTSQueueObject:
    def __init__(self, key, Queue, C):
        self.key = key
        self.C = C
        self.alpha = 1
        self.beta = 1
        self.bestH = 10000000000
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
        for x in range(5, 100, 5):# I only check 20 points along the curve bc i figure that is enough
            xnew = float(x)
            xnew*=0.01 #makes it a decimal
            dist = np.power(xnew, self.alpha-1)*np.power((1-xnew), self.beta-1)#beta distribution function
            if dist>=highestDist:
                highestDist = dist
                xval = xnew
        return xval

    #this seems to work better than betaDistribution ^^^
    def betaDistribution2(self, xnew): # returns the highest probability density at a given point
        return np.power(xnew, self.alpha-1)*np.power((1-xnew), self.beta-1)#beta distribution function
