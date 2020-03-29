import math
from Heuristics import *

class SchedulerIndependent:
    # scheduler needs to know what the world looks like
    def __init__(self, map, Queues):
        self.map = map
        self.Queues = Queues #DTS will take this Queue list and create its own queue list with alpha, beta and probability distribution values


    def chooseQueue(self): #Todo function name from metaA* paper psuedo code
        pass

    def UpdateMetaMethod(self):#Todo function name from metaA* paper psuedo code
        pass


