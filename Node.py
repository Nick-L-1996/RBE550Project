class Node:
    def __init__(self, x, y, Env):
        self.xcoord = x
        self.ycoord = y
        self.parent = None
        self.Distance = 10000000000
        self.Heuristic = 10000000000
        self.Environment = Env