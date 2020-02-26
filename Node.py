class Node:
    def __init__(self, x, y, row, column):
        self.xcoord = x
        self.ycoord = y
        self.row = row
        self.column = column
        self.parent = None
        self.Distance = 10000000000
        self.Heuristic = 10000000000
        self.Environment = None