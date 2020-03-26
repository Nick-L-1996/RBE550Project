class SMHAStar:
    def __init__(self):
        self.visited = []
        self.unVisited = []
        self.finalPath = []
        self.startNode = None
        self.endNode = None
        self.foundGoal = False
        # Need world or graph object from GUI
        self.graph = None
        #scheduler goes here
        self.scheduler = None
        pass


    def run(self):
        """
        Pseudo Code (according to the Paper):
            remove s from open list
            v(s) = g(s)
            for all s' Succ do
            if s'was never generated then
            g(s') = ∞; bp(s') = null
            v(s') = ∞.
            if g(s') > g(s) + c(s, s') then
            g(s') = g(s) + c(s, s'); bp(s') = s

            if s' ∈ / CLOSEDanchor then
            Insert / Update s' in OPEN0 with Key(s', 0)
            if s' ∈ / CLOSEDinad then

            for i = 1, 2, ..., n do
                if Key(s', i) ≤ w2 ∗ Key(s', 0) then
                Insert / Update s' in OPENi with Key(s', i)

         @RICH I didn't REALLY follow the pseudo-code itself (as written) from the paper, but this is how I understood it:
             1) put start node in unvisited and pop off the unvisited list
             2) current node = popped off node
             3) at current node, have scheduler determine next best node to go
             4) add current node to visited
             5) add next node to the unvisited list, and sort the list by total cost (h + g)
             6) Go to step 2
        """
        pass
#
#     def Key(selfs, i):
#         pass
#         #return g(s)+w1*hi(s)