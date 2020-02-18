class SMHAStar:
    def __init__(self):
        pass

    def runExpansion(self):
        pass
    #Takes parameters for the stack, explored, etc
    #Outputs changes to data structures and reports if path found
    #allows for multiThreaded app to run

    #sudo code from paper
        #remove s from open list
        #v(s) = g(s)
        #for all s' Succ do
        #if s'was never generated then
        # g(s') = ∞; bp(s') = null
        # v(s') = ∞.
        # if g(s') > g(s) + c(s, s') then
        # g(s') = g(s) + c(s, s'); bp(s') = s

        #if s' ∈ / CLOSEDanchor then
        # Insert / Update s' in OPEN0 with Key(s', 0)
        #if s' ∈ / CLOSEDinad then

        #for i = 1, 2, ..., n do
            #if Key(s', i) ≤ w2 ∗ Key(s', 0) then
            # Insert / Update s' in OPENi with Key(s', i)

    def Key(selfs, i):
        pass
        #return g(s)+w1*hi(s)