class MetaAStar:
    def __init__(self):
        pass

    # procedure INITIALIZEMETAMETHOD()
    #for i ∈ {1, 2, ..., n} do
        #Gm[i] ← 0
        #Hm[i] ← hi(sstart) / (∆hi)max
        # Fm[i] ← Hm[i]
    # procedure UPDATEMETAMETHOD(i)
        #Gm[i] ← Gm[i] + 1
        #Hm[i] ← (mins∈OPENi hi(s)) / (∆hi)max
        # Fm[i] ← Gm[i] + wm ∗ Hm[i]
    #procedure CHOOSEQUEUE()
    # return arG mini Fm[i]