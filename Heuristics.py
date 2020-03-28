import numpy as np
class heuristic:
    def __init__(self):
        pass
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        return 0

## TODO ERROR WHEN RUNNING
class heuristicMud(heuristic): #High cost for making turns
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        directionVector1 = np.array(CurrentNode.RobotDirection)
        directionVector2 = np.array([NodeToBeExplored.xcoord-CurrentNode.xcoord, NodeToBeExplored.ycoord-CurrentNode.ycoord])
        print(directionVector1.shape, directionVector2.shape)
        VectorInSameDirection = np.dot(directionVector1, directionVector2)
        cost = np.sqrt(
            np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        if VectorInSameDirection == 1.0:
            cost+=0
        else:
            cost+= 1000  #This is arbitrary change if needed
        return cost

class heuristicConcrete(heuristic):
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode): #current node not used
        cost = np.sqrt(
            np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        return cost

class heuristicWater(heuristic): #slows you down and damages you if stay in for 3 grid cells
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        cost = 2*np.sqrt(np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        try:
            parentterrain = CurrentNode.parent.Environment
        except:
            parentterrain = "None"
        #check if three waters in a row
        if NodeToBeExplored.Environment == "Water" and CurrentNode.Environment == "Water" and parentterrain == "Water":
            cost += 100000 #arbitrary value
        return cost

class heuristicTrees(heuristic):
     def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        if NodeToBeExplored.Environment == "Trees":
             cost = 10000000000000 #Arbitrary
        else:
            cost = np.sqrt(np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        return cost

class heuristicSand(heuristic):
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        cost = 2 * np.sqrt(np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        return cost