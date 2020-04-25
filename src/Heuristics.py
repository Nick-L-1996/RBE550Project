import numpy as np
class Heuristic:
    def __init__(self):
        self.type = "Concrete"
        pass
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        return 0

#TODO: IF THE NODE TO BE EXPLORED IS A TREE, COST IS HUGE, BUT IT CAN STILL GO THROUGH
class HeuristicMud(Heuristic): #High cost for making turns
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):

        #calculate if the current direction of the robot is the same as the movement to explore a frontier node
        directionVector1 = np.array(CurrentNode.RobotDirection)
        directionVector2 = np.array([NodeToBeExplored.xcoord-CurrentNode.xcoord, NodeToBeExplored.ycoord-CurrentNode.ycoord])
        VectorInSameDirection = np.dot(directionVector1, directionVector2) # if dot product is 1 then the robot is staying straight
        cost = np.sqrt(
            np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        if VectorInSameDirection == 1.0:
            if(NodeToBeExplored.Environment == "Mud"):
                cost*=.5 # Because concrete has a greater coefficient of friction than mud (sliding)
        else:
            cost*= 2  #This is arbitrary change if needed
        
        if(NodeToBeExplored.Environment == "Trees"):
            cost += 10000000000000
        return cost

class HeuristicConcrete(Heuristic):
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode): #current node not used
        cost = np.sqrt(
            np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        """
        If we transition from anything thats not concrete, we're heavier, and therefore slower:
        increase cost by some amount (multiple) 
        """
        if(CurrentNode.Environment != "Concrete" and NodeToBeExplored.Environment == "Concrete"):
            # print("Going from not concrete to concrete")
            cost*=1.5

        if(NodeToBeExplored.Environment == "Trees"):
            cost += 10000000000000

        return cost

class HeuristicWater(Heuristic): #slows you down and damages you if stay in for 3 grid cells
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        #water slows you down a little bit
        cost = 1.1*np.sqrt(np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        try:
            parentterrain = CurrentNode.parent.Environment
        except:
            parentterrain = "None"
        """
        If we transition from mud or sand, the first node in water has a lower cost, because we are getting cleaned
        so, multiply by .7 (lower cost)
        """
        if(CurrentNode.Environment == "Mud" or CurrentNode.Environment == "Sand"):
            if(NodeToBeExplored.Environment == "Water"):
                cost*= .7


        #check if three waters in a row, damages car
        if NodeToBeExplored.Environment == "Water" and CurrentNode.Environment == "Water" and parentterrain == "Water":
            cost += 100000 #arbitrary value
        #print ("WATER")

        if(NodeToBeExplored.Environment == "Trees"):
            cost += 10000000000000

        return cost

class HeuristicTrees(Heuristic):
     def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        # if NodeToBeExplored.Environment == "Trees":
        cost = 10000000000000 #Arbitrary
        """
        if the next node is a tree cost*=2
        """
        if(NodeToBeExplored.Environment == "Trees"):
            cost *= 2
        return cost

class HeuristicSand(Heuristic):
    def getHeuristic(self, CurrentNode, NodeToBeExplored, EndNode):
        """
        easier to turn
        """
         #calculate if the current direction of the robot is the same as the movement to explore a frontier node
        directionVector1 = np.array(CurrentNode.RobotDirection)
        directionVector2 = np.array([NodeToBeExplored.xcoord-CurrentNode.xcoord, NodeToBeExplored.ycoord-CurrentNode.ycoord])
        VectorInSameDirection = np.dot(directionVector1, directionVector2) # if dot product is 1 then the robot is staying straight
        #sand slows you down a little bit more than water
        cost = 1.3*np.sqrt(
            np.power(EndNode.xcoord - NodeToBeExplored.xcoord, 2) + np.power(EndNode.ycoord - NodeToBeExplored.ycoord, 2))
        if VectorInSameDirection == 1.0:
            cost*= 2
        else:
            if(NodeToBeExplored.Environment == "Sand"):
                cost*= .7  #This allows us easier to turn
        if(NodeToBeExplored.Environment == "Trees"):
            cost += 10000000000000
        return cost