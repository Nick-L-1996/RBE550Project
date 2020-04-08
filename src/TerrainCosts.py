import numpy as np
import random
class TerrainCosts:
    def __init__(self):
        pass
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored):
        return 0

class EdgeCostMud(TerrainCosts): #High cost for making turns
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored):
        #calculate if the current direction of the robot is the same as the movement to explore a frontier node
        directionVector1 = np.array(CurrentNode.RobotDirection)
        directionVector2 = np.array([NodeToBeExplored.xcoord-CurrentNode.xcoord, NodeToBeExplored.ycoord-CurrentNode.ycoord])
        VectorInSameDirection = np.dot(directionVector1, directionVector2) # if dot product is 1 then the robot is staying straight
        cost = np.sqrt(
            np.power(NodeToBeExplored.xcoord - CurrentNode.xcoord, 2) + np.power(NodeToBeExplored.ycoord - CurrentNode.ycoord, 2))
        if VectorInSameDirection == 1.0:
            cost*=.5 # Because concrete has a greater coefficient of friction than mud
        else:
            cost*= 2  #This is arbitrary change if needed

        #cost+=random.randint(0, 20)# random cost to simulate the terrain not being perfect. Change values or disable if needed
        return cost

class EdgeCostConcrete(TerrainCosts):
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored): #current node not used
        cost = np.sqrt(
            np.power(NodeToBeExplored.xcoord - CurrentNode.xcoord, 2) + np.power(NodeToBeExplored.ycoord - CurrentNode.ycoord, 2))
        
        """
        If we transition from anything thats not concrete, we're heavier, and therefore slower:
        increase cost by some amount (multiple) 
        """
        if(CurrentNode.Environment != "Concrete" and NodeToBeExplored.Environment == "Concrete"):
            cost*=1.5

        if(NodeToBeExplored.Environment == "Tree"):
            cost += 10000000000000
        
        #cost += random.randint(0,20)  # random cost to simulate the terrain not being perfect. Change values or disable if needed
        #print ("Default concrete")
        return cost

class EdgeCostWater(TerrainCosts): #slows you down and damages you if stay in for 3 grid cells
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored):
        cost = 1.1*np.sqrt(
            np.power(NodeToBeExplored.xcoord - CurrentNode.xcoord, 2) + np.power(NodeToBeExplored.ycoord - CurrentNode.ycoord, 2))
        try:
            parentterrain = CurrentNode.parent.Environment
        except:
            parentterrain = "None"
        #check if three waters in a row
        
        """
        If we transition from mud or sand, the first node in water has a lower cost, because we are getting cleaned
        so, multiply cost by (0,1] (random)
        """
        if(CurrentNode.Environment == "Mud" or CurrentNode.Environment == "Sand"):
            if(NodeToBeExplored.Environment == "Water"):
                cost*= random.uniform(0,1)

        if NodeToBeExplored.Environment == "Water" and CurrentNode.Environment == "Water" and parentterrain == "Water":
            cost += 100000 #arbitrary value
        #cost += random.randint(0,20)  # random cost to simulate the terrain not being perfect. Change values or disable if needed
        return cost

class EdgeCostTrees(TerrainCosts):
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored):
        if NodeToBeExplored.Environment == "Trees":
             cost = 10000000000000 #Arbitrary
        else:
            cost = np.sqrt(
            np.power(NodeToBeExplored.xcoord - CurrentNode.xcoord, 2) + np.power(NodeToBeExplored.ycoord - CurrentNode.ycoord, 2))

        """
        if the next node is a tree cost*=2
        """
        if(NodeToBeExplored.Environment == "Tree"):
            cost *= 2
        
        #cost += random.randint(0,20)  # random cost to simulate the terrain not being perfect. Change values or disable if needed
        return cost

class EdgeCostSand(TerrainCosts):
    def getTerrainEdgeCost(self, CurrentNode, NodeToBeExplored):
        cost = 2 * np.sqrt(
            np.power(NodeToBeExplored.xcoord - CurrentNode.xcoord, 2) + np.power(NodeToBeExplored.ycoord - CurrentNode.ycoord, 2))
        directionVector1 = np.array(CurrentNode.RobotDirection)
        directionVector2 = np.array([NodeToBeExplored.xcoord-CurrentNode.xcoord, NodeToBeExplored.ycoord-CurrentNode.ycoord])
        VectorInSameDirection = np.dot(directionVector1, directionVector2) # if dot product is 1 then the robot is staying straight
        #cost += random.randint(0,20)  # random cost to simulate the terrain not being perfect. Change values or disable if needed
        if VectorInSameDirection == 1.0:
            cost*= 2
        else:
            if(NodeToBeExplored.Environment == "Sand"):
                cost*= .7  #This allows us easier to turn
        if(NodeToBeExplored.Environment == "Tree"):
            cost += 10000000000000

        return cost