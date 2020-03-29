from SchedulersShared import *
class RR_SchedulerShared(SchedulerShared):
    def __init__(self, map):
        super().__init__(map)
        pass

    #OVERLOADED FUNCTION
    def updateUnvisitedQueue(self, currentNode, visited, unvisited, endNode):
        
        # get neighbors of current node
        neighbors = self.getNodeNeighbors(currentNode)
        
        print("Current Node:", currentNode.row, currentNode.column)
        # get unexplored neighbors
        # for every neighbor find the lowest value across all heuristics
        for neighbor in neighbors:
            #print("Neighbor Node:", neighbor.row, neighbor.column)
            
            # if this neighbor was marked as visited
            if(neighbor in visited):
               #print("Visited", neighbor.row, neighbor.column)
               continue
                
            terrain_type = neighbor.Environment
            #if(terrain_type != "Concrete"):
            #    print("Environment:", terrain_type)    

            # get the edge cost between the current node and neighbor use same key as heuristic dict because the strings are the same.
            edgeCost = currentNode.getNeighborEdgeCost(neighbor)
            #print("Edge Cost:", edgeCost)

            chosenHeuristic = None
            # check a neighbor with all heuristics
            # for each key in the heuristic getters.
            for heuristic_type in self.heuristicGetters.keys():
                
                # get the hueristic value between the current node and the neighbor using specific hueristic and end goal
                heuristicCost = self.getHeuristicValue(heuristic_type, currentNode, neighbor, endNode)
                #print("Heuristic Cost:", heuristic_type, heuristicCost)

                # calculate the would be cost to travel from current node to neighbor
                tempCost = edgeCost + heuristicCost

                # if new temp path cost is cheaper than another current path cost to get to neighbor, update neighbor cost
                if tempCost < neighbor.totalCost:
                    chosenHeuristic = heuristic_type
                    neighbor.totalCost = tempCost
                    # we need to set the new parent if the cost has changed
                    neighbor.parent = currentNode
                    #print("Neighbor Cost: ", neighbor.totalCost)

            # if neighbor is not in the unvisited list, add it to unvisited
            if (neighbor not in unvisited):
                print ("Added Neighbor:", neighbor.row, neighbor.column, chosenHeuristic, neighbor.Environment, edgeCost)
                unvisited.append(neighbor)
    

        # sort nodes in unvisited by their cost
        unvisited.sort(key=lambda x: x.totalCost)

        # return updated unvisited (open list)
        return unvisited