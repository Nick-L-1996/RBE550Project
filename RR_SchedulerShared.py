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
            print("Neighbor Node:", neighbor.row, neighbor.column)
            
            # if this neighbor was marked as visited
            if(neighbor in visited):
                continue

            # check a neighbor with all heuristics
            for heuristic_type in self.heuristicGetters.keys():
                
                # get the edge cost between the current node and neighbor
                edgeCost = self.getEdgeCost(currentNode, neighbor)
                print("Edge Cost:", edgeCost)

                # get the hueristic value between the current node and the neighbor using specific hueristic and end goal
                heuristicCost = self.getHeuristicValue(heuristic_type, currentNode, neighbor, endNode)
                print("Heuristic Cost:", heuristic_type, heuristicCost)

                # calculate the would be cost to travel from current node to neighbor
                tempCost = edgeCost + heuristicCost

                # if new temp path cost is cheaper than another current path cost to get to neighbor, update neighbor cost
                if tempCost < neighbor.totalCost:
                    neighbor.totalCost = tempCost
                    # we need to set the new parent if the cost has changed
                    neighbor.parent = currentNode

                # if neighbor is not in the unvisited list, add it to unvisited
                if (neighbor not in unvisited):
                    unvisited.append(neighbor)

        # sort nodes in unvisited by their cost
        unvisited.sort(key=lambda x: x.totalCost)

        # return updated unvisited (open list)
        return unvisited