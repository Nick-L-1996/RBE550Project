#!/usr/bin/env python3

from RBE550Project.srv import GetWaypoint
from geometry_msgs.msg import Point, Twist
import rospy
import sys

class PathServer():
    def __init__(self, path):
        rospy.init_node('path_server')
        # Temporarily, we are making a point, for testing purposes. 
        # Eventually, we will integrate this into the Simulation App
        # TODO: Convert Node object into Point object!
        
        goal1 = Point()
        goal1.x = .1
        goal1.y = .1
        
        self.path = self.makePointsFromNodes(path)
        #self.path.reverse()
        
        # Creating Server 
        # GetWaypoint is an object with attribute Waypoint
        self.server = rospy.Service('waypoint_service', GetWaypoint, self.path_server)
        print("Server is created")
        [print(i) for i in self.path]
        rospy.spin()

    def makePointsFromNodes(self, path):
        waypoints = []
        for node in path:
            waypoint = Point(node.xcoord, node.ycoord, 0)
            waypoints.append(waypoint)
            print(waypoint.x, waypoint.y)
        return waypoints

    # callback to send back a goal message when requested by client
    # we don't need data but when the callback is run it's given by default (it errors otherwise)
    def path_server(self, data):
        # if the path is not emtpy get next waypoint from the list of waypoints
        if(len(self.path) is not 0):
            waypoint = self.path.pop()
            print ("Returning waypoint:", waypoint.x, waypoint.y)
        else:
            # return max int if there's no more points. ROS expects a Point message returned in every case
            waypoint = Point(sys.maxsize, sys.maxsize, sys.maxsize)
            
        # returns an object of type GetWaypoint
        # to get the waypoint itself, the controller calls [service_variable_name].Waypoint
        return waypoint

if __name__ == "__main__":
    #Instantiates an object of type PathServer 
    ps = PathServer([]) # needs waypoint list in parameter, empty for testing
