#!/usr/bin/env python

from RBE550Project.srv import GetWaypoint
from geometry_msgs.msg import Point, Twist
from Node import *
import pickle
import rospy
import sys

class PathServer():
    def __init__(self, path_nodes):
        rospy.init_node('path_server')
        # Temporarily, we are making a point, for testing purposes. 
        # Eventually, we will integrate this into the Simulation App
        # TODO: Convert Node object into Point object!
        print("Initializing server")
        goal1 = Point()
        goal1.x = .1
        goal1.y = .1

        # path_nodes = pickle.load(pickle_file)

        self.path = self.fromNodesToPoints(path_nodes)
        
        # Creating Server 
        # GetWaypoint is an object with attribute Waypoint
        self.server = rospy.Service('waypoint_service', GetWaypoint, self.path_server)
        print("Server is created")
        rospy.spin()


    def fromNodesToPoints(self, path):
        path = []
        for node in path:
            waypoint = Point(node.xcoord, node.ycoord, 0)
            print("Added waypoint:", waypoint.x, waypoint.y)
            path.append(waypoint)
        return path

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
    with open ("FinalPath.pkl","rb") as fileOpener:
        path = pickle.load(fileOpener)
    for item in path:
        print(item)
    print("opened pickle path file")
    #Instantiates an object of type PathServer 
    ps = PathServer(path) # needs waypoint list in parameter, empty for testing
