#!/usr/bin/env python3

from RBE550Project.srv import GetWaypoint
from geometry_msgs.msg import Point, Twist
from Node import *
import pickle
import rospy
import sys
import sys
from pathlib import Path

class PathServer():
    def __init__(self):
        rospy.init_node('path_server')
        print("Initializing server")
        
        self.path = []
        #self.path.reverse()
        self.makePath()
        # Creating Server 
        # GetWaypoint is an object with attribute Waypoint
        self.server = rospy.Service('waypoint_service', GetWaypoint, self.path_server)
        print("Server is created")
        rospy.spin()

    def makePath(self):
        # This method will take the pickle file which containes the map of nodes from the simulation map
        # unpack it, into a list called path, and pass this path to makePoints from nodes
        home_path = Path.home()
        with open (str(home_path) + "/catkin_ws/src/RBE550Project/src/FinalPath.pkl","rb") as fileOpener:
            path = pickle.load(fileOpener)
        print("opened pickle path file")
        self.path = self.makePointsFromNodes(path)

    def makePointsFromNodes(self, path):
        # This method takes the nodes given in the path list, and turns them into a list of "Point" objects
        # which can be passed to the turtlebot controller
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
    # #Instantiates an object of type PathServer 
    ps = PathServer() # needs waypoint list in parameter, empty for testing
