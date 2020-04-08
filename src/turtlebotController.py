#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from RBE550Project.srv import GetWaypoint
from geometry_msgs.msg import Point, Twist
import math
import sys

class TurtleBot():
    def __init__(self):
        print("Waiting for server...")
        rospy.wait_for_service('waypoint_service')
        print("Connected to server")

        self.nextWaypointProxy = rospy.ServiceProxy('waypoint_service', GetWaypoint)
        print("Turtlebot waypoint client created")

        # Initial position of TB, newOdom updates this anyway
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        # The distance to goal, which will be updated in update goal 
        self.distanceToGoal = 0
        
        # flags to tell whether or not we have reached a goal
        self.rotatedFully = False
        self.reachedGoal = False
        self.newGoalRecieved =  False

        #The heading to the new goal
        self.headingToGoal = 0

        # Initialize this goal
        rospy.init_node("turtlebot_controller")

        # Subscriber for odometry and publisher for twist message
        self.sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        # Run at 10Hz
        self.r = rospy.Rate(10)

        # Starting point of the TB before it goes to next goal, updated in update goal
        self.start = Point()
        self.start.x = 0
        self.start.y = 0
        
        # Next waypoint to go to
        self.nextWaypoint = Point()
        self.nextWaypoint.x = 0
        self.nextWaypoint.y = 0

    #Recieves new odom information and updates robot pose
    def newOdom(self, msg):
        #Get the X and Y of the robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        #Get the orientation as a quaternion
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    # Rotates to a given heading 
    def rotateTo(self, angle):
        goalHeading = angle
        rotateSpeed = .15

        #Wrap around 
        if goalHeading > math.pi:
            goalHeading = goalHeading - 2*math.pi
        elif goalHeading < -1*math.pi:
            goalHeading = goalHeading + 2*math.pi

        # find the error between our current theta and our desired heading
        error = goalHeading - self.theta
        # if we haven't rotated fully
        if abs(error) > .05:
            # Determines which way to turn
            if error < 0: # was angle	    
                self.publishTwist(0, -1*rotateSpeed)
                self.rotatedFully = False
            else:
                self.publishTwist(0, rotateSpeed)
                self.rotatedFully = False
        # if we have reached the goal, stop and set the flag accordingly
        else:
            self.publishTwist(0,0)
            self.rotatedFully = True

    # This method publishes a twist message with a desired linear and angular velocity
    def publishTwist(self, linearVelocity, angularVelocity):
        try:
            msg = Twist()
            msg.linear.x = linearVelocity #set linear and angular velocities 
            msg.angular.z = angularVelocity
            self.pub.publish(msg) #publish
        except:
            print ("Failed to publish")
    
    # Once the robot has rotated to the proper heading, this method will get it to the goal point
    def goToGoal(self, newGoal):
        speed = .15 
        # calculates the distance moved since a new goal was set
        distanceMoved = (math.sqrt((self.start.y-self.y)**2 + (self.start.x-self.x)**2))
        # if we haven't moved the correct distance
        if(abs(distanceMoved) < self.distanceToGoal): 
            self.publishTwist(speed,0) #publish a twist message to move forward 
            return False
        # if we have reached the correct distance, stop
        else:
            self.publishTwist(0,0)
            return True
            
    def updateGoal(self, newGoal):
        # This method is called when we request a new waypoint. 
        # It updates our start point (current point), distance to the goal, and heading to the goal
        self.start.x = self.x
        self.start.y = self.y
        self.distanceToGoal = math.sqrt((newGoal.y-self.start.y)**2 + (newGoal.x-self.start.x)**2)
        self.headingToGoal = math.atan2((newGoal.y-self.y),(newGoal.x-self.x))
        print("Heading", self.headingToGoal)
        print("Distance to goal", self.distanceToGoal)


    def getWaypointClient(self):
        try:
            # get the next waypoint from the server. Waypoint is a Point message in the response message
            self.nextWaypoint = self.nextWaypointProxy().Waypoint
            print("Next waypoint", self.nextWaypoint.x,self.nextWaypoint.y)
            #check if there's no more goals. In server if there's no more we send max int size
            if(self.nextWaypoint.x > 1000000):
                print("No more goals")
            else:
                # set the new goal as the next received waypoint
                self.updateGoal(self.nextWaypoint)
                # reset rotate and drive straight flags
                self.rotatedFully = False
                self.reachedGoal = False
        except rospy.ServiceException as e:
            print ("Service call failed: ")

TB = TurtleBot()
while not rospy.is_shutdown():
    if(not TB.rotatedFully): # if not fully rotated, then rotate
        TB.rotateTo(TB.headingToGoal)
    elif(not TB.reachedGoal): # if we havent reached the goal, keep driving
        # drive function returns boolean true if turtlebot reaches goal otherwise false
        TB.reachedGoal = TB.goToGoal(TB.distanceToGoal)
    else:
        print("Requesting new waypoint")
        #request new waypoint from service
        TB.getWaypointClient()
    #rospy sleep
    TB.r.sleep()
