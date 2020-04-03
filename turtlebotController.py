#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, Twist
import math

class TurtleBot():
    def __init__(self):
        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.distance = 0
        self.wayPoints = []
        rospy.init_node("turtlebot_controller")

        self.sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        r = rospy.Rate(4)
        self.start = Point()
        self.start.x = 0
        self.start.y = 0

        self.goal = Point()
        self.goal.x = 0
        self.goal.y = 0
        goal1 = Point()
        x = 0
        y = 4

        goal2 = Point()
        goal2.x = 4 
        goal2.y = 4

        self.wayPoints = [goal1, goal2]

    #Recieves new odom information and updates robot pose
    def newOdom(self, msg):
        #Get the X and Y of the robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        #Get the orientation as a quaternion
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def rotate(self, angle):
        #Goal heading is current theta + angle to rotate
        goalHeading = self.theta + angle
        print("Goal heading", goalHeading)
        if goalHeading > math.pi:
            goalHeading = goalHeading - 2*math.pi
        elif goalHeading < -1*math.pi:
            goalHeading = goalHeading + 2*math.pi
        while abs(goalHeading - self.theta) > .01:
            if angle < 0:	    
                self.publishTwist(0, -.1)
            else:
                self.publishTwist(0, .1)
        self.publishTwist(0,0)

    def publishTwist(self, linearVelocity, angularVelocity):
        try:
            msg = Twist()
            msg.linear.x = linearVelocity #set linear and angular velocities 
            msg.angular.z = angularVelocity
            self.pub.publish(msg) #publish
            #print "Published!"
        except:
            print "Failed to publish"

    def goToGoal(self, newGoal):
        speed = .1
        distanceMoved = (math.sqrt((self.start.y-self.y)**2 + (self.start.x-self.x)**2))
        print("I've moved", distanceMoved)
        if(abs(distanceMoved) < self.distance): #check to see if distance has been reached 
            self.publishTwist(speed,0) #publish a twist message to move forward 
            return False
        else:
            self.publishTwist(0,0)
            return True
            
    def updateGoal(self, newGoal):
        if(newGoal.y <= self.y):
            heading = math.atan2((newGoal.y-self.y),(newGoal.x-self.x)) - math.pi/2
        else:
            heading = math.atan2((newGoal.y-self.y),(newGoal.x-self.x))
        print("Heading", heading)
        self.rotate(heading)
        self.start.x = self.x
        self.start.y = self.y
        self.goal.x = newGoal.x
        self.goal.y = newGoal.y
        self.distance = math.sqrt((self.goal.y-self.start.y)**2 + (self.goal.x-self.start.x)**2)
        print("Distance to goal", self.distance)





### Recieve new XY from Map
### Calculate heading
### Rotate to heading
### Update start.x and start.y, goal.x and goal.y
### Drive until reached goal

TB = TurtleBot()
reachedGoal = False
for goal in TB.wayPoints:
    print("=======", "New Goal")
    TB.updateGoal(goal)
    print("goalx", TB.goal.x, "goaly", TB.goal.y)
    # goToGoal as long as you havent reached goal 
    while(not reachedGoal):
        reachedGoal = TB.goToGoal(goal)
        print("X:", TB.x, "Y:", TB.y)
    reachedGoal = False
    
