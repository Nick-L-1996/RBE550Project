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
        self.rotatedFully = False
        self.reachedGoal = False
        self.newGoalRecieved =  False
        rospy.init_node("turtlebot_controller")

        self.sub = rospy.Subscriber("/odom", Odometry, self.newOdom)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)

        self.r = rospy.Rate(10)
        self.start = Point()
        self.start.x = 0
        self.start.y = 0

        self.goal = Point()
        self.goal.x = 0
        self.goal.y = 0

        # goal1 = Point()
        # goal1.x = -2
        # goal1.y = -4

        # goal2 = Point()
        # goal2.x = 4 
        # goal2.y = 4

        #self.wayPoints = [goal1, goal2]

    #Recieves new odom information and updates robot pose
    def newOdom(self, msg):
        #Get the X and Y of the robot
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        #Get the orientation as a quaternion
        rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])

    def rotateTo(self, angle):
        #Goal heading is current theta + angle to rotate
        goalHeading = angle
        rotateSpeed = .15
        #Wrap around 
        if goalHeading > math.pi:
            goalHeading = goalHeading - 2*math.pi
        elif goalHeading < -1*math.pi:
            goalHeading = goalHeading + 2*math.pi
        #While we haven't rotated fully
        error = goalHeading - self.theta
        if abs(error) > .01:
            if angle < 0:	    
                self.publishTwist(0, -1*rotateSpeed)
                self.rotatedFully = False
            else:
                self.publishTwist(0, rotateSpeed)
                self.rotatedFully = False
        else:
            self.publishTwist(0,0)
            self.rotatedFully = True

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
        speed = .15 
        distanceMoved = (math.sqrt((self.start.y-self.y)**2 + (self.start.x-self.x)**2))
        if(abs(distanceMoved) < self.distance): #check to see if distance has been reached 
            self.publishTwist(speed,0) #publish a twist message to move forward 
            return False
        else:
            self.publishTwist(0,0)
            return True
            
    def updateGoal(self, newGoal):
        heading = math.atan2((newGoal.y-self.y),(newGoal.x-self.x))
        print("Heading", heading)
        self.rotateTo(heading)
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

goal1 = Point()
goal1.x = -2
goal1.y = -4
TB = TurtleBot()
heading = math.atan2((goal1.y-TB.y),(goal1.x-TB.x))
print("Goal heading", heading)
TB.distance = math.sqrt((goal1.y-TB.start.y)**2 + (goal1.x-TB.start.x)**2)
while not rospy.is_shutdown():
    if(not TB.rotatedFully):
        TB.rotateTo(heading)
    elif(not TB.reachedGoal):
        TB.reachedGoal = TB.goToGoal(goal1)
    # else:
    #     TB.rotatedFully = False
    #     TB.reachedGoal = False
    TB.r.sleep()
