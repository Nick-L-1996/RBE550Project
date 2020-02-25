#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from std_msgs.msg import UInt8

class Car():
	"""
          This constructor sets all of the publishers and subscribers. Since the car does not normally maintain throttle
	  and brake, we have the while loop constantly publish the current thottle and brake commands, which get updated
          via a callback. 
	"""
	def __init__(self):
	    rospy.init_node('lowLevelCarController', anonymous = True)
	    self.accelSub = rospy.Subscriber('throttle', Float64, self.setAccelFlag)
	    self.brakeSub = rospy.Subscriber("brake", Float64, self.setBrakeFlag)
            self.steerSub = rospy.Subscriber("steer", Float64, self.setSteerAngle)
	    self.gearSub = rospy.Subscriber("gear", UInt8, self.setGear)


	    self.brakePub = rospy.Publisher('brake_cmd', Float64, queue_size = 1)
 	    self.throttlePub = rospy.Publisher('throttle_cmd', Float64, queue_size = 1)
	    self.steerPub = rospy.Publisher('steering_cmd', Float64, queue_size = 1)
	    self.gearPub = rospy.Publisher('gear_cmd', UInt8, queue_size = 1)

	    self.throttleCmd = 0
            self.brakeCmd = 0
	    self.steeringCmd = 0
	    self.stopping = False

	def setAccelFlag(self, data):
            self.throttleCmd = data
            self.stopping = False

        def setBrakeFlag(self, data):
	    self.brakeCmd = data
	    self.stopping = True

	#Convert the given data to radians and publish it to the car
	def setSteerAngle(self, data):
	    self.steeringCmd = (((data.data*3.14)/180)*17.3)
	    self.steerPub.publish(self.steeringCmd)

	#Set the gear of the car, 1 is reverse, drive is 0.
        def setGear(self, data):
            self.gearCmd = data
            self.gearPub.publish(self.gearCmd)

	#Also set the car's throttle percentage  to 0
        def brakeCar(self, data): 
            self.throttleCmd = 0
	    self.brakePub.publish(self.brakeCmd)

        def moveCar(self, data):
            self.throttlePub.publish(data)



c = Car()
while not rospy.is_shutdown():
    c.moveCar(c.throttleCmd)
    if(c.stopping):
    	c.brakeCar(c.brakeCmd)

