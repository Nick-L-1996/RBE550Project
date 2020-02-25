#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def applyGearRatio(angle):
	return 17.3 * angle

def moveSteer():
    pub = rospy.Publisher('steering_cmd', Float64, queue_size=10)
    rospy.init_node('steerMover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    increment = 0
    turningRight = True
    while not rospy.is_shutdown():
	if(turningRight == True):
	    increment = increment + .02
	    if(increment > .7):
		turningRight = False
	else:
	    increment = increment - .02
	    if(increment < -.7):
	        turningRight = True

        steeringAngle = applyGearRatio(increment)
        pub.publish(steeringAngle)
        rate.sleep()

if __name__ == '__main__':
    try:
        moveSteer()
    except rospy.ROSInterruptException:
        pass
