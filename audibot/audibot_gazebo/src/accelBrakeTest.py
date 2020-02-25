#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import Float64

def moveCar():
    accelPub = rospy.Publisher('throttle_cmd', Float64, queue_size=10)
    brakePub = rospy.Publisher('brake_cmd', Float64, queue_size = 10)
    steerPub = rospy.Publisher('steering_cmd', Float64, queue_size=10)
    rospy.init_node('carMover', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    increment = 0
    steerPub.publish(0)
    accel = True
    while not rospy.is_shutdown():
	if(accel == True):
	    increment = increment - .02
	    if(increment < -.2):
		accel = False
	    accelPub.publish(increment)
	else:
	    increment = increment + .02
	    if(increment >= 0):
	        accel = True
            brakePub.publish(200)

        rate.sleep()

if __name__ == '__main__':
    try:
        moveCar()
    except rospy.ROSInterruptException:
        pass
