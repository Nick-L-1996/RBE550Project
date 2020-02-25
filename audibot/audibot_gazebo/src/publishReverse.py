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
    steerPub.publish(25)
    while not rospy.is_shutdown():
        accelPub.publish(-.2)
        rate.sleep()

if __name__ == '__main__':
    try:
        moveCar()
    except rospy.ROSInterruptException:
        pass
