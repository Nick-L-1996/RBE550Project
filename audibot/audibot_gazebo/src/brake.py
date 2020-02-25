#!/usr/bin/env python
# license removed for brevity
#This is a service server
import rospy
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from gazebo_msgs.srv import GetModelState, GetModelStateRequest

#rospy.wait_for_service ('/gazebo/get_model_state')
#get_model_srv = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
#model = GetModelStateRequest()
#model.model_name='audibot'


def getBrakeCommand():
	rospy.init_node('carBraker', anonymous = True)
	rospy.Subscriber("brake", Float64, brakeCar)
	#Put a rospy.spin() if we don't want it to maintain brake
	
#This is the callback
#For right now, we'll just send 200 nm, but we'll take in a float and publish that
def brakeCar(data): 
    accelPub = rospy.Publisher('throttlePercentage', Float64, queue_size=10)
    brakePub = rospy.Publisher('brake_cmd', Float64, queue_size=10)
    brakePub.publish(data)
    #brakePub.publish(200)
    accelPub.publish(0)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            getBrakeCommand()
        except rospy.ROSInterruptException:
            pass




