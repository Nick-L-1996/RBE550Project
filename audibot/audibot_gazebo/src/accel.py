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


def getAccelCommand():
	rospy.init_node('carMover', anonymous = True)
	rospy.Subscriber("throttlePercentage", Float64, moveCar)
	#Put a rospy.spin() if we don't want it to maintain this speed
	
#This is the callback
def moveCar(data): 
    print(type(data))
    throttlePub = rospy.Publisher('throttle_cmd', Float64, queue_size=10)
    throttlePub.publish(data)


if __name__ == '__main__':
    while not rospy.is_shutdown():
        try:
            getAccelCommand()
        except rospy.ROSInterruptException:
            pass




