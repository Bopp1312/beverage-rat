#!/usr/bin/python3

import rospy 
from sensor_msgs.msg import Joy 
from geometry_msgs.msg import Twist 

CONST_LINEAR_VELOCITY = 0.1 # m/s
CONST_ANGULAR_VELOCITY = 1.0  # rad/s

pub = rospy.Publisher('cmd/velocity', Twist,queue_size=1)

def callback(data):
	print(data.axes[0])
	twist = Twist()
	twist.linear.x = 0.5*data.axes[0]
	twist.angular.z = data.axes[1]*-0.5
	if (data.buttons[15] == True):
		twist.linear.x = CONST_LINEAR_VELOCITY
	if (data.buttons[13] == True):
		twist.angular.z = -CONST_ANGULAR_VELOCITY
	if (data.buttons[16] == True):
		twist.linear.x = -CONST_LINEAR_VELOCITY
	if (data.buttons[14] == True):
		twist.angular.z = CONST_ANGULAR_VELOCITY
	if (data.buttons[2] == True):
		# Do not publish
		return
		
	pub.publish(twist)


def listener():
	rospy.init_node('joy_listener', anonymous=True)
	rospy.Subscriber("/joy",Joy,callback)
	rospy.spin()





if __name__ == "__main__":
	listener()