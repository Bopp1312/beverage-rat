#!/usr/bin/python3

import rospy
import numpy as np
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R

def callback(data):
    quat = data.orientation
    rotation = R.from_quat([quat.x, quat.y, quat.z, quat.w])
    euler_angles = rotation.as_euler('xyz')
    world_x_angle = euler_angles[0]
    #print(world_x_angle*180.0/np.pi)
    #print(euler_angles[1]*180.0/np.pi)
    print(euler_angles[2]*180.0/np.pi)

def listener():
    rospy.init_node("imu_listener", anonymous=True)
    rospy.Subscriber("/imu", Imu, callback)
    rospy.spin()

if __name__ == "__main__":
    listener()
