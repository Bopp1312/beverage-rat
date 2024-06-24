#! /usr/bin/env python3

import rospy 
import numpy as np
import tf
from scipy.spatial.transform import Rotation as R
import threading
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped, Pose
import time

class Command:
    def __init__(self,type,distance=0,angle=0,position=[0,0],pause=0):
        # Types ["angle","position","pause"]
        self.type = type
        self.pause = pause #seconds
        self.position = position #(x,y)
        self.angle = angle
        self.distance =distance

rotate_30 = Command("angle",angle=30)
rotate_60 = Command("angle",angle=60)
rotate_0  = Command("angle",angle=0)
forward_1 = Command("distance", distance=1)

class Navigator:
    def __init__(self):
        self.listener = tf.TransformListener()
        self.broadcaster = tf.TransformBroadcaster()
        self.rate = rospy.Rate(20)

        self.robot_pose = Pose()

        #Publishers
        self.robot_cmd_pub = rospy.Publisher("/cmd/velocity",Twist,queue_size=1)
        
        # Subscribers
        rospy.Subscriber("/kalman/pose",Pose,self.robot_pose_cb,queue_size=1)

        self.commands = []
        self.commands.append(forward_1)
        self.commands.append(rotate_30)
        self.commands.append(rotate_60)
        self.commands.append(rotate_30)
        self.commands.append(rotate_0)

        self.thread = threading.Thread(target=self.loop())
        self.thread.start()

        rospy.loginfo("Navigator Initialized")
    
    def robot_pose_cb(self,msg):
        self.robot_pose = msg

    def process(self, command):
        if command.type == "pause":
            time.sleep(command.pause)

        if command.type == "angle":
            rospy.loginfo("Go to angle: %.3f" %command.angle )

            self.go_to_angle(command.angle)

    def go_to_distance(self, distance)
        desired_distance = distance
        start_pos = np.array([self.robot_pose.position.x,self.robot_pose.positin.position.y])
        actual_distance = 0.0
        
        distance_error = desired_distance - actual_distance
        msg = Twist()
        while(np.abs(distance_error) > 0.001):
            actual_pose = np.array([self.robot_pose.position.x,self.robot_pose.positin.position.y])
            actual_distance = np.linalg.norm(actual_pose-start_pose)
            distance_error = desired_distance - actual_distance
            rospy.loginfo("Completing distance[error: %.3f actual: %.3f]"%(distance_error,actual_distance))
            
            msg.linear.x = self.clip(distance_error, -0.1,0.1)
            self.robot_cmd_pub.publish(msg)
            self.rate.sleep()
        # Once finished
        rospy.loginfo("Go to distance Complete")
        msg.linear.x = 0
        self.robot_cmd_pub.publish(msg)


    def go_to_angle(self, angle):
        # Publish desired robot frame
        desired_angle = angle*np.pi/180.0
        actual_angle = self.get_robot_yaw()
        rospy.loginfo("Angles: %.3f" % actual_angle)
       
        angular_error = desired_angle - actual_angle
        msg = Twist()
        while(np.abs(angular_error) > 0.001):
            actual_angle = self.get_robot_yaw()
            angular_error = desired_angle - actual_angle
            rospy.loginfo("Completing rotation[error: %.3f actual:%.3f]"%(angular_error,actual_angle))
            
            msg.angular.z = self.clip(angular_error*2,-0.2,0.2)
            self.robot_cmd_pub.publish(msg)
            self.rate.sleep()
        # Once finished
        rospy.loginfo("Go to angle Complete")
        msg.angular.z = 0
        self.robot_cmd_pub.publish(zero)

    def get_robot_yaw(self):
        q = self.robot_pose.orientation
        quat = [q.w,q.x,q.y,q.z]
        while(np.linalg.norm(quat) < 0.0001):
            q = self.robot_pose.orientation
            quat = [q.w,q.x,q.y,q.z]
            rospy.loginfo("Waiting for pose")

        r = R.from_quat(quat)
        angle = r.as_euler('xyz')
        return angle[0]

    def clip(self,value, min_val, max_val):
            if value > max_val:
                return max_val
            elif value < min_val:
                return min_val
            return value

    def loop(self):
        while(rospy.is_shutdown() == False):
            rospy.loginfo("Loop")
            if(len(self.commands) > 0):
                time.sleep(1.0)
                self.process(self.commands.pop(0))
            else: 
                rospy.loginfo("Navigation complete")
                quit()

if __name__ == "__main__":
    rospy.init_node("navigator_node", anonymous=True)
    obj = Navigator()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")
