#! /usr/bin/env python3

import rospy
import numpy as np
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped, Pose
from std_msgs.msg import Float64MultiArray
from scipy.spatial.transform import Rotation as R
import tf

def get_quaternion_from_euler(roll, pitch, yaw):
  """
  Convert an Euler angle to a quaternion.
   
  Input
    :param roll: The roll (rotation around x-axis) angle in radians.
    :param pitch: The pitch (rotation around y-axis) angle in radians.
    :param yaw: The yaw (rotation around z-axis) angle in radians.
 
  Output
    :return qx, qy, qz, qw: The orientation in quaternion [x,y,z,w] format
  """
  qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
  qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
  qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
  qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
 
  return [qx, qy, qz, qw]
 
class Kalman_node:
    def __init__(self):
        rospy.Subscriber("/heading", Twist, self.imu_callback, queue_size=1)
        rospy.Subscriber("/odom", Twist, self.odom_callback, queue_size=1)
        rospy.Subscriber("/visual", Pose, self.visual_callback, queue_size=1)
        rospy.Subscriber("/cmd/velocity", Twist, self.command_callback, queue_size=1)
        #rospy.Subscriber("/robot/pose", PoseStamped, self.grounding_update, queue_size=1)

        self.listener = tf.TransformListener()
        self.br = tf.TransformBroadcaster()
        
        self.pose_pub = rospy.Publisher("/kalman/pose", Pose, queue_size=1)
        self.kalman_gain_pub = rospy.Publisher("kalman/gain", Float64MultiArray, queue_size=1)
        self.kalman_estimate_pub = rospy.Publisher("kalman/estimate",Float64MultiArray, queue_size=1)

        self.kalman_lin_vel_w_pub = rospy.Publisher("kalman/world/lin_vel",Float64MultiArray, queue_size=1)

        # Create uncertainty matrices
        self.P_0 = np.diag((0.001, 0.001, 0.001, 0.001, 0.001, 0.001))
        Q = self.P_0
        R = np.diag((0.00001, 0.00001, 0.00001, 0.00001))

        self.pose = Pose()

        freq = 50
        dt = 1/freq

        # Initialize states
        self.theta_offset = 0 # Degrees
        self.theta = 0
        self.theta_dot = 0

        self.robot_ang_vel_z = 0 
        self.robot_lin_vel_x = 0

        self.X_0 = np.matrix([[0],
                             [0],
                             [0],
                             [0],
                             [0],
                             [0]])

        self.A = np.matrix([[1, 0, 0, dt,  0,  0],
                            [0, 1, 0,  0, dt,  0],
                            [0, 0, 1,  0,  0, dt],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0]])

        self.H = np.matrix([[0,0,0,1,0,0],
                            [0,0,0,0,1,0],
                            [0,0,1,0,0,0],
                            [0,0,0,0,0,1]])

        self.X = self.X_0
        self.X_n_n = self.X_0
        self.P_n_n = self.P_0
        self.Q = Q
        self.R = R

        self.rate = rospy.Rate(freq)
        # Start loop on a thread

        self.thread = threading.Thread(target=self.loop())
        self.thread.start()

    def restart_filter(self, trans):
        new_x = trans[0] 
        new_y = trans[1] 
        # Create uncertainty matrices
        self.P_0 = np.diag((0.001, 0.001, 0.001, 0.001, 0.001, 0.001))
        Q = self.P_0
        R = np.diag((0.00001, 0.00001, 0.00001, 0.00001))

        self.pose = Pose()

        freq = 50
        dt = 1/freq

        # Initialize states
        self.theta = 0
        self.theta_dot = 0

        self.X_0 = np.matrix([[new_x],
                              [new_y],
                              [self.theta],
                              [0],
                              [0],
                              [0]])

        self.A = np.matrix([[1, 0, 0, dt,  0,  0],
                            [0, 1, 0,  0, dt,  0],
                            [0, 0, 1,  0,  0, dt],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0],
                            [0, 0, 0,  0,  0,  0]])

        self.H = np.matrix([[0,0,0,1,0,0],
                            [0,0,0,0,1,0],
                            [0,0,1,0,0,0],
                            [0,0,0,0,0,1]])

        self.X = self.X_0
        self.X_n_n = self.X_0
        self.P_n_n = self.P_0
        self.Q = Q
        self.R = R
        #print("updated")

    def imu_callback(self, data):
        #print("imu")
        self.theta = -1.0*data.linear.x*np.pi/180.0 
 
    def odom_callback(self, data):
        #print("odom")
        self.robot_lin_vel_x = data.linear.x
        self.robot_ang_vel_z = -1.0*data.angular.z
        self.world_lin_vel_x = np.cos(self.theta)*self.robot_lin_vel_x
        self.world_lin_vel_y = np.sin(self.theta)*self.robot_lin_vel_x
        self.theta_dot = self.robot_ang_vel_z
        msg_vel = Float64MultiArray()
        msg_vel.data = [self.world_lin_vel_x,self.world_lin_vel_y]
        self.kalman_lin_vel_w_pub.publish(msg_vel)

    def visual_callback(self, data):
        # Use the visual pose estimate to
        # initialize the Kalman filter with the pose estimate
        print("visual_update")

    def command_callback(self, data):
        print("cmd")
        self.robot_lin_vel_x_cmd = data.linear.x
        self.robot_ang_vel_z_cmd = data.angular.z

    def update_estimation(self):
        print("State estimation")

        self.B = np.matrix([[0                 , 0],
                            [0                 , 0],
                            [0                 , 0],
                            [np.cos(self.theta), 0],
                            [np.sin(self.theta), 0],
                            [0                 , 1]])

        # Update control signal
        self.U = np.matrix([[self.robot_lin_vel_x_cmd],
                       [self.robot_ang_vel_z_cmd]])  

        # Simulate dynamics
        X_g = self.A*self.X+self.B*self.U
        Y_n = self.H*self.X

        # Populate Z vector with measurements
        z = np.matrix([[self.world_lin_vel_x],
                       [self.world_lin_vel_y],
                       [self.theta],
                       [self.theta_dot]])

        # State Estimation
        (X_e,P_1) = self.predict(self.A,self.B,self.U)
        
        msg = Float64MultiArray()
        msg.data = np.diagonal(P_1).tolist()
        self.kalman_estimate_pub.publish(msg)
        
        self.update(self.H, z)

        #print(X_e)
        self.pose.position.x = X_e[0,0]
        self.pose.position.y = X_e[1,0]

        r = R.from_euler('xyz',[self.theta,0,0])
        w,x,y,z = r.as_quat()
        
        self.pose.orientation.x = x 
        self.pose.orientation.y = y
        self.pose.orientation.z = z 
        self.pose.orientation.w = w 

        self.pose_pub.publish(self.pose)

        self.X = X_g

    def loop(self):
        while(rospy.is_shutdown() == False):
            print("loop")
            try:
                self.update_estimation()
            except Exception as e:
                print("Problem: " + str(e))
            
            vel = np.matrix([self.robot_ang_vel_z, self.robot_lin_vel_x])
            quat = get_quaternion_from_euler(0,0,self.theta)
            self.br.sendTransform((self.pose.position.x,self.pose.position.y,0),quat, rospy.Time.now(),"robot","world")

            # if(np.linalg.norm(vel)<0.05):
            #     try:
            #         (trans,rot) = self.listener.lookupTransform('/camera', '/world', rospy.Time(0))
            #         self.restart_filter(trans)
            #     except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            #         print("No transform ready")

            self.rate.sleep()

    def predict(self, A, B, U):
        X_1_n = A*self.X_n_n+B*U
        P_1_n = A*self.P_n_n*A.transpose()+self.Q
        self.P_1_n = P_1_n
        self.X_1_n = X_1_n
        return (X_1_n, P_1_n)

    def update(self, H, z_n):
        # Advance a time step
        self.X_n_n = self.X_1_n
        self.P_n_n = self.P_1_n
        X = self.X_n_n
        P = self.P_n_n
        R = self.R
        I = np.eye(6)

        K_n = P*H.transpose()*np.linalg.inv(H*P*H.transpose()+R)
        self.X_n_n = X + K_n*(z_n - H*X)
        self.P_n_n = (I-K_n*H)*P*(I-K_n*H).transpose()+K_n*R*K_n.transpose()
        
        msg = Float64MultiArray()
        msg.data = np.diagonal(K_n).tolist()
        self.kalman_gain_pub.publish(msg)
        
        return (self.X_n_n, self.P_n_n)


def main():
    rospy.init_node("kalman_filter", anonymous=True)
    obj = Kalman_node()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting Down")

if __name__ == "__main__":
    main()