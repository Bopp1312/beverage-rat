#! /usr/bin/ python3

import numpy as np
import cv2, PIL
from cv2 import aruco
import matplotlib.pyplot as plt
import matplotlib as mpl
import pandas as pd
import denHartLib as dh
from geometry_msgs.msg import PoseStamped

aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
parameters =  aruco.DetectorParameters_create()

# Load camera calibration
matrix_coef = np.load("../../Calibration/matrix_coefficents.npy")
distortion_coef = np.load("../../Calibration/distortion_coefficents.npy")

marker_length = 0.210 #m


# Transform from world origin to markers
# World frame is defined as 0.5 meters infront of marker:0
# with X+ of the world frame facing Marker:0
TW_0 = dh.transformTranx(0.5) * dh.rotz(np.pi)

print(TW_0)

def main():
    cap = cv2.VideoCapture(2)
    while(not rospy.is_shutdown()):
        # Capture frame-by-frame
        ret, frame = cap.read()
        frame = cv2.resize(frame, (800,450),cv2.INTER_LINEAR)
        # Extract all information of tags from image
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=parameters,cameraMatrix=matrix_coef,distCoeff=distortion_coef)
    
        #frame = (aruco.drawDetectedMarkers(frame.copy(), corners, ids))
        if len(corners) > 0:
            for i in range(len(ids)):
                rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(corners[i], marker_length, matrix_coef,distortion_coef)
                #print(rvec)
                print(tvec[0][0]/1000)
                print(ids[i])
                frame = (cv2.aruco.drawAxis(frame,matrix_coef,distortion_coef,rvec[i,:,:],tvec[i,:,:],marker_length))

        #thumbnail = cv2.resize(frame, (900,600), cv2.INTER_LINEAR)
        thumbnail = frame
        # Display the resulting frame
        cv2.imshow("frame",thumbnail)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            quit()

    # When everything done, release the capture
    cap.release()
    cv2.destroyAllWindows()

def toTransform(self,rVec,tVec):
    transform = np.eye(4,4)
    rotation, jac = cv2.Rodrigues(rVec)
    position = tVec
    transform[:3,:3] = rotation
    transform[:3,3] = position
    return np.matrix(transform) 

if __name__ == "__main__":
    rospy.init_node("aruco_watcher",)
    main()
