#!/usr/bin/env python3
import cv2
import os
import numpy as np
import pickle
import rospy
from std_msgs.msg import Float64MultiArray

# px to cm
x_px_to_cm = 28.2/960
y_px_to_cm = 20.5/720

# homogeneous matrix
# 180 deg rotation around y-axis of the arm
r180_y = [[np.cos(np.pi),0,np.sin(np.pi)],
          [0,1,0],
          [-np.sin(np.pi), 0 , np.cos(np.pi)]]

# 1 deg rotation around z axis of the arm
rad = (1/180) * np.pi
r4_z = [[np.cos(rad),-np.sin(rad),0],
        [np.sin(rad),np.cos(rad),0],
        [0,0,1]]

# rad1 = (-6/180) * np.pi
# r4_y = [[np.cos(rad1),0,np.sin(rad1)],
#         [0,1,0],
#         [-np.sin(rad1), 0, np.cos(rad1)]]

rot = np.dot(r180_y,r4_z)
# rot = np.dot(rot,r4_y)

disp = [[12.7], [8.3], [0]]

homo = np.concatenate((rot,disp),1)
homo = np.concatenate((homo, [[0,0,0,1]]),0)

w = 960
h = 720
# w = 1080
# h = 1080
cap = cv2.VideoCapture(2)
cap.set(3, w)
cap.set(4,h)

cm_path = '/home/bavin/arm_file/currently_working_codes/CameraCalibration/cameraMatrix.pkl'
dist_path = '/home/bavin/arm_file/currently_working_codes/CameraCalibration/dist.pkl'

with open(cm_path, 'rb') as file:
    cameraMatrix = pickle.load(file)
with open(dist_path, 'rb') as file:
    dist = pickle.load(file)

# ros node
rospy.init_node("det_homo_node")
pub = rospy.Publisher("/robot_frame_coords",
                     Float64MultiArray,
                     queue_size=10)


while True:
    
    _, img = cap.read()
    print(img.shape)
    
    # calibration
    h,  w = img.shape[:2]
    newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(cameraMatrix, dist, (w,h), 1, (w,h))
    dst = cv2.undistort(img, cameraMatrix, dist, None, newCameraMatrix)
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    img = dst
   
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    # lower = np.array([71, 113, 109], np.uint8)
    # lower = np.array([45, 56, 54], np.uint8)
    # lower = np.array([5, 49, 153], np.uint8)
    # lower = np.array([90, 139, 99], np.uint8)
    # lower = np.array([83, 97, 85], np.uint8)
    # lower = np.array([83, 132, 38], np.uint8)
    # lower = np.array([92, 160, 80], np.uint8)
    lower = np.array([71, 123, 43], np.uint8)

    higher = np.array([255, 255, 255], np.uint8)

    blue = cv2.inRange(hsv, lower, higher)
    kernel = np.ones((5,5), "uint8")

    blue = cv2.dilate(blue, kernel)
    img=cv2.circle(img,(0,0),10,(255,0,0),-1)


    # mask = cv2.inRange(hsv, lower, higher)
    # result = cv2.bitwise_and(img, img, mask=mask)

    contours,hierarchy=cv2.findContours(blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(contour)
        if area > 100:
            x,y,w,h = cv2.boundingRect(contour)

            xp = int((2*x+w)/2)
            yp = int((2*y+h)/2)
            
            img = cv2.rectangle(img, (x,y), (x+w,y+h), (255,0,0),2)
            img = cv2.circle(img, (xp,yp),5,(255,0,0), -1)
            img=cv2.line(img,(0,0),(xp,yp),(0,255,0),2)

            x_cam_cm = xp * x_px_to_cm
            y_cam_cm = yp * y_px_to_cm

            vec = [[x_cam_cm], [y_cam_cm], [0], [1]]

            p = np.dot(homo, vec)
            x_base_cm = p[0]
            y_base_cm = p[1]
        
            # print('\n',x_cam_cm, y_cam_cm)
            # print(x_base_cm, y_base_cm)

            # coordinates in ros format
            coords = Float64MultiArray()
            coords.data = [x_base_cm, y_base_cm, 4.0, x_base_cm+4.0, y_base_cm+4.0, 4.0]
            pub.publish(coords)

    cv2.imshow("original", blue)
    cv2.imshow("masked", img)

    key = cv2.waitKey(1)
    if key==27:
        break

cap.release()
cv2.destroyAllWindows()
