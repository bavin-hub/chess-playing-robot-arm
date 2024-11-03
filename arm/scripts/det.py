#!/usr/bin/env python3
import cv2
import os
import numpy as np
import pickle
import rospy
from std_msgs.msg import Float64MultiArray, Int16
import threading




class DetectionTransforms:
    def __init__(self):
        # px to cm
        self.x_px_to_cm = 28.2/960
        self.y_px_to_cm = 20.5/720

        # homogeneous matrix
        # 180 deg rotation around y-axis of the arm
        self.r180_y = [[np.cos(np.pi),0,np.sin(np.pi)],
                [0,1,0],
                [-np.sin(np.pi), 0 , np.cos(np.pi)]]

        # 1 deg rotation around z axis of the arm
        self.rad = (1/180) * np.pi
        self.r4_z = [[np.cos(self.rad),-np.sin(self.rad),0],
                    [np.sin(self.rad),np.cos(self.rad),0],
                    [0,0,1]]


        self.rot = np.dot(self.r180_y,self.r4_z)

        self.disp = [[12.7], [8.3], [0]]

        self.homo = np.concatenate((self.rot,self.disp),1)
        self.homo = np.concatenate((self.homo, [[0,0,0,1]]),0)

        self.w = 960
        self.h = 720
        # w = 1080
        # h = 1080
        self.cap = cv2.VideoCapture(2)
        self.cap.set(3, self.w)
        self.cap.set(4,self.h)

        self.cm_path = '/home/bavin/arm_file/currently_working_codes/CameraCalibration/cameraMatrix.pkl'
        self.dist_path = '/home/bavin/arm_file/currently_working_codes/CameraCalibration/dist.pkl'

        with open(self.cm_path, 'rb') as file:
            self.cameraMatrix = pickle.load(file)
        with open(self.dist_path, 'rb') as file:
            self.dist = pickle.load(file)

        # ros node
        rospy.init_node("det_homo_node")
        self.pub = rospy.Publisher("/robot_frame_coords",
                            Float64MultiArray,
                            queue_size=10)
        

        self.pnp_status_subscriber = rospy.Subscriber("/pnp_status",
                                                      Int16,
                                                      self.pnp_status_cb)
        self.pnp_status = Int16()
        self.pnp_status.data = 0


        self.publish_flag = 1
        coordinates_thread = threading.Thread(target=self.detect_and_find_coordinates)
        coordinates_thread.start()

    def pnp_status_cb(self, msg):
        self.pnp_status = msg
        if not self.publish_flag and self.pnp_status.data == 1:
            print("received from arduino : ", self.pnp_status.data)
            self.publish_flag = 1
            self.pnp_status.data = 0
            print("pick and place operation has been completed so turning on the flag")

        
        

    def detect_and_find_coordinates(self):
        while True:
            
            _, img = self.cap.read()
            # print(img.shape)
            
            # calibration
            h,  w = img.shape[:2]
            newCameraMatrix, roi = cv2.getOptimalNewCameraMatrix(self.cameraMatrix, self.dist, (self.w,self.h), 1, (self.w,self.h))
            dst = cv2.undistort(img, self.cameraMatrix, self.dist, None, newCameraMatrix)
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

                    x_cam_cm = xp * self.x_px_to_cm
                    y_cam_cm = yp * self.y_px_to_cm

                    vec = [[x_cam_cm], [y_cam_cm], [0], [1]]

                    p = np.dot(self.homo, vec)
                    x_base_cm = p[0]
                    y_base_cm = p[1]
                
                    # print('\n',x_cam_cm, y_cam_cm)
                    # print(x_base_cm, y_base_cm)

                    # coordinates in ros format
                    coords = Float64MultiArray()
                    coords.data = [x_base_cm, y_base_cm, 4.0, x_base_cm+4.0, y_base_cm+4.0, 4.0]
                    if self.publish_flag:
                        self.pub.publish(coords)
                        print("coordinates published")
                        self.publish_flag = 0

            cv2.imshow("original", blue)
            cv2.imshow("masked", img)

            if cv2.waitKey(1) & 0XFF == ord('q'):
                break

        self.cap.release()
        cv2.destroyAllWindows()


if __name__=="__main__":
    det_obj = DetectionTransforms()
    rospy.spin()