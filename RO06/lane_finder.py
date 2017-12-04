#!/usr/bin/env python
# coding: utf-8



# imports
import sys
import roslib
import rospy
import cv2
import numpy as np

from sklearn import linear_model

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class lane_finder:

    def __init__(self):
    # subscriptions
        self.image_pub_rgb = rospy.Publisher("/image_processing/bin_rgb", Image, queue_size=1)
        self.image_pub_hsv = rospy.Publisher("/image_processing/bin_hsv", Image, queue_size=1)
        self.image_pub_bin = rospy.Publisher("/image_processing/bin_img", Image, queue_size=1)
        self.image_pub_con = rospy.Publisher("/image_processing/con_img", Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw", Image, self.callback, queue_size=1)

    
    def callback(self, data):
        CROP = 120         # crop from top
        DRAW_CONTOURS = False
        
        # get the image -------------------------------------------------------
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        y_dim = cv_image.shape[0]
        x_dim = cv_image.shape[1]    

        #----------------------------------------------------------------------
        # Task 1: Colorspaces -------------------------------------------------
        #----------------------------------------------------------------------

        # 1st: BGR ------------------------------------------------------------

        ABS_THRES = 200 * 3

        # sum over all channels
        bgr_bin = np.sum(cv_image, axis=2)

        # threshold
        bgr_bin[bgr_bin[:,:] < ABS_THRES] = 0
        bgr_bin[bgr_bin[:,:] >= ABS_THRES] = 255

        try:
            self.image_pub_rgb.publish(self.bridge.cv2_to_imgmsg(bgr_bin.astype(np.uint8), "mono8"))
        except CvBridgeError as e:
            print(e)
        


        # 2nd: HSV ------------------------------------------------------------

        V_THRES = 200
        
        # Convert BGR to HSV
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # mask on V only
        hsv_bin = np.zeros((y_dim, x_dim))
        hsv_bin[hsv[:,:,2] > V_THRES] = 255
        
        try:
            self.image_pub_hsv.publish(self.bridge.cv2_to_imgmsg(hsv_bin.astype(np.uint8), "mono8"))
        except CvBridgeError as e:
            print(e)
        

        # 3rd: Grayscale ------------------------------------------------------

        # convert BGR to gray
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # make it binary
        max_value = 255
        thres_val = 200
        ret,thresh1=cv2.threshold(gray, thres_val, max_value, cv2.THRESH_BINARY);

        # publish the image somewhere:
        try:
            self.image_pub_bin.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
        except CvBridgeError as e:
            print(e)
        
        #----------------------------------------------------------------------
        # Task 2: detect the lane----------------------------------------------
        #----------------------------------------------------------------------

        # for the time beeing crop the first 300px
        thresh2 = thresh1[CROP:,:]    

        # using contours
        im2, contours, hierarchy = cv2.findContours(thresh2, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        # get the two larges contours as lanes:
        lane_contours = [None, None]
        first_size = 0
        secnd_size = 0

        for contour in contours:
            test = cv2.contourArea(contour)
            if test > first_size:
                lane_contours[1] = lane_contours[0]
                secnd_size = first_size
                lane_contours[0] = contour
                first_size = test
            elif test > secnd_size:
                lane_contours[1] = contour
                secnd_size = test
        
        if DRAW_CONTOURS:
            cv2.drawContours(cv_image[CROP:,:], lane_contours, 0, (0,255,0), 3)
            cv2.drawContours(cv_image[CROP:,:], lane_contours, 1, (255,0,0), 3)

        # Robustly fit linear model with RANSAC algorithm
        lane_models = [None, None]
        for i in range(len(lane_contours)):
            points = lane_contours[i]
            lane_models[i] = linear_model.RANSACRegressor()
            lane_models[i].fit(points[:,:,0], points[:,:,1])


        # somewhat awkward m*x+b parameter extraction
        b1 = lane_models[0].predict(0).item(0)
        b2 = lane_models[1].predict(0).item(0)
        m1 = (lane_models[0].predict(100).item(0) - b1)/100
        m2 = (lane_models[1].predict(100).item(0) - b2)/100

        print("m1: " + str(m1) + " b1: " + str(b1) )
        print("m2: " + str(m2) + " b2: " + str(b2) )


        # draw the lines
        x_max = int(cv_image.shape[1])    
        cv2.line(cv_image[CROP:,:], (0,int(b1)), (x_max,int(b1+x_max*m1)), 
                 (0,0,255), thickness=2, lineType=cv2.LINE_AA)
        cv2.line(cv_image[CROP:,:], (0,int(b2)), (x_max,int(b2+x_max*m2)), 
                 (0,0,255), thickness=2, lineType=cv2.LINE_AA)

        
        # publish an image to the control topic
        try:
            self.image_pub_con.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)




# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('lane_finder', anonymous=True)
  ic = lane_finder()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)