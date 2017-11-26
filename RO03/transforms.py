#!/usr/bin/env python
# coding: utf-8

# talker.py code for reference
"""
def talker():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello world %s" % rospy.get_time()
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
"""

# imports
import sys
import roslib
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class image_converter:

    def __init__(self):
    # subscriptions

        self.image_pub_gray = rospy.Publisher("/image_processing/bin_gray",Image, queue_size=1)
        self.image_pub_bin = rospy.Publisher("/image_processing/bin_img",Image, queue_size=1)
        self.image_pub_con = rospy.Publisher("/image_processing/con_img",Image, queue_size=1)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/app/camera/rgb/image_raw",Image,self.callback, queue_size=1)

    
    def callback(self, data):
        
        # get the image -------------------------------------------------------
        try:
          cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # make it gray
        gray=cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        try:
            self.image_pub_gray.publish(self.bridge.cv2_to_imgmsg(gray, "mono8"))
        except CvBridgeError as e:
            print(e)
        
        # make it binary
        max_value = 255
        thres_val = 225
        ret,thresh1=cv2.threshold(gray, thres_val, max_value, cv2.THRESH_BINARY);

        # publish the image somewhere:
        try:
            self.image_pub_bin.publish(self.bridge.cv2_to_imgmsg(thresh1, "mono8"))
        except CvBridgeError as e:
            print(e)
        
        # find the blobs ------------------------------------------------------
        
        # create a list of blobs
        blobs = []
        # create a reference matrix to record blob assignments
        ref = np.ones(thresh1.shape) * -1
        
        for y in range(thresh1.shape[0]):
            for x in range(thresh1.shape[1]):

                # is the pixel we are looking at white?
                if thresh1[y,x] == max_value:

                    # the radius in which we look for pixels that already belong to a blob                    
                    r = 9

                    # look if a neighbour of x,y is already assigned to a blob
                    try:
                        n_blob = np.max(ref[y-r:y+r,x-r:x+r])            
                    except ValueError:  #raised if `n_blob` is empty.
                        n_blob = -1

                    if n_blob > 0:
                        blob_num = n_blob
                    else:
                        blob_num = len(blobs)
                        blobs.append(blob(blob_num))

                    # add x,y to the same blob
                    blobs[int(blob_num)].add_point(x, y, 1)
                    # set the reference
                    ref[y,x] = blob_num
        
        b_size_cutoff = 15

        points = []
        
        for b in blobs:
            # paint into control image and collect as img points
            if b.size >= b_size_cutoff:
                x = int(b.get_center()[0])
                y = int(b.get_center()[1])
                cv_image[y,x] = [0, 0, 255]

                points.append([x,y])
          
        
        # publish the image to a control topic:
        try:
            self.image_pub_con.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

        # getting the transform -----------------------------------------------
        
        """
        for some stupid reasons, cv2 likes to have its nxm 
        arrays as nxmx1 arrays so we have to be somewhat 
        explicit when setting up the things
        """
        if len(points) == 6:
            img_points = np.zeros((6,2,1))
            img_points[:,:,0] = np.array(points)
            
            print('image points: ---------')
            print(img_points[:,:,0])

            fx = 614.1699
            fy = 614.9002
            cx = 329.9491
            cy = 237.2788

            camera_mat = np.zeros((3,3,1))
            camera_mat[:,:,0] = np.array([[fx, 0, cx],
                                          [0, fy, cy],
                                          [0,  0,  1]])

            k1 =  0.1115
            k2 = -0.1089
            t1 =  0
            t2 =  0

            dist_coeffs = np.zeros((4,1))
            dist_coeffs[:,0] = np.array([[k1, k2, t1, t2]])

            # far to close, left to right (order of
            # discovery) in cm
            obj_points = np.zeros((6,3,1))
            obj_points[:,:,0] = np.array([[00.0, 00.0, 0],
                                          [21.8, 00.0, 0],
                                          [00.0, 30.0, 0],
                                          [22.2, 30.0, 0],
                                          [00.0, 60.0, 0],
                                          [22.0, 60.0, 0]])

            retval, rvec, tvec = cv2.solvePnP(obj_points, img_points, 
                                              camera_mat, dist_coeffs)

            print('rotation vector: ---------')
            print(rvec)
            print('tranlation vector: ---------')
            print(tvec)

            rmat = np.zeros((3,3))

            cv2.Rodrigues(rvec, rmat, jacobian=0)

            print('rotation matrix: ---------')
            print(rmat)
              
# helper classes
class blob:
    
    def __init__(self, i):
        self.num = i       # number given from outside
        self.points = []
        self.size = 0
    
    def add_point(self, x, y, c):
        # does not work yet
        self.points.append([x, y, c])
        self.size += 1

    def get_center(self):
        # returns the (x,y) average of the mean of a blobs points
        mat = np.array(self.points)
        center = np.sum(mat, axis=0)/len(self.points)
        return (center[0], center[1])

# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('image_converter', anonymous=True)
  ic = image_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)