#!/usr/bin/env python
# coding: utf-8



# imports
import sys
import roslib
import rospy
import cv2
import numpy as np

from sklearn import linear_model

from std_msgs.msg import String, Int16

from sensor_msgs.msg import Image
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from matplotlib import pyplot as plt

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

# functions -------------------------------------------------------------------
def yaw_to_quaternion(yaw):
    """convert a yaw angle (in radians) into a Quaternion message"""
    return Quaternion(0, 0, math.sin(yaw / 2), math.cos(yaw / 2))

def quaternion_to_yaw(quaternion):
    return np.arccos(quaternion.w) * 2 * np.sign(quaternion.z)

def read_pose(pose, corr=False):
    if corr:
        x =  - pose.position.x
        y = pose.position.y
        yaw = np.pi - quaternion_to_yaw(pose.orientation)
    else:
        x = pose.position.x
        y = pose.position.y
        yaw = quaternion_to_yaw(pose.orientation)
    return x, y, yaw


# the node -------------------------------------------------------------------
class forced_driver:
    # loads a potential map and follows it after figuring out where it is

    def __init__(self):
        print("running")
        # subscriptions
        # we import odom messages from odom_gps
        self.odom_gps_sub  = rospy.Subscriber("/assignment6/odom", Odometry, self.callback, queue_size=1)
        
        # we publish to steer / speed
        self.steer = rospy.Publisher('/manual_control/steering', Int16, queue_size=10, latch=True)
        self.speed = rospy.Publisher("/manual_control/speed", Int16, queue_size=10, latch=True)

        # setup the map (in m)
        self.x_size = 6 
        self.y_size = 4 
        self.res = 0.1 

        self.pot = np.load('matrixDynamic_lane1.npy')
        # self.pot = np.load('matrixDynamic_lane2.npy')

        self.first = True 
    
    def callback(self, data):

        circles = False   # are we doing task 1b
        publish = True   # do we want to publish data to the car?

        # data is a odom message
        x, y, theta = read_pose(data.pose.pose, corr=True)



        x_i = int(x*self.res)
        y_i = int(y*self.res)
        
        # if we are outside the map, place us on its edge
        if x_i < 0:
            x_i = 0
        if x_i > ((self.x_size / self.res) - 1):
            x_i = (self.x_size / self.res) - 1

        if y_i < 0:
            y_i = 0
        if y_i > ((self.y_size / self.res) - 1):
            y_i = (self.y_size / self.res) - 1

        # read out target x, y at potential position    
        x_t, y_t = self.pot[x_i, y_i,:]

        # calculate d_x, d_y
        f_x = np.cos(theta) * x_t + np.sin(theta) * y_t
        f_y =-np.sin(theta) * x_t + np.cos(theta) * y_t
        
        # the actual controller...
        Kp = 4.0
        steer_a = Kp * np.arctan(f_y / (4.0 * f_x))

        steering = (steer_a * (180/np.pi)) + 90

        if f_x > 0:
            speed = -150
        else:
            speed = 150
            # TODO: what do we do here? / Is this task 1?
            if f_y > 0:
                steering = 0
            if f_y < 0:
                steering = 180

        if steering > 180:
            steering = 180

        if steering < 0:
            steering = 0

        print('f_x:' + str(f_x) + '; f_y:'+ str(f_x) +'; steering:'+ str(steering) +'; speed:'+ str(speed))

        # publish what we figured out
        if publish:
            if circles:
                if self.first:       # T1b
                    print('task 1b')
                    rospy.sleep(10)
                    self.steer.publish(Int16(steering))
                    self.speed.publish(Int16(speed))
                    self.first = False

            else:                            # T2 
                print('task2')
                self.steer.publish(Int16(steering))
                self.speed.publish(Int16(speed))   
        

# main ------------------------------------------------------------------------
# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('forced_driver', anonymous=True)
  fd = forced_driver()
  rospy.sleep(5)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)