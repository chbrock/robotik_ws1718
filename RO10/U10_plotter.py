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
    
        # setup the map (in m)
        self.x_size = 6 
        self.y_size = 4 
        self.res = 0.1 

        self.pot = np.load('matrixDynamic_lane1.npy')
        # self.pot = np.load('matrixDynamic_lane2.npy')

        self.first = True 
    
    def plot(self, x, y, theta, filename):

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
                steer_a = (np.pi)/4
            if f_y < 0:
                steering = 180
                steer_a = -(np.pi)/4

        if steering > 180:
            steering = 180

        if steering < 0:
            steering = 0

        print('f_x:' + str(f_x) + '; f_y:'+ str(f_x) +'; steering:'+ str(steering) +'; speed:'+ str(speed))


        # Zaras plot
        yaw = theta
        car_length=0.3

        r = car_length * np.abs(np.tan((np.pi)/2-steering))

        if (r>10):
            r = 10
            print(r)
        if (steer_a<0.0):
            r=-r

        xc = x - np.sin(yaw) * r
        yc = y + np.cos(yaw) * r

        fig, ax = plt.subplots(1,1)
        plt.xlim((0,8))
        plt.ylim((0,6))
        ax.arrow(x, y, car_length*np.cos(yaw), car_length*np.sin(yaw), width=car_length, head_width=car_length, head_length=0.09, fc='b', ec='b')

        #ax = plt.axes()
        ax.arrow(x, y, f_x*np.cos(yaw), f_x*np.sin(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r')

        #ax = plt.axes()
        ax.arrow(x, y, -f_y*np.sin(yaw), f_y*np.cos(yaw), head_width=0.01, head_length=0.01, fc='r', ec='r')

        #ax = plt.axes()
        ax.arrow(x, y, x_t, y_t, head_width=0.01, head_length=0.01, fc='g', ec='g')



        ax.scatter(*(x, y), color='r')
        ax.scatter(*(x + x_t, y + y_t), color='g')
        circ = plt.Circle((xc, yc), r, color='r', fill=False)
        plt.gcf().gca().add_artist(circ)

        plt.show
        fig.savefig(filename)

# main ------------------------------------------------------------------------
# no main. just used for plottinz

fd = forced_driver()

fd.plot(1.0, 3.0, -(np.pi)/4, '1_3.png')
fd.plot(3.0, 1.0, 0, '3_1.png')
fd.plot(5.0, 2.0, (np.pi)/4, '5_2.png')
