#!/usr/bin/env python
# coding: utf-8

# imports
import sys
import roslib
import rospy
import numpy as np
from matplotlib import pyplot as plt

# message types
from std_msgs.msg import Int16
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class controller:

    def __init__(self):
    # subscriptions
        self.steer = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
        # odom TODO:
        self.odom_sub = rospy.Subscriber("/odom",Odometry,self.callback, queue_size=1)
        self.last_t = rospy.Time.now()

        self.deltaY_t_old = 0

        self.first = True
        self.Y_TARGET = 0.2
        self.counter = 0
    
    def callback(self, data):
        # Constants -----------------------------------------------------------
        ZERO = 68     # approx. for .110  -> very bad steering!
        Kp = -50 # Konstante mit der gemessener Referenzwert Lenkeinschlag berechnet wird
        Kd = -10 # Dämpfungskonstante
        cFreq = 0.01   # assume 10ms update!

        # get actual position and heading --------------------------------

        # update cFreq
        t = rospy.Time.now()
        t_inc = t - self.last_t
        cFreq = 1 / t_inc.to_sec()
        self.last_t = t
        
        # data holds odom message
        y_t = data.pose.pose.position.y
        deltaY_t =  self.Y_TARGET - y_t 
        

        # publish to steer -----------------------------------------------        
        value = Kp * (deltaY_t) + Kd * ((self.deltaY_t_old - deltaY_t)) * cFreq + ZERO
 
        # limit the value
        if value < 0: value = 0
        if value > 180: value = 180

        if self.first:
            self.Y_TARGET = data.pose.pose.position.y + 0.2
            self.first = False

        else:    
            self.steer.publish(value)

            if self.counter %10 ==0:
                filename = "task05_02_y.csv"
                with open(filename, 'a') as output:
                    output.write(str(deltaY_t)+"\n")
            self.counter += 1

        # push deltaH_t to memory
        self.deltaY_t_old = deltaY_t

# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('A2_controller', anonymous=True)
  c = controller()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)