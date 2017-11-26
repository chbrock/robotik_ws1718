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
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class controller:

    def __init__(self):
    # subscriptions
        self.steer = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
        # yaw TODO:
        self.yaw_sub = rospy.Subscriber("/model_car/yaw",Float32,self.callback, queue_size=1)
    
        self.first = True
        self.YAW_TARGET = 0

    def callback(self, data):
        # Constants -----------------------------------------------------------
        ZERO = 100     # approx. for .104
        Kp = 12

        # get actual position and heading --------------------------------

        # data holds yaw message
        deltaYaw_t = self.YAW_TARGET-data.data

        # publish to steer -----------------------------------------------        
        value = Kp * (deltaYaw_t) + ZERO
 
        # limit the value
        if value < 0: value = 0
        if value > 180: value = 180
        if self.first:
            # Aufgabenstellung: Start​ ​ at​ ​ plus​ ​ or​ ​ minus​ ​ 10​ ​ degrees​ ​ with​ ​ your​ ​ orientation​ ​ angle​ ​ theta.
            self.YAW_TARGET = data.data - 10 
            self.first = False

            # Um den Anfangswert für die Plots zu machen
            filename = "task05_01_heading_angle.csv"
            with open(filename, 'a') as output:
                output.write("Target: " + str(self.YAW_TARGET)+"\n")

        else:
            self.steer.publish(value)

            filename = "task05_01_heading_angle.csv"
            with open(filename, 'a') as output:
                output.write(str(data.data)+"\n")


# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('A1_controller', anonymous=True)
  c = controller()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)