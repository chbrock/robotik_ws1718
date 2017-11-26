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
from sensor_msgs.msg import LaserScan

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class controller:

    def __init__(self):
    # subscriptions
        self.steer = rospy.Publisher('/manual_control/steering', Int16, queue_size=10)
        self.lidar_sub = rospy.Subscriber("/scan",LaserScan,self.callback, queue_size=1)
        self.last_t = rospy.Time.now()

        self.deltaH_t_old = 0
    
    def callback(self, data):
        # Constants -----------------------------------------------------------
        ZERO = 100     # approx. for .104
        # mit kp positiv 100 und kd null lenkt er in die richtige Richtung
        Kp = 400
        Kd = 50
        cFreq = 0.01   # assume 10ms update!
        p = 0.4 # Abstand zur Wand

        # get actual position and heading --------------------------------

        # update cFreq
        t = rospy.Time.now()
        t_inc = t - self.last_t
        cFreq = 1 / t_inc.to_sec()
        self.last_t
        
        # data holds a LaserScan message
        off = data.angle_min
        inc = data.angle_increment
        l = len(data.ranges)

        # get distance to wall and our heading

        # pick two angles around 90 to the right (240 and 300) 
        deg30 = np.pi/6
        a1 = int((8*deg30)/inc)
        a2 = int((10*deg30)/inc)

        d1 = data.ranges[a1]
        d2 = data.ranges[a2]


        if not np.isinf(d1) and not np.isinf(d2):

            #print(d1, d2)

            # as in assignment 4

            # we need to know the angle to the wall phi1
            # get distance on wall (Kosinussatz)
            w =  np.sqrt(d1**2 + d2**2 - 2*d1*d2 * np.cos(np.pi/3))      
            # get angle of d1 to wall (Sinussatz)
            phi1 = np.arcsin((np.sin(np.pi/3)) *d2/ w)

            #print(w, phi1)
            # get d0 (Rechtwinkliges Dreieck)
            d0 = np.sin(phi1) * d1

            # as in assignment 5
            
            # get direction of car (s. skizze)
            # winkel der winkelhalbierenden zw a1 und a2 zur wand (gamma)
            gamma = np.pi - phi1 - np.pi/6
            
            # angle of car to line parallel to wall
            theta = np.pi/2 - (np.pi - gamma)
            print(d0, theta)
            
            s = 0.25 # Achsenlänge zwischen Vorher und Hinterachse
            l = 0.5  # 
            c_y = d0 + np.sin(theta) * s
            theta_star = np.arctan((p - c_y)/ l)

            # publish to steer -----------------------------------------------

            deltaH_t = theta_star - theta
            value = Kp * (deltaH_t) + Kd * ((deltaH_t - self.deltaH_t_old)) * cFreq + ZERO
     
            # limit the value
            if value < 0: value = 0
            if value > 180: value = 180
            
            
            print(deltaH_t)

            filename = "task05_03_y.csv"
            with open(filename, 'a') as output:
                    output.write(str(t.to_sec())+" "+str(d0)+" "+str(theta)+" "+str(value)+"\n")
            
            self.steer.publish(value)

            # push deltaH_t to memory
            self.deltaH_t_old = deltaH_t

# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('A3_controller', anonymous=True)
  c = controller()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)