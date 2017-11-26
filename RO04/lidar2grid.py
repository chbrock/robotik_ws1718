#!/usr/bin/env python
# coding: utf-8

# imports
import sys
import roslib
import rospy
import cv2
import numpy as np

# message types
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid

# class structure analog to https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

class lidar_converter:

    def __init__(self):
    # subscriptions

        self.pub = rospy.Publisher("scan_grid", OccupancyGrid, queue_size=100)        
        self.image_sub = rospy.Subscriber("/scan",LaserScan,self.callback, queue_size=1)

    
    def callback(self, data):

        # define the grid -----------------------------------------------------

        #do a 8mx8m Grid in 5cm spaces
        spacing = 0.05  # m/cell
        size = 8        # size in m
        points = int(size / spacing)

        # init temprorary grid -----------------------------------------------
        grid = np.ones((points, points)) * -1 # wird eigentlich -1 (wissen nicht)

        # init occupancy grid ------------------------------------------------
        occ_grid = OccupancyGrid()
        occ_grid.header.frame_id = "laser"
        occ_grid.info.resolution = spacing # in m/cell

        # width x height cells
        occ_grid.info.width = points
        occ_grid.info.height = points

        # origin is shifted at half of cell size * resolution
        occ_grid.info.origin.position.x = int(-1.0 * occ_grid.info.width / 2.0) * occ_grid.info.resolution
        occ_grid.info.origin.position.y = int(-1.0 * occ_grid.info.height / 2.0) * occ_grid.info.resolution
        occ_grid.info.origin.position.z = 0
        occ_grid.info.origin.orientation.x = 0
        occ_grid.info.origin.orientation.y = 0
        occ_grid.info.origin.orientation.z = 0
        occ_grid.info.origin.orientation.w = 1

        # process the scan ----------------------------------------------------
        
        # data holds a LaserScan message
        off = data.angle_min
        inc = data.angle_increment
        l = len(data.ranges)

        
        # assuming data comes in rad
        for i in range(l):
            
            # get the position of the reflection point in meters
            a = ((off + i*inc) - np.pi/2) % (2*np.pi)
            v = np.array([np.cos(a), np.sin(a)])  # vgl. Kreisformel
            p = data.ranges[i] * v
            
            # slope
            s = v[1]/v[0]
            
            sng = int(np.sign(p[0]))
            
            if abs(p[0]) != np.inf:                
                i = points/2 + int(p[0]/spacing)
                j = points/2 + int(p[1]/spacing)
            else:
                i = int(sng*(size+1))
                j = int(sng*(size+1))
            
            #cut back to grid size:
                i = trim(i, points)
                j = trim(j, points)
                
            # add free spaces to grid    
            j1 = points/2
            j2 = 0
            start = int(points/2 + sng*1)
            direction = int(sng*1)

            for i_y in range(start, int(i)+sng, direction):
                # stop painting outside the grid in x
                if i_y < 0 or i_y >= points:
                    break
                
                # slope is same in m and gridpoints
                j2 = points/2 + int(1 + (i_y - points/2) * s)
                if j1 < j2:
                    if j2 > points: j2 = points
                    grid[i_y,j1:j2+1] = 0
                else:
                    if j2 < 0: j2 = 0
                    grid[i_y,j2:j1+1] = 0

                # stop painting outside the grid in y
                if j2 < 0 or j2 >= points:
                    break    
                
                j1 = j2


            # add reflection point to grid            
            if ((0 <= i < points) and (0 <= j < points)):
                grid[i,j] = 100

        # dump the thing into the occ_grid
        occ_grid.data = [np.int8(x) for x in list(grid.flat)]
        
        # publish the grid -----------------------------------------------
        
        self.pub.publish(occ_grid)

# helpers ----------------------------------------------------------------
def trim(x, size):
    if x > size: return size-1
    elif x < 0: return 0
    else: return x

# main as in https://github.com/richrdcm/catkin_ws_user/tree/master/src/py_image_processing

def main(args):
  rospy.init_node('lidar_converter', anonymous=True)
  ic = lidar_converter()
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()


if __name__ == '__main__':
    main(sys.argv)