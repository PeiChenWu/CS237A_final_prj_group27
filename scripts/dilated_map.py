#!/usr/bin/env python3

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData, Path
from scipy.ndimage.morphology import grey_dilation
from geometry_msgs.msg import Twist, Pose2D, PoseStamped
from std_msgs.msg import String
import tf
import numpy as np
from numpy import linalg
from utils.utils import wrapToPi
from utils.grids import StochOccupancyGrid2D
from planners import AStar, compute_smoothed_traj
import scipy.interpolate
import matplotlib.pyplot as plt
from controllers import PoseController, TrajectoryTracker, HeadingController
from enum import Enum


from dynamic_reconfigure.server import Server
from asl_turtlebot.cfg import NavigatorConfig


class Map_Dilation:
    """
    This node handles point to point turtlebot motion, avoiding obstacles.
    It is the sole node that should publish to cmd_vel
    """

    def __init__(self):
        rospy.init_node("map_dilation", anonymous=True)           
        
        ###########  THIS IS FOR MAP DILATION  ###############
        # add a publisher for map dilation
        self.map_dilation_pub = rospy.Publisher("/dilated_map", OccupancyGrid, queue_size=10)
        # add subscriber to original map and dilate it.
        rospy.Subscriber("/map", OccupancyGrid, self.dilate_map_callback) # Subscribe to the original map, and dilate the map and publish it.
        ###########  THIS IS FOR MAP DILATION  ###############
        
        
    def dilate_map_callback(self, msg):
        original_map = np.array(msg.data)
        length = original_map.shape[0]
        original_map = original_map.reshape(int(round(np.sqrt(length))), int(round(np.sqrt(length))))
        dilated_map = grey_dilation(original_map, size=(3,3))
        dilated_map = dilated_map.reshape(int(round(np.sqrt(length)))*int(round(np.sqrt(length))))
        dilated_msg = OccupancyGrid()
        dilated_msg.info = msg.info
        dilated_msg.data = dilated_map
        self.map_dilation_pub.publish(dilated_msg)
    

    def run(self):
        rate = rospy.Rate(25)  # 10 Hz
        rospy.spin()
        #while not rospy.is_shutdown():

        #    try:

        #   except:

        #   rate.sleep()


if __name__ == "__main__":
    dilation_map = Map_Dilation()
    dilation_map.run()
