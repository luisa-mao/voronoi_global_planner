#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import random
import math
import heapq
import matplotlib.pyplot as plt
from vor_utils import *



class ScanToGoal:
    def __init__(self):
        rospy.init_node('scan_to_goal')
        self.count = 0
        self.points = []

        # Create subscribers and publishers
        scan_sub = Subscriber('/front/scan', LaserScan)
        odom_sub = Subscriber('/enml_odometry', Odometry) # enml odom is frame odom
        self.goal_pub = rospy.Publisher('/move_base_simple/localgoal', PoseStamped, queue_size=10)

        # Synchronize the scan and odom messages
        ts = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=10, slop=0.1)
        ts.registerCallback(self.scan_odom_callback)

        rospy.spin()

    def scan_odom_callback(self, scan_msg, odom_msg):
        print("scan odom callback")

        # if scan_msg.header.seq != 211:
        #     return

        # TODO: Implement scan-to-goal conversion based on the scan and odom data
        # You can access the scan data with scan_msg.ranges and the odom data with odom_msg.pose.pose
        # make the tranforms better laser base -> odom
        # also do the forward inference thing to account for delays
        yaw = get_yaw(odom_msg)
        shift_x = odom_msg.pose.pose.position.x
        shift_y = odom_msg.pose.pose.position.y
        start = (shift_x, shift_y)
        goal = (0,10)
        # print(start)

        angle_min = scan_msg.angle_min
        increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        clearance = 0.7

        points = ranges_to_coordinates(ranges, angle_min + yaw, increment)

        for i in range(len(points)):
            x, y = points[i]
            x += shift_x
            y += shift_y
            points[i] = (x, y)

        self.points += points
        self.points = list(set(self.points))
        vor = Voronoi(self.points)
        # fig = voronoi_plot_2d(vor)
        map = get_edge_map(vor, self.points, clearance, start, goal)
        path = astar(start, goal, map)

        name = "gif_images/" + str(self.count) + ".png"


        # make a gif
        plot(vor, self.points, path, start, False, True, name)
        self.count+=1
        
        # Create a new goal PoseStamped message and fill in its fields
        # goal_msg = PoseStamped()
        # TODO: Fill in the fields of goal_msg based on the scan and odom data
        
        # Publish the goal message
        # self.goal_pub.publish(goal_msg)

if __name__ == '__main__':
    try:
        ScanToGoal()
    except rospy.ROSInterruptException:
        pass
