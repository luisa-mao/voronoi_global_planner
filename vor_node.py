#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import tf2_ros
import tf2_geometry_msgs
from vor_utils import *



class ScanToGoal:
    def save_data_invocation(self):
        self.save_data()

    def __init__(self):
        rospy.init_node('scan_to_goal')
        self.count = 0
        self.points = []

        self.clearance = 10
        self.images = []
        self.vor = []
        self.paths = []
        self.points_list = []
        self.starts = []
        self.path = None


        # Create subscribers and publishers
        scan_sub = Subscriber('/front/scan', LaserScan)
        odom_sub = Subscriber('/enml_odometry', Odometry) # enml odom is frame odom
        self.path_pub = rospy.Publisher('/luisa_path', Path, queue_size=1)

        # Synchronize the scan and odom messages
        ts = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.scan_odom_callback)

        # Create a buffer object to store the transform data
        self.tf_buffer = tf2_ros.Buffer()

        # Create a transform listener object to receive the transform data
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)


        # rospy.on_shutdown(self.save_data_invocation)

        rospy.spin()

    def scan_odom_callback(self, scan_msg, odom_msg):
        print("scan odom callback")
        # print(scan_msg.header)

        try:
        # Lookup the transform from base_link to odom
            trans = self.tf_buffer.lookup_transform( scan_msg.header.frame_id, 'base_link', rospy.Time())
            # Transform the laser message from base_link to odom frame
            # scan_msg = tf2_geometry_msgs.do_transform_laser_scan(scan_msg, trans)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # Handle the exception appropriately
            rospy.logerr(f"Failed to transform laser scan to base_link frame: {e}")
            return

        # TODO: Implement scan-to-goal conversion based on the scan and odom data
        # You can access the scan data with scan_msg.ranges and the odom data with odom_msg.pose.pose
        # make the tranforms better laser base -> odom
        # also do the forward inference thing to account for delays
        # replan only if a path is infeasable, otherwise, it's fine
        # make a list of everything needed to make the plots,
        # then make a function to plot and save the images
        # then make into a gif, so images aren't saved concurrently

        yaw = get_yaw(odom_msg)
        shift_x = odom_msg.pose.pose.position.x
        shift_y = odom_msg.pose.pose.position.y
        start = (shift_x * 10 , shift_y * 10)
        goal = (0,100)
        print(start)
        print(shift_x, shift_y)

        angle_min = scan_msg.angle_min
        increment = scan_msg.angle_increment
        ranges = scan_msg.ranges


        points = ranges_to_coordinates(ranges, angle_min + yaw, increment)

        for i in range(len(points)):
            x, y = points[i]
            # x += shift_x
            # y += shift_y
            # x *= 10
            # y *= 10
            x += trans.transform.translation.x + shift_x
            y += trans.transform.translation.y + shift_y
            x = round(x  * 10 / 2) *2
            y = round(y * 10 /2) *2
            points[i] = (x, y)

        self.points += points
        self.points = list(set(self.points))
        tmp_points = [(0,100), start] + self.points
        vor = Voronoi(tmp_points)

        map = get_edge_map(vor, tmp_points, self.clearance, start, goal)
        # can change this so that luisa_path is not none at first, so don't
        # need if statement in astar function
        self.path = astar(start, goal, map, self.path)
        if self.clearance == 2.5:
            print("no feasible path")
            return
        if self.path == None:
            self.clearance -= 0.5
            print("decreased clearance ", self.clearance)
            return



        # make a gif
        self.vor.append(vor)
        self.points_list.append(self.points)
        self.starts.append(start)
        self.paths.append(self.path.copy())
        print("path length ", len(self.path))


        luisa_path = create_ros_path(smooth_curve(self.path))
        
        # Publish the goal message
        self.path_pub.publish(luisa_path)
    def save_data(self):
        for i in range(len(self.vor)):
            name = "gif_images/"+str(i)
            plot(self.vor[i], self.points_list[i], self.paths[i], self.starts[i], False, True, name)
            plt.close()


if __name__ == '__main__':
    try:
        ScanToGoal()
    except rospy.ROSInterruptException:
        pass
