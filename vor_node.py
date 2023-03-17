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
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vor_utils import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import geometry_msgs.msg
from smh_astar import *



class ScanToGoal:

    def __init__(self):
        rospy.init_node('scan_to_goal')
        self.count = 0
        self.points = []

        self.path = None
        self.initial_yaw  = 0
        self.start = None


        # Create subscribers and publishers
        scan_sub = Subscriber('/front/scan', LaserScan)
        odom_sub = Subscriber('/enml_odometry', Odometry) # enml odom is frame odom
        self.path_pub = rospy.Publisher('/luisa_path', Path, queue_size=1)
        self.obstacle_viz = rospy.Publisher('obstacle_markers', Marker, queue_size=1)
        self.vertices_viz = rospy.Publisher('vor_vertices', Marker, queue_size=1)
        self.path_vertices = rospy.Publisher('path_vertices', Marker, queue_size=1)

        # Synchronize the scan and odom messages
        ts = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.scan_odom_callback)

        rospy.spin()

    def scan_odom_callback(self, scan_msg, odom_msg):
        print("scan odom callback")
        # print(scan_msg.header)

        # for base_link -> odom transform
        yaw = get_yaw(odom_msg)
        shift_x = odom_msg.pose.pose.position.x
        shift_y = odom_msg.pose.pose.position.y
        start = (shift_x * 10 , shift_y * 10)

        if self.count ==0: # short term hack
            self.initial_yaw = yaw
            self.start = start

        goal = (100*math.cos(self.initial_yaw),100* math.sin(self.initial_yaw))

        # print(start)
        # print(shift_x, shift_y)

        angle_min = scan_msg.angle_min
        increment = scan_msg.angle_increment
        ranges = scan_msg.ranges

        points = ranges_to_coordinates(ranges, angle_min + yaw, increment)

        for i in range(len(points)):
            x, y = points[i]
            x += -0.055 + shift_x # hardcoded laser -> base_link
            y += shift_y
            x = round(x  * 10 / 2) *2   # scale by 10, discretize by 2, make it an int
            y = round(y * 10 /2) *2
            points[i] = (x, y)

        self.points += points
        self.points = list(set(self.points))

        # not sure how the ellipse will work on worlds where orientation seems flipped
        tmp_points = [goal, start] + self.points + generate_ellipse_arc(20, 15, math.pi/2+self.initial_yaw, 3*math.pi/2 + self.initial_yaw, 20)

        vor = Voronoi(tmp_points)

        map = get_edge_map2(vor, start)

        # vor_vertices = np.append(vor.vertices, [[0,0]], axis=0)
        vor_vertices = np.append(vor.vertices, [[start[0],start[1]]], axis=0)
        vor_vertices = np.append(vor_vertices, [[0,100]], axis=0)
        start = len(vor.vertices)
        goal = len(vor.vertices)+1
        self.path = a_star(start, goal, map, vor_vertices, self.path)
        
        if self.path == None: # clear obstacles from environment. Try to start again
            self.points = []
            self.path = None
            print("no feasible path")
            return

        # luisa_path = create_ros_path(smooth_curve(self.path))
        luisa_path = create_ros_path(self.path)
        
        # Publish the path message
        self.path_pub.publish(luisa_path)

        # clear viz
        marker = Marker()
        marker.action = 3
        self.obstacle_viz.publish(marker)
        marker.action = 3
        self.vertices_viz.publish(marker)
        marker.action = 3
        # self.path_vertices.publish(marker)

        # publish viz
        obs = self.make_viz_message(tmp_points, "white", self.count)
        self.obstacle_viz.publish(obs)
        vertices = self.make_viz_message(vor.vertices, "blue", self.count)
        self.vertices_viz.publish(vertices)

        # path_vertices = self.make_viz_message(self.path, "blue", self.count)
        # self.path_vertices.publish(path_vertices)

        self.count+=1

    def make_viz_message(self, points, color, id):
        # Create marker
        marker = Marker()
        marker.id = id
        marker.header.frame_id = "odom"
        marker.type = Marker.POINTS
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        
        # Set marker color based on input argument
        if color == "red":
            marker.color.r = 1.0
            marker.color.g = 0.0
            marker.color.b = 0.0
            marker.color.a = 1.0
        elif color == "blue":
            marker.color.r = 0.0
            marker.color.g = 0.0
            marker.color.b = 1.0
            marker.color.a = 1.0
        elif color == "white":
            marker.color.r = 1
            marker.color.g = 1
            marker.color.b = 1
            marker.color.a = 1.0

        # Set marker points from input list of tuples
        for point in points:
            p = Point()
            p.x = point[0]/10
            p.y = point[1]/10
            p.z = 0
            marker.points.append(p)
        
        return marker


if __name__ == '__main__':
    try:
        ScanToGoal()
    except rospy.ROSInterruptException:
        pass
