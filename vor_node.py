#!/usr/bin/env python3.8

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool

from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from scipy.spatial import Voronoi, voronoi_plot_2d
import matplotlib
# matplotlib.use('Agg')
# from matplotlib import pyplot as plt
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from vor_utils import *
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import geometry_msgs.msg
from smh_astar import *

import roslib
roslib.load_manifest('amrl_msgs')
from amrl_msgs.msg import VisualizationMsg, ColoredPoint2D

from scipy.spatial.distance import directed_hausdorff as hd
import yaml 
import time 

class ScanToGoal:

    def __init__(self):
        rospy.init_node('scan_to_goal')
        self.count = 0
        self.points = []
        self.point_map = {}

        self.path = None
        self.initial_yaw  = 0
        self.start = None


        with open('config/params.yaml', 'r') as f:
            self.config = yaml.safe_load(f)

        # Create subscribers and publishers
        scan_sub = Subscriber('/front/scan', LaserScan) if self.config["simulation"] else Subscriber('/scan', LaserScan)
        odom_sub = Subscriber('/enml_odometry', Odometry) # enml odom is frame odom
        self.path_pub = rospy.Publisher('/luisa_path', Path, queue_size=1)
        self.other_path_pub = rospy.Publisher('/other_path', Path, queue_size=1)
        self.halt_pub = rospy.Publisher("/halt_robot", Bool)
        self.obstacle_viz = rospy.Publisher('obstacle_markers', Marker, queue_size=1)
        self.vertices_viz = rospy.Publisher('vor_vertices', Marker, queue_size=1)
        self.path_vertices = rospy.Publisher('path_vertices', Marker, queue_size=1)
        self.goal_subscriber = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)

        self.webviz_viz_publisher = rospy.Publisher("/visualization", VisualizationMsg, queue_size=1)

        self.goal = None

        # Synchronize the scan and odom messages
        ts = ApproximateTimeSynchronizer([scan_sub, odom_sub], queue_size=1, slop=0.1)
        ts.registerCallback(self.scan_odom_callback)

        self.rate = rospy.Rate(10)
        rospy.spin()

    def get_goal(self, goal: PoseStamped):
        self.goal = (goal.pose.position.x, goal.pose.position.y)

    def scan_odom_callback(self, scan_msg, odom_msg):
        x_scale = 1 / low_resolution
        # self.config["velodyne_transform"]["x"]
        y_scale = 1 / low_resolution
        # self.config["velodyne_transform"]["y"]
        if not self.config["simulation"] and type(self.goal) == type(None):
            return
        print("scan odom callback")
        # print(scan_msg.header)

        # for base_link -> odom transform
        yaw = get_yaw(odom_msg)
        shift_x = odom_msg.pose.pose.position.x
        shift_y = odom_msg.pose.pose.position.y
        start = (shift_x * x_scale, shift_y * y_scale)

        if self.count ==0: # short term hack
            self.initial_yaw = yaw
            self.start = start

        # self.goal = None
        goal = (self.goal[0] * x_scale, self.goal[1] * y_scale) if not self.config["simulation"] else \
            (10*x_scale*math.cos(self.initial_yaw)+self.start[0],10*y_scale* math.sin(self.initial_yaw)+self.start[1])
        
        print("goal", goal)

        angle_min = scan_msg.angle_min
        increment = scan_msg.angle_increment
        ranges = scan_msg.ranges
        if (self.config["simulation"]):
            self.points = correct_obstacles(self.points, ranges, angle_min + yaw, increment, shift_x, shift_y, yaw)

        points, self.point_map = translate_and_scale(ranges_to_coordinates(ranges, angle_min, increment, yaw), shift_x, shift_y, self.point_map)

        self.points += points
        self.points = list(set(self.points))

        # not sure how the ellipse will work on worlds where orientation seems flipped
        radius = 10 / low_resolution
        num = 100
        circle_points = generate_circle_points(start, radius, num)
        tmp_points = [goal, start] + self.points + circle_points 
        # \ + generate_ellipse_arc(20, 15, math.pi/2+self.initial_yaw, 3*math.pi/2 + self.initial_yaw, 20)
        self.point_map.setdefault(goal, set()).add(goal)
        self.point_map.setdefault(start, set()).add(start)


        # for elem in tmp_points:
        #     assert elem in self.point_map.keys(), f"{elem} is not a key in the dictionary"


        vor = Voronoi(tmp_points)

        # map = get_edge_map2(vor, start)
        map = get_edge_map(vor, start, goal, self.point_map, circle_points)

        old_path = self.path
        # self.path = closest_path(start, goal, map, old_path)
        # if self.path is None:
        result = astar(start, goal, map)

        if result is None:
            self.points = []
            self.path = None
            vertices = self.make_viz_message(vor.vertices, "blue", self.count)
            self.vertices_viz.publish(vertices)
            obs = self.make_viz_message(tmp_points, "white", self.count)
            self.obstacle_viz.publish(obs)
            msg = Bool()
            self.halt_pub.publish(msg)
            self.rate.sleep()
            return
        new_path, new_avg_gap = result

        if old_path is not None: #and path_distance(old_path) < 1.05 * path_distance(new_path):
            p1 = connected_path(old_path)
            self.path, avg_gap = copy_astar_path(start, goal, map, p1)
            p2 = connected_path(self.path)
            d = max(hd(p1, p2)[0], hd(p2, p1)[0])
            if switch_plan(new_avg_gap, avg_gap, old_path, new_path, d):
                self.path = new_path
            luisa_path = create_ros_path(self.path)

            # luisa_path = create_ros_path(old_path)
        # if False: # if two valid paths, choose the shorter one
        #     pass

        else:
            self.path = new_path
            luisa_path = create_ros_path(self.path)

        # Publish the path message
        self.path_pub.publish(luisa_path)
        self.other_path_pub.publish(create_ros_path(new_path))


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

        path_vertices = self.make_viz_message(self.path, "blue", self.count)
        
        # publish to webviz
        self.publish_message(tmp_points, 16711680, "vertices")
        self.publish_message(self.path, 65280, "vor_path")

        self.path_vertices.publish(path_vertices)

        self.count+=1

    def publish_message(self, points, color, namespace):
        visualization = VisualizationMsg()
        visualization.header.stamp = rospy.Time.now()
        visualization.header.frame_id = "map"

        visualization.ns = namespace 
        for point in points:
            p = ColoredPoint2D()
            p.point.x = point[0] * low_resolution
            p.point.y = point[1] * low_resolution

            p.color = color

            visualization.points.append(p)
        self.webviz_viz_publisher.publish(visualization)

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
            p.x = point[0] * low_resolution
            p.y = point[1] * low_resolution
            p.z = 0
            marker.points.append(p)
        
        return marker


if __name__ == '__main__':
    try:
        ScanToGoal()
    except rospy.ROSInterruptException:
        pass
