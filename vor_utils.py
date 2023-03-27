from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import random
import math
import heapq
# import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import scipy.interpolate as interpolate
# from sklearn.cluster import DBSCAN
from scipy.spatial.distance import directed_hausdorff
# from sklearn.linear_model import RANSACRegressor


def get_yaw(odom_msg):
    # Extract the yaw angle from the orientation quaternion
    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))

    return yaw


def ranges_to_coordinates(ranges, angle_min, angle_increment):
    """
    Converts a list of LaserScan ranges to x-y coordinates.
    """
    coords = []
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        if not math.isinf(ranges[i]):
            x = ranges[i] * math.cos(angle)
            y = ranges[i] * math.sin(angle)
            coords.append((x, y))
    return coords

def get_edge_map(vor, start, goal):

    map = {}
    ridge_points = vor.ridge_points
    edges = vor.ridge_vertices
    vertices = vor.vertices
    points = vor.points

    for i in range(len(ridge_points)):
        
        pair = ridge_points[i]

        point1 = tuple(points[pair[0]])
        point2 = tuple(points[pair[1]])
        gap = math.dist(point1, point2)
        # set1 = clusters.get(point1, [point1])
        # set2 = clusters.get(point2, [point2])
        # gap = max(directed_hausdorff(set1, set2)[0], directed_hausdorff(set2, set1)[0])

        edge = edges[i]
        if (edge[0]==-1 or edge[1]==-1):
            continue

        node = (vertices[edge[0]][0], vertices[edge[0]][1])
        other = (vertices[edge[1]][0], vertices[edge[1]][1])
        if (map.get(node) == None):
            map[node] = [(other, gap)]
        else:
            map[node].append((other, gap))

        if (map.get(other) == None):
            map[other] = [(node, gap)]
        else:
            map[other].append((node, gap))

    
    pts = vor.regions[vor.point_region[0]]
    print(vor.points[0])
    for p in pts:
        if p == -1:
            continue
        pt = (vertices[p][0], vertices[p][1])
        # print(pt)
        if map.get(pt) != None:
            # print("appended")
            map[pt].append((goal, 10)) # basically this gap is always valid
    
    pts = vor.regions[vor.point_region[1]]
    map[start] = []
    for p in pts:
        if p == -1:
            continue
        pt = (vertices[p][0], vertices[p][1])
        # print(pt)
        map[start].append((pt, 10)) # this gap also always valid


        
    return map

# def plot(vor, points, path, start, show, save, name):
#     fig = voronoi_plot_2d(vor)
#     plt.axis([-50, 50, -30, 70])

#     rx = [p[0] for p in points]
#     ry = [p[1] for p in points]
#     plt.plot(rx, ry, 'ko')

#     print(len(path), start)
#     x = [p[0] for p in path]
#     y = [p[1] for p in path]
#     plt.plot(x, y, 'bo')
#     plt.plot([start[0], 0], [start[1], 100], 'go')
#     if show:
#         plt.show()
#     if save:
#         print("saved in "+ name)
#         plt.savefig(name)
#         plt.close('all')

# def save_images(vor_list, points_list, path_list, start_list):
#     for i in range(len(vor_list)):
#         name = "gif_images/"+str(i)
#         plot(vor_list[i], points_list[i], path_list[i], start_list[i], False, True, name)

def create_ros_path(coords):
    path = Path()
    path.header.frame_id = 'odom'  # Set the frame ID for the path

    # Loop through the coordinates and add each one to the path as a PoseStamped message
    for coord in coords:
        pose = PoseStamped()
        pose.header.frame_id = 'odom'  # Set the frame ID for the pose
        pose.pose.position.x = coord[0]/10  # Set the x coordinate
        pose.pose.position.y = coord[1]/10  # Set the y coordinate
        pose.pose.position.z = 0.0  # Set the z coordinate to zero
        pose.pose.orientation.x = 0.0  # Set the orientation to zero
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0
        path.poses.append(pose)

    return path


def astar(start, goal, edges, old_path):
    # initialize the open and closed sets
    open_set = [(0, start)]
    closed_set = set()
    # initialize the g and f scores
    g = {start: 0}
    f = {start: heuristic(start, goal)}
    # initialize the dictionary to store the path
    came_from = {}
    # start the search
    while open_set:
        # get the node with the lowest f score
        current_f, current = heapq.heappop(open_set)
        # check if we've reached the goal
        if current == goal:
            return reconstruct_path(start, goal, came_from)
        # add the current node to the closed set
        closed_set.add(current)
        # explore the neighbors of the current node
        for neighbor, gap in edges.get(current, []):
            # check if the neighbor is already in the closed set
            if neighbor in closed_set or gap < 4: # 5 is hard limit
                continue
            # if current!=start:
                # prev = came_from[current] # discard if angle too sharp
                # cos_angle = cosine_angle(prev,neighbor, current)
                # if cos_angle > -0.2 and gap < 4 and math.dist(prev, neighbor) < 7:
                # # if cos_angle > -0.2:
                #     print("discarded")
                #     continue
            # calculate the tentative g score for the neighbor
            tentative_g = g[current] + manhattan_distance(current, neighbor)
            # check if we've already evaluated this neighbor
            if neighbor not in g or tentative_g < g[neighbor]:
                # update the g and f scores for the neighbor
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal) + 800/gap

                if old_path != None:
                    d = distance_to_nearest_point(neighbor, old_path)
                    x = len(came_from) +1
                    f[neighbor] += d * 20 * math.exp(-1/5 * (x)) # adjust here

                # update the path dictionary
                came_from[neighbor] = current
                # add the neighbor to the open set
                heapq.heappush(open_set, (f[neighbor], neighbor))
    # if we've exhausted all possible paths and haven't found the goal, return None
    return None

def reconstruct_path(start, goal, came_from):
    path = [goal]
    while path[-1] != start:
        path.append(came_from[path[-1]])
    return list(reversed(path))

def heuristic(node, goal):
    # in this implementation, we use the Manhattan distance as the heuristic
    return manhattan_distance(node, goal)

def manhattan_distance(node1, node2):
    # calculate the Manhattan distance between two nodes
    return abs(node1[0] - node2[0]) + abs(node1[1] - node2[1])

def distance_to_nearest_point(point, point_list):
    point_array = np.array(point)
    point_list_array = np.array(point_list)
    distances = np.sum(np.abs(point_array - point_list_array), axis=1)
    min_distance = np.min(distances)
    return min_distance


def smooth_curve(points):
    x = [p[0] for p in points]
    y = [p[1] for p in points]
    tck, u = interpolate.splprep([x, y], s=0.5)

    x_i, y_i = interpolate.splev(np.linspace(0, 1, 1000), tck)
    path = [(x_i[i], y_i[i]) for i in range(len(x_i))]

    # plt.plot(x_i, y_i, 'r')


    return path

def connected_path(points):
    # Initialize a list to store the filled in points
    filled_points = []

    # Generate line segments between each consecutive pair of points
    for i in range(len(points)-1):
        p1, p2 = points[i], points[i+1]
        xs = np.linspace(p1[0], p2[0], num=10, endpoint=True)
        ys = np.linspace(p1[1], p2[1], num=10, endpoint=True)
        segment_points = list(zip(xs, ys))
        filled_points.extend(segment_points)
    return filled_points


def generate_arc_points(min_angle, max_angle, radius, num_points):
    # Determine the angle step size for generating points
    angle_step = (max_angle - min_angle) / float(num_points-1)
    
    # Generate points along the arc
    points = []
    for i in range(num_points):
        angle = min_angle + i*angle_step
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        points.append((x,y))
        
    return points

def generate_ellipse_arc(min_axis, max_axis, min_angle, max_angle, num_points):
    # Ensure that the angle range is between 0 and 2*pi
    full_circle = 2 * math.pi
    angle_range = ((max_angle - min_angle) % full_circle + full_circle) % full_circle
    if angle_range == 0:
        angle_range = full_circle
    
    # Calculate the step size between each point
    step = angle_range / (num_points - 1)
    if min_angle > max_angle:
        step *= -1
    
    # Generate a list of points that follow the ellipse arc
    points = []
    for i in range(num_points):
        angle = min_angle + i * step
        x = min_axis * math.cos(angle)
        y = max_axis * math.sin(angle)
        points.append((x, y))
    
    # Reverse the list of points if they were generated clockwise
    if step < 0:
        points.reverse()
    
    return points


def closest_path(start, goal, edge_map, old_path):
    curr = start
    path = [curr]

    if old_path is None:
        return None

    for i in range(1, len(old_path)):
        neighbors = edge_map.get(curr, [])
        # find closest neighbor to old_path[i]
        min_dist = 100000
        min_point = None
        min_gap = 0
        for n, gap in neighbors:
            dist = math.dist(n, old_path[i])
            if dist < min_dist:
                min_dist = dist
                min_point = n
                min_gap = gap
        # if curr is closer to old_path[i] than min_point, continue
        if math.dist(curr, old_path[i]) < min_dist:
            continue
        if min_gap < 4: # path is infeasible
            return None
        curr = min_point

        path.append(curr)

    path.append(goal)
    return old_path