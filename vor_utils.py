from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import random
import math
import heapq
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import scipy.interpolate as interpolate
from scipy.spatial.distance import directed_hausdorff
import matplotlib
from scipy.spatial.distance import cdist
import yaml


# Load the YAML file
with open("config/params.yaml", 'r') as stream:
    try:
        params = yaml.safe_load(stream)
    except yaml.YAMLError as e:
        print(e)


def get_yaw(odom_msg):
    # Extract the yaw angle from the orientation quaternion
    qx = odom_msg.pose.pose.orientation.x
    qy = odom_msg.pose.pose.orientation.y
    qz = odom_msg.pose.pose.orientation.z
    qw = odom_msg.pose.pose.orientation.w
    yaw = math.atan2(2 * (qw*qz + qx*qy), 1 - 2 * (qy*qy + qz*qz))

    return yaw


def ranges_to_coordinates(ranges, angle_min, angle_increment, yaw):
    """
    Converts a list of LaserScan ranges to x-y coordinates.
    """
    coords = []
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        if angle < -1.57 or angle > 1.57:
            continue

        if not math.isinf(ranges[i]):
            if ranges[i] > 5:
                continue
            x = ranges[i] * math.cos(angle + yaw)
            y = ranges[i] * math.sin(angle + yaw)
            coords.append((x, y))
    return coords

def ranges_to_coordinates2(ranges, angle_min, angle_increment, yaw):
    """
    Converts a list of LaserScan ranges to x-y coordinates.
    """
    coords = []
    for i in range(len(ranges)):
        angle = angle_min + i * angle_increment
        if angle < -2.3561899662017822 or angle > 2.3561899662017822:
            continue
        if not math.isinf(ranges[i]):
            x = (ranges[i]+.005) * math.cos(angle)
            y = (ranges[i]+.005) * math.sin(angle)
            coords.append((x, y))
        else:
            x = 5 * math.cos(angle)
            y = 5 * math.sin(angle)
            coords.append((x, y))
    return coords

def translate_and_scale(points, shift_x, shift_y, map, discretization = 2):
    for i in range(len(points)):
        x, y = points[i]
        x += -0.055 + shift_x # hardcoded laser -> base_link
        y += shift_y
        (a, b) = (round(x*10), round(y*10))
        x = round(x  * 10 / discretization) *discretization   # scale by 10, discretize by 2, make it an int
        y = round(y * 10 /discretization) *discretization
        points[i] = (x, y)
        map.setdefault((x, y), set()).add((a, b))
    return points, map

def correct_obstacles(old_obstacles, ranges, angle_min, increment, shift_x, shift_y, yaw):
    start = (shift_x*10, shift_y*10)
    points = translate_and_scale(ranges_to_coordinates2(ranges, angle_min, increment, yaw), shift_x, shift_y)
    polygon = matplotlib.path.Path([start]+points)
    new_points = []
    for p in old_obstacles:
        if not polygon.contains_point(p):
            new_points.append(p)
    return new_points

def get_min_dist(points1, points2):
    min_dist = 100000000
    for p1 in points1:
        for p2 in points2:
            dist = math.dist(p1, p2)
            if dist < min_dist:
                min_dist = dist
    return min_dist

def get_gap(point_map, point1, point2):
    points1 = point_map[point1]
    points2 = point_map[point2]
    return get_min_dist(points1, points2)



def get_edge_map(vor, start, goal, point_map, circle_points = set()):
    circle_points = set(circle_points)
    map = {}
    ridge_points = vor.ridge_points
    edges = vor.ridge_vertices
    vertices = vor.vertices
    points = vor.points

    for i in range(len(ridge_points)):
        
        pair = ridge_points[i]

        point1 = tuple(points[pair[0]])
        point2 = tuple(points[pair[1]])
        # gap = math.dist(point1, point2)

        points1 = point_map.get(point1, [point1])
        points2 = point_map.get(point2, [point2])

        gap = get_min_dist(points1, points2)

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


def astar(start, goal, edges):
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
            if neighbor in closed_set or gap < 4.5: # 5 is hard limit
                continue
            # calculate the tentative g score for the neighbor
            tentative_g = g[current] + math.dist(current, neighbor)
            # check if we've already evaluated this neighbor
            if neighbor not in g or tentative_g < g[neighbor]:
                # update the g and f scores for the neighbor
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal) + 800/gap
                # update the path dictionary
                came_from[neighbor] = (current, gap)
                # add the neighbor to the open set
                heapq.heappush(open_set, (f[neighbor], neighbor))
    # if we've exhausted all possible paths and haven't found the goal, return None
    return None

def reconstruct_path(start, goal, came_from):
    min_gap = 100
    avg_gap = 0
    count = 0
    path = [goal]
    while path[-1] != start:
        node, gap = came_from[path[-1]]
        if gap < min_gap:
            min_gap = gap
        if gap < 50:
            avg_gap += gap
            count += 1
        path.append(node)
    return list(reversed(path)), avg_gap/count
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

        
def cosine_angle(p1, p2, p3):
    """
    Calculates the cosine of the angle between the vectors formed by p1-p2 and p3-p2.
    """
    v1 = (p1[0]-p2[0], p1[1]-p2[1])
    v2 = (p3[0]-p2[0], p3[1]-p2[1])
    dot_product = v1[0]*v2[0] + v1[1]*v2[1]
    mag_v1 = math.sqrt(v1[0]**2 + v1[1]**2)
    mag_v2 = math.sqrt(v2[0]**2 + v2[1]**2)
    cosine = dot_product / (mag_v1 * mag_v2)
    return cosine


def path_distance(points):
    # initialize total distance to 0
    total_distance = 0
    
    # iterate through the list of points, starting from the second point
    for i in range(0, len(points)):

        # calculate the distance between the current point and the previous point
        distance = math.sqrt((points[i][0] - points[i-1][0])**2 + (points[i][1] - points[i-1][1])**2)
        
        # add the distance to the total distance
        total_distance += distance
    
    # return the total distance
    return total_distance


def copy_astar_path(start, goal, edges, old_path):
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
            if neighbor in closed_set or gap < 4.5: # 5 is hard limit
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

                d = distance_to_nearest_point(neighbor, old_path)

                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal) + d*50

                # update the path dictionary
                came_from[neighbor] = (current, gap)
                # add the neighbor to the open set
                heapq.heappush(open_set, (f[neighbor], neighbor))
    # if we've exhausted all possible paths and haven't found the goal, return None
    return None

def generate_circle_points(center, radius, num):
    circle_points = []
    for i in range(num):
        angle = math.radians(i * (360.0 / num))
        x = center[0] + (radius * math.cos(angle))
        y = center[1] + (radius * math.sin(angle))
        circle_points.append((x, y))
    return circle_points

def switch_plan(new_avg_gap, avg_gap, old_path, new_path, hd):
    gap_size_hysteresis = params['switching_cond']['gap_size_hysteresis']
    gap_size_sanity = params['switching_cond']['gap_size_sanity']
    old_path_length = params['switching_cond']['old_path_length']
    hausdorff_dist = params['switching_cond']['hausdorff_dist']
    path_dist_scale = params['switching_cond']['path_dist_scale']

    return new_avg_gap > gap_size_hysteresis* avg_gap and len(old_path) >old_path_length \
    or hd>hausdorff_dist \
    or (path_distance(new_path) < path_dist_scale*path_distance(old_path) and new_avg_gap > gap_size_sanity*avg_gap)