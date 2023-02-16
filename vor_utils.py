from scipy.spatial import Voronoi, voronoi_plot_2d
import numpy as np
import random
import math
import heapq
import matplotlib.pyplot as plt

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

def get_edge_map(vor, points, clearance, start, goal):

    map = {}
    ridge_points = vor.ridge_points
    edges = vor.ridge_vertices
    vertices = vor.vertices

    for i in range(len(ridge_points)):
        
        pair = ridge_points[i]

        if math.dist(points[pair[0]], points[pair[1]]) > clearance:
            edge = edges[i]
            if (edge[0]==-1 or edge[1]==-1):
                continue

            node = (vertices[edge[0]][0], vertices[edge[0]][1])
            other = (vertices[edge[1]][0], vertices[edge[1]][1])
            if (map.get(node) == None):
                map[node] = [other]
            else:
                map[node].append(other)

            if (map.get(other) == None):
                map[other] = [node]
            else:
                map[other].append(node)
    min_start = goal
    min_goal = start
    for key in map.keys():
        if (math.dist(start, key)<math.dist(start,min_start)):
            min_start = key
        elif (math.dist(goal, key)<math.dist(goal,min_goal)):
            min_goal = key
    map[start] = [min_start]
    map[min_goal].append(goal)
        
    return map

def plot(vor, points, path, start, show, save, name):
    fig = voronoi_plot_2d(vor)
    rx = [p[0] for p in points]
    ry = [p[1] for p in points]
    plt.plot(rx, ry, 'ko')

    # print(path)
    x = [p[0] for p in path]
    y = [p[1] for p in path]
    plt.plot(x, y, 'bo')
    plt.plot([start[0], 0], [start[1], 10], 'go')
    if show:
        plt.show()
    if save:
        print("saved in "+ name)
        plt.savefig(name)
    plt.close()

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
        for neighbor in edges[current]:
            # check if the neighbor is already in the closed set
            if neighbor in closed_set:
                continue
            # calculate the tentative g score for the neighbor
            tentative_g = g[current] + manhattan_distance(current, neighbor)
            # check if we've already evaluated this neighbor
            if neighbor not in g or tentative_g < g[neighbor]:
                # update the g and f scores for the neighbor
                g[neighbor] = tentative_g
                f[neighbor] = tentative_g + heuristic(neighbor, goal)
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
