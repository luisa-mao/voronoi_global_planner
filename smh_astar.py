from vor_utils import *
import pdb

w1 = 1
w2 = 100

def euclidean_distance(node, goal):
    return round(math.dist(node, goal))

def heuristic1(node, goal, gap, old_path):
    return 100/gap

def heuristic2(node, goal, gap, old_path):
    if (old_path is None):
        return 0
    
    return distance_to_nearest_point(node, old_path)


def a_star(start, goal, edge_map, vor_vertices, old_path):

    def expand_state(s):
        # pdb.set_trace()
        for i in range(len(heuristics)):
            if s in open_sets[i]:
                open_sets[i].remove(s)
        neighbors = edge_map.get(s[1], [])
        for neighbor, gap in neighbors:
            if gap <= 5: # hard limit on gap size
                closed_anchor.add(neighbor)
            if neighbor not in g_scores:
                g_scores[neighbor] = math.inf
                parents[neighbor] = None
            if g_scores[neighbor] > g_scores[s[1]] + euclidean_distance(vor_vertices[s[1]], vor_vertices[neighbor]):
                g_scores[neighbor] = g_scores[s[1]] + euclidean_distance(vor_vertices[s[1]], vor_vertices[neighbor])
                parents[neighbor] = s[1]
                if neighbor not in closed_anchor:
                    # insert/update
                    tup = get_tuple_by_second_element(open_sets[0], neighbor)
                    if tup is not None:
                        open_sets[0].remove(tup)
                    open_sets[0].append(( g_scores[neighbor]+euclidean_distance(vor_vertices[neighbor], vor_vertices[goal]), neighbor))

                    if neighbor not in closed_inad:
                    
                        for i in range(1, len(heuristics)):
                            # insert/update
                            tup = get_tuple_by_second_element(open_sets[i], neighbor)
                            if tup is not None:
                                open_sets[i].remove(tup)
                            open_sets[i].append((heuristics[i](vor_vertices[neighbor], vor_vertices[goal], gap, old_path), neighbor))

    def reconstruct_path2(start, goal, came_from):
        path = [goal]
        while path[-1] != start:
            path.append(came_from[path[-1]])
        path_indices = list(reversed(path))
        path_nodes = []
        for i in path_indices:
            path_nodes.append(vor_vertices[i])
        return path_nodes



    heuristics = [euclidean_distance, heuristic1]
    open_sets = [[(0, start)] for heuristic in heuristics]
    closed_anchor = set()
    closed_inad = set()
    g_scores = {start: 0, goal: math.inf}
    parents = {start: None}
    
    while open_sets[0]:
        for i in range(1, len(heuristics)):
            heapq.heapify(open_sets[i])
            heapq.heapify(open_sets[0])
            # if len(open_sets[i]) == 0 or len(open_sets[0]) == 0:
            #     return old_path
            if len(open_sets[i])>0 and open_sets[i][0][0] <= w2 * open_sets[0][0][0]:
                print("here", open_sets[i][0][0], w2* open_sets[0][0][0])
                if g_scores[goal] <= open_sets[i][0][0]:
                    if g_scores[goal] < math.inf:
                        return reconstruct_path2(start, goal, parents)
                else:
                    s = heapq.heappop(open_sets[i])
                    expand_state(s)
                    closed_inad.add(s[1])
            else:
                if g_scores[goal] <= open_sets[0][0][0]:
                    if (g_scores[goal]) < math.inf:
                        return reconstruct_path2(start, goal, parents)
                else:
                    s = heapq.heappop(open_sets[0])
                    expand_state(s)
                    closed_anchor.add(s[1])

    

def get_tuple_by_second_element(my_set, x):
    for tup in my_set:
        if tup[1] == x:
            return tup
    return None


def get_edge_map2(vor, start):

    map = {}
    ridge_points = vor.ridge_points
    edges = vor.ridge_vertices
    vertices = vor.vertices
    points = vor.points

    # min_dist = math.inf
    # closest_vertex = None

    for i in range(len(ridge_points)):
        
        pair = ridge_points[i]

        point1 = tuple(points[pair[0]])
        point2 = tuple(points[pair[1]])
        gap = math.dist(point1, point2)

        edge = edges[i]
        if (edge[0]==-1 or edge[1]==-1):
            continue

        if (map.get(edge[0]) == None):
            map[edge[0]] = [(edge[1], gap)]
        else:
            map[edge[0]].append((edge[1], gap))

        if (map.get(edge[1]) == None):
            map[edge[1]] = [(edge[0], gap)]
        else:
            map[edge[1]].append((edge[0], gap))

        # if math.dist(start, vertices[edge[0]])<min_dist:
        #     min_dist = math.dist(start, vertices[edge[0]])
        #     closest_vertex = edge[0]
        # if math.dist(start, vertices[edge[1]])<min_dist:
        #     min_dist = math.dist(start, vertices[edge[1]])
        #     closest_vertex = edge[1]

    # pdb.set_trace()
    pts = vor.regions[vor.point_region[0]]
    print(vor.points[0])
    for p in pts:
        if p == -1:
            continue
        if map.get(p) != None:
            map[p].append((len(vor.vertices)+1, 100)) # this is goal

    # map[len(vor.vertices)] = [(closest_vertex, 100)] # this is start

    pts = vor.regions[vor.point_region[1]]
    map[len(vor.vertices)] = []
    for p in pts:
        if p == -1:
            continue
        map[len(vor.vertices)].append((p, 100)) # this is start
    
    return map