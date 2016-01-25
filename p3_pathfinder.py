from heapq import heappop, heappush
from math import sqrt

def dijkstras_shortest_path(initial_position, destination, graph, adj, initial_xy, dest_xy):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exists, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """

    distances = {initial_position: 0}           # Table of distances to cells 
    previous_cell = {initial_position: None}    # Back links from cells to predecessors
    queue = [(0, initial_position)]             # The heap/priority queue used
    detail_points = {initial_position: initial_xy}

    # Initial distance for starting position
    distances[initial_position] = 0

    while queue:
        # Continue with next min unvisited node
        current_distance, current_node = heappop(queue)
        
        # Early termination check: if the destination is found, return the path
        if current_node == destination:
            node = destination
            path = []
            point_path = [dest_xy]
            while node is not None:
                path.append(node)
                point_path.append (detail_points[node])
                node = previous_cell[node]
            point_path.reverse()
            return (list(zip (point_path, point_path[1:])), path[::-1])

        # Calculate tentative distances to adjacent cells
        for adjacent_node, edge_cost, detail_point in adj(graph, current_node, detail_points[current_node]):
            new_distance = current_distance + edge_cost

            if adjacent_node not in distances or new_distance < distances[adjacent_node]:
                # Assign new distance and update link to previous cell
                distances[adjacent_node] = new_distance
                previous_cell[adjacent_node] = current_node
                heappush(queue, (new_distance, adjacent_node))
                detail_points[adjacent_node] = detail_point
                    
    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None

def navigation_edges (mesh, box, current_point):
    result = []
    for adj_box in mesh['adj'][box]:
        dp, dist = shortest_path_to_segment (current_point, get_border(box, adj_box))
        result.append ((adj_box, dist, dp))
    return result


def contains_point (pnt, box):
    x,y = pnt
    x1, x2, y1, y2 = box
    return (x1 < x and x < x2) and (y1 < y and y < y2)

def find_path (src, dest, mesh):
    src_box, dest_box = None, None
    for box in mesh['boxes']:
        if contains_point (src, box):
            src_box = box
        if contains_point (dest, box):
            dest_box = box
    if not src_box or not dest_box:
        print ("Bad source or destination")
        return ([], [])
    path = dijkstras_shortest_path (src_box, dest_box, mesh, navigation_edges, src, dest)
    if not path:
        return ([], [])
    print (len(path[0]), len(path[1]))
    print (path[0], "\n\n\n", path[1])
    return path

def get_border (box1, box2):
    b1x1, b1x2, b1y1, b1y2 = box1
    b2x1, b2x2, b2y1, b2y2 = box2
    xborder = (max (b1x1, b2x1), min (b1x2, b2x2))
    yborder = (max (b1y1, b2y1), min (b1y2, b2y2))
    is_xborder = xborder[1] - xborder[0] > 0
    is_yborder = yborder[1] - yborder[0] > 0
    if is_xborder:
        segment = ((xborder[0], b1y1), (xborder[1], b1y1))
    else:
        segment = ((b1x1, yborder[0]), (b1x1, yborder[1]))
    return segment


# Adapated from dist_Point_to_Segment in:
# http://geomalgorithms.com/a02-_lines.html
def shortest_path_to_segment (entry_point, segment):
    """ Finds the point closest to entry_point on segment
    Args:
        entry_point: Starting point. A tuple containing (x, y)
        segment: A line segment. A tuple containing ((x1, y1), (x2,y2))

    Returns:
        Returns a tuple containing:
            1. The point on the line segment closest to entry_point
            2. The distance between entry_point and the point above
    """
    start, end = segment
    v = vector_subtract (end, start)
    w = vector_subtract (entry_point, start)

    c1 = vector_dot (w, v)
    if c1 <= 0:
        return (start, vector_dist (entry_point, start))

    c2 = vector_dot (v, v)
    if (c2 <= c1):
        return (end, vector_dist (entry_point, end))

    b = float(c1) / float(c2)
    Pb = vector_add (start, vector_scalar_multiply(b, v))
    return (Pb, vector_dist (entry_point, Pb))

def vector_subtract (p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return (x1-x2, y1-y2)

def vector_add (p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return (x1+x2, y1+y2)

def vector_scalar_multiply (scalar, vector):
    x,y = vector
    return (scalar * x, scalar * y)

def vector_dot (p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return x1*x2 + y1*y2

def vector_dist (p1, p2):
    x1,y1 = p1
    x2,y2 = p2
    return sqrt ((x2 - x1)**2 + (y2 - y1)**2)

def midpoint (box):
    x1,x2,y1,y2 = box
    return ((x1 + x2) / float(2), (y1 + y2) / float(2))