# cs146_p3
from heapq import heappop, heappush
from math import sqrt

def dijkstras_shortest_path(initial_position, destination, graph, adj, initial_xy, dest_xy):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.
    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.
        initial_xy: The initial xy coordinates within the initial_position cell
        dest_xy: The destination xy coordinates witihin the destination cell
    Returns:
        If a path exists, return a list containing all cells from initial_position to destination.
        Otherwise, return None.
    """
    # heuristic just uses euclidian distance
    def heuristic (curr, dest):
        return vector_dist (curr, dest)

    forward_dist = {initial_position: 0}                    # Distance from initial_position when searching "forward"
    backward_dist = {destination: 0}                        # Distance from destination when searching "backward"
    forward_prev = {initial_position: None}                 # Back links from the "forward" direction
    backward_prev = {destination: None}                     # Back links from the "backward" direction
    queue = [(0, initial_position, 'destination')]          # The heap/priority queue used
    heappush(queue, (1, destination, 'initial_position'))
    f_detail_points = {initial_position: initial_xy}        # Holds the entry point into each cell
    b_detail_points = {destination: dest_xy}

    explored_boxes = []

    while queue:
        # Continue with next min unvisited node
        current_distance, current_node, goal = heappop(queue)
        explored_boxes.append (current_node)
        # If we've reached the opposite frontier
        if (goal == 'destination' and current_node in backward_prev) or (goal == 'initial_position' and current_node in forward_prev):
            node = current_node
            # Build the path from the final point in the backward frontier
            # to the src node
            point_path = [b_detail_points[current_node]]
            while node is not None:
                point_path.append(f_detail_points[node])
                node = forward_prev[node]
            # This path goes from end to beginning, so reverse it
            point_path.reverse()

            # Now append the path from the final point in the backward frontier
            # to the destination node
            node = backward_prev[current_node]
            while node is not None:
                point_path.append(b_detail_points[node])
                node = backward_prev[node]

            # format of point_path is [((x1,y1), (x2,y2)), ((x2,y2), (x3,y3)), ((x3,y3), (x4,y4))...]
            # This can be created by zipping point_path with point_path[1:] (which throws away the first element)
            point_path = list(zip(point_path, point_path[1:]))
            return (point_path, explored_boxes)

        # Assigning the various variables to the values they should be depending on
        # which way we are going
        is_dest = goal == 'destination'

        detail_points = f_detail_points if is_dest else b_detail_points
        dist = forward_dist if is_dest else backward_dist
        prev = forward_prev if is_dest else backward_prev
        dest = dest_xy if is_dest else initial_xy
        dest_id = goal
        # Calculate tentative distances to adjacent cells
        for adjacent_node, edge_cost, detail_point in adj(graph, current_node, detail_points[current_node]):
            new_distance = dist[current_node] + edge_cost

            if adjacent_node not in dist or new_distance < dist[adjacent_node]:
                # Assign new distance and update link to previous cell
                dist[adjacent_node] = new_distance
                prev[adjacent_node] = current_node
                # For priority, use distance + heuristic
                heappush(queue, (new_distance + heuristic (detail_point, dest), adjacent_node, dest_id))
                detail_points[adjacent_node] = detail_point

    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None

def navigation_edges (mesh, box, current_point):
    result = []
    for adj_box in mesh['adj'][box]:
        closest_point , dist = shortest_path_to_segment (current_point, get_border(box, adj_box))
        result.append ((adj_box, dist, closest_point))
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
    return path

# Adapted from the project description
def get_border (box1, box2):
    """ Finds the line segment where box1 and box2 overlap
    Args:
        box1: The first box
        box2: The second box
    Returns:
        Returns the line segment where box1 and box2 overlap
    """
    b1x1, b1x2, b1y1, b1y2 = box1
    b2x1, b2x2, b2y1, b2y2 = box2
    xborder = (max (b1x1, b2x1), min (b1x2, b2x2))
    yborder = (max (b1y1, b2y1), min (b1y2, b2y2))
    is_xborder = xborder[1] - xborder[0] > 0
    is_yborder = yborder[1] - yborder[0] > 0
    if is_xborder:
        segment = ((xborder[0], yborder[0]), (xborder[1], yborder[0]))
    elif is_yborder:
        segment = ((xborder[0], yborder[0]), (xborder[0], yborder[1]))
    else:
        segment = ((xborder[0], yborder[0]),(xborder[0], yborder[0]))

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
    """ Performs vector subtraction on the two vectors
    Args:
        p1: The first vector. An (x,y) tuple
        p2: The second vector. An (x,y) tuple
    Returns:
        Returns the difference of the two vectors. p1 - p2
    """
    x1,y1 = p1
    x2,y2 = p2
    return (x1-x2, y1-y2)

def vector_add (p1, p2):
    """ Performs vector addition on the two vectors
    Args:
        p1: The first vector. An (x,y) tuple
        p2: The second vector. An (x,y) tuple
    Returns:
        Returns the sum of the two vectors
    """
    x1,y1 = p1
    x2,y2 = p2
    return (x1+x2, y1+y2)

def vector_scalar_multiply (scalar, vector):
    """ Gives the scalar product of the given scalar and vector
    Args:
        scalar: The scalar value to multiple vector by
        vector: The vector to be multiplied. An (x,y) tuple.
    Returns:
        Returns the scalar product of scalar and vector
    """
    x,y = vector
    return (scalar * x, scalar * y)

def vector_dot (p1, p2):
    """ Gives the vector dot product of the two vectors
    Args:
        p1: The first vector. An (x,y) tuple
        p2: The second vector. An (x,y) tuple
    Returns:
        Returns dot product of the two vectors
    """
    x1,y1 = p1
    x2,y2 = p2
    return x1*x2 + y1*y2

def vector_dist (p1, p2):
    """ Gives the euclidian distance between p1 and p2
    Args:
        p1: The first point. An (x,y) tuple
        p2: The second point. An (x,y) tuple
    Returns:
        Returns the euclidian distance.
    """
    x1,y1 = p1
    x2,y2 = p2
    return sqrt ((x2 - x1)**2 + (y2 - y1)**2)
