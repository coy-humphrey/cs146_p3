from heapq import heappop, heappush
from math import sqrt

def dijkstras_shortest_path(initial_position, destination, graph, adj):
    """ Searches for a minimal cost path through a graph using Dijkstra's algorithm.

    Args:
        initial_position: The initial cell from which the path extends.
        destination: The end location for the path.
        graph: A loaded level, containing walls, spaces, and waypoints.
        adj: An adjacency function returning cells adjacent to a given cell as well as their respective edge costs.

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.

    """

    distances = {initial_position: 0}           # Table of distances to cells 
    previous_cell = {initial_position: None}    # Back links from cells to predecessors
    queue = [(0, initial_position)]             # The heap/priority queue used

    # Initial distance for starting position
    distances[initial_position] = 0

    while queue:
        # Continue with next min unvisited node
        current_distance, current_node = heappop(queue)
        
        # Early termination check: if the destination is found, return the path
        if current_node == destination:
            node = destination
            path = [node]
            while node is not None:
                path.append(previous_cell[node])
                node = previous_cell[node]
            return path[::-1]

        # Calculate tentative distances to adjacent cells
        for adjacent_node, edge_cost in adj(graph, current_node):
            new_distance = current_distance + edge_cost

            if adjacent_node not in distances or new_distance < distances[adjacent_node]:
                # Assign new distance and update link to previous cell
                distances[adjacent_node] = new_distance
                previous_cell[adjacent_node] = current_node
                heappush(queue, (new_distance, adjacent_node))
                    
    # Failed to find a path
    print("Failed to find a path from", initial_position, "to", destination)
    return None

def navigation_edges (mesh, box):
	return [(new_box, 1) for new_box in mesh['adj'][box]]


def contains_point (pnt, box):
	x,y = pnt
	x1, x2, y1, y2 = box
	return (x1 < x and x < x2) and (y1 < y and y < y2)

def find_path (src, dest, mesh):
	for box in mesh['boxes']:
		if contains_point (src, box):
			src_box = box
		if contains_point (dest, box):
			dest_box = box
	path = dijkstras_shortest_path (src_box, dest_box, mesh, navigation_edges)
	return ([src, dest], path)

# Adapated from dist_Point_to_Segment in:
# http://geomalgorithms.com/a02-_lines.html
def shortest_path_to_segment (entry_point, segment):
    """ Finds the point closest to entry_point on segment
    Args:
        entry_point: Starting point. A tuple containing (x, y)
        segment: A line segment. A tuple containing ((x1, y1), (x2,y2))

    Returns:
        If a path exits, return a list containing all cells from initial_position to destination.
        Otherwise, return None.
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

    b = c1 / float(c2)
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