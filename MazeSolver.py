#region VEXcode Generated Robot Configuration
import math
import random
from vexcode_vr import *

# Brain should be defined by default
brain=Brain()

drivetrain = Drivetrain("drivetrain", 0)
pen = Pen("pen", 8)
pen.set_pen_width(THIN)
left_bumper = Bumper("leftBumper", 2)
right_bumper = Bumper("rightBumper", 3)
front_eye = EyeSensor("frontEye", 4)
down_eye = EyeSensor("downEye", 5)
front_distance = Distance("frontdistance", 6)
distance = front_distance
magnet = Electromagnet("magnet", 7)
location = Location("location", 9)

#endregion VEXcode Generated Robot Configuration
# ------------------------------------------
# 
# 	Project:      MazeSolver
#	  Author:       Sam Robbins
#	  Created:      06/01/2025
#   Last edited:  20/01/2025
#	  Description:  VEXcode VR Python Project that solves the dynamic maze.
# 
# ------------------------------------------

import heapq
from collections import defaultdict
import time

TILE_DISTANCE = 250
COLUMNS = 8
ROWS = 8
START_COORDS = (4, 7)
DEST_COORDS = (3, 0)
DIRECTIONS = [(0, -1), (1, 0), (0, 1), (-1, 0)]  # Change in coords for each cardinal direction
global exit_is_found


class Node:
    def __init__(self):
        self.prev_node_x = 0
        self.prev_node_y = 0
        self.total_cost = float('inf')
        self.start_cost = float('inf')  # Cost from the start of path to this cell
        self.dest_cost = 0  # Cost from this cell to the destination


def map_maze():
    """Recursively moves the robot through the maze creating a graph representation."""
    # The graph is a dictionary with each key being the coordinate of a square.
    # The list value are the coordinates of the connected nodes.
    graph = defaultdict(list)

    x = START_COORDS[0]
    y = START_COORDS[1]
    key = (x * 10) + y
    
    # The previously visited squares
    visited = {key}
    
    global exit_is_found
    exit_is_found = False

    # Start square, check West then north then east for connected nodes
    search_nodes(graph, visited, x, y)
    
    return graph


def search_nodes(graph, visited, x, y):
    """Recursive function that populates the given graph with all connected tiles."""
    # Check if the exit has been found
    global exit_is_found
    if not exit_is_found:
        exit_is_found = down_eye.detect(RED)
    
    for i in range(4):
        h = 90 * i
        new_x = x + DIRECTIONS[i][0]
        new_y = y + DIRECTIONS[i][1]

        old_key = (x * 10) + y
        new_key = (new_x * 10) + new_y

        # Check if new x and y are out of bounds
        if new_x < 0 or new_x >= 8 or new_y < 0 or new_y >= 8:
            continue

        # Check if already been to that position in ethier direction
        if not visited.isdisjoint({new_key}):
            continue

        # Don't continue if the exit is already found
        if exit_is_found:
            continue

        drivetrain.turn_to_heading(h, DEGREES, wait=True)

        if not front_distance.get_distance(MM) < 100:
            graph[old_key].append(new_key)
            visited.add(new_key)
            drivetrain.drive_for(FORWARD, TILE_DISTANCE, MM)
            
            search_nodes(graph, visited, new_x, new_y)
            
            drivetrain.turn_to_heading(h, DEGREES, wait=True)
            drivetrain.drive_for(REVERSE, TILE_DISTANCE, MM)


def a_star(graph):
    """Uses the A* algorithm to find the shortest path in the given graph."""
    # Populate the list of nodes
    nodes = [[Node() for _ in range(COLUMNS)] for _ in range(ROWS)]

    x = START_COORDS[0]
    y = START_COORDS[1]

    nodes[y][x].prev_node_x = x
    nodes[y][x].prev_node_y = y
    nodes[y][x].total_cost = 0
    nodes[y][x].start_cost = 0
    nodes[y][x].dest_cost = 0

    visited = set()
    # List of nodes to search in tuple form (float, x, y) where float is the hueristic score 
    # (most likely to be closest to the goal)
    to_search = []
    heapq.heappush(to_search, (0.0, x, y))

    has_found_dest = False

    while len(to_search) > 0:
        # Pop the node with the smallest total cost
        node_details = heapq.heappop(to_search)

        # Set x y of current node
        x = node_details[1]
        y = node_details[2]

        # Mark node as visited
        key = (x * 10) + y
        visited.add(key)

        # Skip if key is not in the graph (a dead end)
        if key not in graph:
            continue

        # Search all connected nodes with current
        for next_node_key in graph[key]:
            # Get the next nodes x y from its key
            next_x = math.trunc(next_node_key / 10)
            next_y = next_node_key - (next_x * 10)

            # Ignore visited nodes
            if next_node_key in visited:
                continue

            # Check if the next node is the destination
            if next_x == DEST_COORDS[0] and next_y == DEST_COORDS[1]:
                nodes[next_y][next_x].prev_node_x = x
                nodes[next_y][next_x].prev_node_y = y
                # Create the path from the shortest nodes
                path = create_shortest_path(nodes)
                return path
            else:
                # Add cost from start (always one tile from last)
                start_cost = nodes[y][x].start_cost + 1
                dest_cost = distance_heuristic(next_x, next_y)
                total_cost = start_cost + dest_cost

                # Only if this nodes cost hasn't been calculated or if this paths cost is lower
                if nodes[next_y][next_x].total_cost == float('inf') or nodes[next_y][next_x].total_cost > total_cost:
                    # Add node to search
                    heapq.heappush(to_search, (total_cost, next_x, next_y))
                    # Update node
                    nodes[next_y][next_x].prev_node_x = x
                    nodes[next_y][next_x].prev_node_y = y
                    nodes[next_y][next_x].total_cost = total_cost
                    nodes[next_y][next_x].start_cost = start_cost
                    nodes[next_y][next_x].dest_cost = dest_cost


def distance_heuristic(x, y):
    """Calculate the Euclidean Distance (diagonal straight line) from the current position to the destination."""
    return ((x - DEST_COORDS[0]) ** 2 + (y - DEST_COORDS[1]) ** 2) ** 0.5
       
        
def create_shortest_path(nodes):
    """Traverses the node map proudced by the a* algorithm and returns a list of coordinates for the robot to move to."""
    path = []
    x = DEST_COORDS[0]
    y = DEST_COORDS[1]

    # Backtrack the path from the destination through the previous values 
    while not (nodes[y][x].prev_node_x == x and nodes[y][x].prev_node_y == y):
        path.append((x, y))
        prev_x = nodes[y][x].prev_node_x
        prev_y = nodes[y][x].prev_node_y
        x = prev_x
        y = prev_y

    # Reverse to get the path from source to destination
    path.reverse()

    return path


def follow_path(path):
    """Moves the robot based on the given path."""
    prev_x = START_COORDS[0]
    prev_y = START_COORDS[1]
    for coords in path:
        x = coords[0]
        y = coords[1]
        # Calculate change in coords
        change = (x - prev_x, y - prev_y)
        # Get the index of the change in directions
        index = DIRECTIONS.index(change)
        # Times this by 90 Degrees to get the heading
        h = index * 90

        # Move robot
        drivetrain.turn_to_heading(h, DEGREES, wait=True)
        drivetrain.drive_for(FORWARD, TILE_DISTANCE, MM, wait=True)
        
        # Update previous position
        prev_x = x
        prev_y = y
    

def main():
    drivetrain.set_drive_velocity(100, PERCENT)
    drivetrain.set_turn_velocity(100, PERCENT)

    start = time.time()
    graph = map_maze()
    end = time.time()

    total_time = end - start
    brain.print(f'Exploration phase completed in: {end - start}s\n\n')

    start = time.time()
    path = a_star(graph)
    end = time.time()

    total_time += end - start
    brain.print(f'Shortest path phase completed in: {end - start}s\n\n')

    pen.set_pen_color(GREEN)
    pen.move(DOWN)
    start = time.time()
    follow_path(path)
    end = time.time()

    total_time += end - start
    brain.print(f'Follow path phase completed in: {end - start}\n\n')
    brain.print(f'Routine complete in a total time of: {total_time}s\n\n')


# VR threads — Do not delete
vr_thread(main)
