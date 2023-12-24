# Basic searching algorithms
from util import Stack, Queue, PriorityQueueWithFunction, PriorityQueue
import numpy as np
from collections import deque
import math
import heapq

# Class for each node in the grid
class Node:
    def __init__(self, row, col, cost, parent, goal):
        self.row = row        # coordinate
        self.col = col        # coordinate
        self.cost = cost      # total cost (depend on the algorithm)
        self.goal = goal      # Goal   
        self.parent = parent  # previous node

# Adding Common functions to support the code -- HELPER FUNCTIONS

def is_grid_valid(grid, x,y):
    # Checking whether a node (Row and column pair) is in the total grid or not
    validity = 0<=x<len(grid) and 0<=y<len(grid[0]) and (grid[x][y] == 0)
    return validity

def get_node_neighbors(grid,node,type, directions=[[0,1], [1,0],[0,-1],[-1,0]]):
    # Get the neighbours around the given node in the grid based on the direction set
    x,y = node
    neighbors = []
    # For DFS the traversal is reverse 
    if type == "DFS":
        directions = directions[::-1]
    # Get all 4 neighbours and check whether they are in Grid or not
    for dx,dy in directions:
        new_x, new_y = x+dx, y+dy
        if is_grid_valid(grid,new_x, new_y):
            neighbors.append([new_x,new_y])
    return neighbors

def manhattan_heuristic(node, goal):
    # Manhattan Distance - Regular Version
    distance = abs(node[0]-goal[0]) + abs(node[1]-goal[1]) 
    return distance

def manhattan_heuristic_directional(node, goal, directional_constant):
    # Manhattan distance with a directional_constant which just weights the regular manhattan distance
    distance_normal = abs(node[0]-goal[0]) + abs(node[1]-goal[1]) 
    distance = directional_constant * distance_normal
    return distance

def constant_heuristic(node,goal, distance):
    # Constant distance If 0 it becomes approximation of Dijkstra's Algo
    return distance

def euclidean_heuristic(node,goal):
    # Euclidean distance based heuristic
    distance = math.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2)
    return distance

def euclidean_heuristic_directional(node,goal, directional_constant):
    # Weighted Euclidean distance based heuristic
    distance = math.sqrt((node[0]-goal[0])**2 + (node[1]-goal[1])**2)
    distance = distance * directional_constant
    return distance

def chebyshev_heuristic(node,goal):
    # Chebyshev for diagonal based movement
    dx = abs(node[0]-goal[0])
    dy = abs(node[1]-goal[1])
    distance = 8*max(dx,dy)
    return distance

def octile_heuristic(node,goal, diagonal_move_cost,regular_move_cost):
    # Modification of chebyshev - Taking both diagonal and regular moves
    #Hyperparameter
    # diagonal_move_cost = math.sqrt(2)
    # regular_move_cost = 1

    dx = abs(node[0]-goal[0])
    dy = abs(node[1]-goal[1])

    distance = diagonal_move_cost*min(dx,dy) + regular_move_cost * (dx+dy -2*min(dx,dy))

    return distance

    

## HELPER FUNCTIONS ENDS


def bfs(grid, start, goal):
    '''Return a path found by BFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. 
            If no path exists return an empty list [].
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> bfs_path, bfs_steps = bfs(grid, start, goal)
    It takes 10 steps to find a path using BFS
    >>> bfs_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]
    '''
    ### YOUR CODE HERE ###

    # Initialize the visited, queue lists
    visited = [[False for _ in range(len(grid[0]))] for _ in range(len(grid))]
    queue = [[start]]
    visited[start[0]][start[1]] = True
    steps = 0
    found = False
    path = []
    
    # Iterating over the queue
    while queue:
        path = queue.pop(0)
        node = path[-1]

        steps +=1

        if node == goal:
            # If goal is found
            found = True
            break

        # For each neighbour add into the queue if its valid
        for neighbor in get_node_neighbors(grid,node, 'BFS'):
            if not visited[neighbor[0]][neighbor[1]]:
                visited[neighbor[0]][neighbor[1]] = True
                new_path = list(path)
                new_path.append(neighbor)
                queue.append(new_path)
        
    if found:
        print(f"It takes {steps} steps to find a path using BFS")
    else:
        path = []
        print("No path found")
    return path, steps




def dfs(grid, start, goal):
    '''Return a path found by DFS alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> dfs_path, dfs_steps = dfs(grid, start, goal)
    It takes 9 steps to find a path using DFS
    >>> dfs_path
    [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2], [2, 3], [3, 3], [3, 2], [3, 1]]
    '''
    ### YOUR CODE HERE ###
    # Initialize the visited, stack
    visited = [[False for _ in range(len(grid[0]))] for _ in range(len(grid))]
    stack = [[start]]
    steps = 0
    found = False
    path = []

    # Iterating over the stack
    # Here we are using Iterative approach rather than recursive approach
    while stack:
        path = stack.pop()
        node = path[-1]

        steps+=1

        if node == goal:
            # If goal is found
            found=True
            break

        if not visited[node[0]][node[1]] :
            visited[node[0]][node[1]] = True

            # For each neighbour add into the queue if its valid
            for neighbor in get_node_neighbors(grid,node, 'DFS'):
                if not visited[neighbor[0]][neighbor[1]]:
                    new_path = list(path)
                    new_path.append(neighbor)
                    stack.append(new_path)

    if found:
        print(f"It takes {steps} steps to find a path using DFS")
    else:
        path = []
        print("No path found")
    return path, steps



def astar(grid, start, goal):
    '''Return a path found by A* alogirhm 
       and the number of steps it takes to find it.

    arguments:
    grid - A nested list with datatype int. 0 represents free space while 1 is obstacle.
           e.g. a 3x3 2D map: [[0, 0, 0], [0, 1, 0], [0, 0, 0]]
    start - The start node in the map. e.g. [0, 0]
    goal -  The goal node in the map. e.g. [2, 2]

    return:
    path -  A nested list that represents coordinates of each step (including start and goal node), 
            with data type int. e.g. [[0, 0], [0, 1], [0, 2], [1, 2], [2, 2]]. If no path exists return
            an empty list []
    steps - Number of steps it takes to find the final solution, 
            i.e. the number of nodes visited before finding a path (including start and goal node)

    >>> from main import load_map
    >>> grid, start, goal = load_map('maps/test_map.csv')
    >>> astar_path, astar_steps = astar(grid, start, goal)
    It takes 7 steps to find a path using A*
    >>> astar_path
    [[0, 0], [1, 0], [2, 0], [3, 0], [3, 1]]

    # Defining a custom heuristic
    '''
    ### YOUR CODE HERE ###
    # Initialize the open list priority queue for adding the queue and closed set for storing analyzed nodes
    found = False
    steps = 0
    open_list = [(0, [start])]
    closed_set = set()
    path = []

    while open_list:

        _, path = heapq.heappop(open_list)
        node = path[-1]

        steps +=1
        if node == goal:
            found=True
            break
        
        if tuple(node) not in closed_set:
            closed_set.add(tuple(node)) # Appending the explored nodes

            # For each neighbour add into the queue if its valid
            for neighbor in get_node_neighbors(grid, node, type='Astar'):
                if tuple(neighbor) not in closed_set:
                    new_path = list(path)
                    new_path.append(neighbor)
                    g_cost = len(new_path) # each node cost - 1
                    # Different custom heuristics
                    h_cost = manhattan_heuristic(neighbor,goal)  
                    #h_cost = constant_heuristic(neighbor, goal,2)
                    #h_cost = euclidean_heuristic(neighbor,goal)
                    # Uncomment below line for getting the weighted manhattan
                    #h_cost = manhattan_heuristic_directional(neighbor, goal, 4) # # Use this for directional increment of manhattan
                    #h_cost = euclidean_heuristic_directional(neighbor, goal, 4)
                    #h_cost = chebyshev_heuristic(node,goal)
                    f_cost = g_cost + h_cost
                    heapq.heappush(open_list, (f_cost, new_path))               

    if found:
        print(f"It takes {steps} steps to find a path using A*")
    else:
        path = []
        print("No path found")
    return path, steps

# Doctest
if __name__ == "__main__":
    # load doc test
    from doctest import testmod, run_docstring_examples
    # Test all the functions
    testmod()
