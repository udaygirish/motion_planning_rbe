# Standard Algorithm Implementation
# Sampling-based Algorithms RRT

import matplotlib.pyplot as plt
import numpy as np
import math


# Class for each tree node
class Node:
    def __init__(self, row, col):
        self.row = row  # coordinate
        self.col = col  # coordinate
        self.parent = None  # parent node
        self.cost = 0.0  # cost


# Class for RRT
class RRT:
    # Constructor
    def __init__(self, map_array, start, goal):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size

        self.start = Node(start[0], start[1])  # start node
        self.goal = Node(goal[0], goal[1])  # goal node
        self.vertices = []  # list of nodes
        self.found = False  # found flag

    def init_map(self):
        """Intialize the map before each search"""
        self.found = False
        self.vertices = []
        self.vertices.append(self.start)

    def dis(self, node1, node2):
        """Calculate the euclidean distance between two nodes
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            euclidean distance between two nodes
        """
        # ### YOUR CODE HERE ###
        euc_dist = math.sqrt((node1.row-node2.row)**2 + (node1.col-node2.col)**2)
        return euc_dist

    def check_collision(self, node1, node2):
        """Check if the path between two nodes collide with obstacles
        arguments:
            node1 - node 1
            node2 - node 2

        return:
            True if the new node is valid to be connected
        """
        # ### YOUR CODE HERE ###
        x1 = node1.row
        y1 = node1.col
        x2 = node2.row
        y2 = node2.col
        
        map_array = self.map_array
        # if np.any(map_array[min(x1,x2): max(x1,x2)+1, min(y1,y2): max(y1,y2)+1] ==1):
        #     return False

        if x1 < 0 and x1 >= self.size_row or y1<0 and y1>= self.size_col or \
            x2 < 0 and x2 >= self.size_row or y2 <0 and y2>=self.size_col:
            return True 
        
        dx = x2 - x1
        dy = y2 - y1

        steps = max(abs(dx), abs(dy))
        steps = math.ceil(steps)

        if dx ==0 and dy == 0:
            return False 
        
        for i in range(steps +1):
            x = int(x1 + i*dx/steps)
            y = int(y1 + i*dy/steps)

            if map_array[x][y] !=1:
                return False
        return True

    def get_new_point(self, goal_bias):
        """Choose the goal or generate a random point
        arguments:
            goal_bias - the possibility of choosing the goal
                        instead of a random point

        return:
            point - the new point
        """
        # ### YOUR CODE HERE ###
        x1 = np.random.randint(0, self.size_row)
        y1 = np.random.randint(0, self.size_col)
        random_Node = Node(x1,y1)
        prob_point = np.random.choice([self.goal, random_Node], p= [goal_bias, 1-goal_bias])

        return prob_point

    def get_nearest_node(self, point):
        """Find the nearest node in self.vertices with respect to the new point
        arguments:
            point - the new point

        return:
            the nearest node
        """
        # ### YOUR CODE HERE ###
        min_d = math.inf 
        for vertex in self.vertices:
            temp_dis = self.dis(vertex, point)
            if temp_dis < min_d:
                min_d = temp_dis
                nearest_node = vertex 

        return nearest_node
    

    def extend(self, q_new, q_nearest):
        E_DIST = 15
        dist = self.dis(q_new, q_nearest)
        x_new = q_new.row
        y_new = q_new.col
        x_nearest = q_nearest.row
        y_nearest = q_nearest.col 

        dx = x_new - x_nearest
        dy = y_new - y_nearest
        if (dist <=10) and (x_new != self.goal.row) and (y_new != self.goal.col):
            return q_new 
        else:
            x = x_nearest + dx*(E_DIST/dist)
            y = y_nearest + dy*(E_DIST/dist)

            if x<0:
                x =0
            elif x> self.size_row:
                x = self.size_row -1 
            if y<0:
                y =0
            elif y> self.size_col:
                y = self.size_col - 1 
            
        q = Node(x,y)
        q.parent= q_nearest
        q.cost = q_nearest.cost + self.dis(q, q_nearest)
        return q
            
            


    def get_neighbors(self, new_node, neighbor_size):
        """Get the neighbors that are within the neighbor distance from the node
        arguments:
            new_node - a new node
            neighbor_size - the neighbor distance

        return:
            neighbors - list of neighbors that are within the neighbor distance
        """
        # ### YOUR CODE HERE ###
        neighbor_list = []
        for vertex in self.vertices:
            temp_dis = self.dis(new_node, vertex)
            if temp_dis < neighbor_size:
                neighbor_list.append(vertex)

        return neighbor_list

    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots(1)
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw Trees or Sample points
        for node in self.vertices[1:-1]:
            plt.plot(node.col, node.row, markersize=3, marker="o", color="y")
            plt.plot(
                [node.col, node.parent.col],
                [node.row, node.parent.row],
                color="y",
            )

        # Draw Final Path if found
        if self.found:
            cur = self.goal
            while cur.col != self.start.col or cur.row != self.start.row:
                plt.plot(
                    [cur.col, cur.parent.col],
                    [cur.row, cur.parent.row],
                    color="b",
                )
                cur = cur.parent
                plt.plot(cur.col, cur.row, markersize=3, marker="o", color="b")

        # Draw start and goal
        plt.plot(
            self.start.col, self.start.row, markersize=5, marker="o", color="g"
        )
        plt.plot(
            self.goal.col, self.goal.row, markersize=5, marker="o", color="r"
        )

        # show image
        plt.show()

    def RRT(self, n_pts=1000):
        """RRT main search function
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        In each step, extend a new node if possible,
        and check if reached the goal
        """
        # Remove previous result
        self.init_map()

        # ### YOUR CODE HERE ###

        # In each step,
        # get a new point,
        # get its nearest node,
        # extend the node and check collision to decide whether to add or drop,
        # if added, check if reach the neighbor region of the goal.

        # Output
        goal_bias = 0.05
       
        for n in range(n_pts):
            
            q_new = self.get_new_point(goal_bias)
           

            q_nearest = self.get_nearest_node(q_new)

            q_new = self.extend(q_new, q_nearest)

            if self.check_collision(q_new, q_nearest)==True:
                q_new.parent = q_nearest
                q_new.cost = q_nearest.cost + self.dis(q_nearest, q_new)
                self.vertices.append(q_new)
                if q_new == self.goal:
                    self.found = True
                    break
            if(self.dis(q_new, self.goal)<=20 and self.check_collision(q_new, self.goal) == True):
                self.found = True
                self.goal.parent = q_new
                self.goal.cost = q_new.cost + self.dis(q_new, self.goal)
                break        

        if self.found:
            steps = len(self.vertices) - 2
            length = self.goal.cost
            print("It took %d nodes to find the current path" % steps)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        # Draw result
        self.draw_map()
