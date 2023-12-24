import numpy as np
from scipy import spatial

from sampling_method import SamplingMethod
from utils import angle_diff, wrap_to_pi


class Node:
    """Class for each tree node"""

    def __init__(self, config):
        self.config = config  # configuration
        self.parent = None  # parent node / edge
        self.cost = 0.0  # cost to parent / edge weight

class RRT(SamplingMethod):
    """RRT/RRT* class"""

    def __init__(self, sampling_method, n_configs, kdtree_d=10):
        """Specifying number of configs and sampling method to initialize
        arguments:
            sampling_method - name of the chosen sampling method
            n_configs - number of configs to sample
            kdtree_d - the distance of a node to be considered as a neighbor
        """
        super().__init__()
        self.sampling_method = sampling_method
        self.n_configs = n_configs
        self.kdtree_d = kdtree_d

        # kd tree spatial.KDTree([[]]), euclidean distance only
        self.kdtree = None
        self.samples = []  # list of sampled config nodes
        self.solution = []  # list of nodes of the found solution


    def init_map(self):
        """Intialize the map before each search"""
        # Add start and goal nodes
        self.start_node = Node(self.start)
        self.goal_node = Node(self.goal)
        # Initialize the containers
        self.samples = []
        self.solution = []

        # Update samples and kdtree
        self.samples.append(self.start_node)
        self.update_kdtree()

    def update_kdtree(self):
        """Update the kd tree after new node is added"""
        self.kdtree = spatial.cKDTree(
            [node.config for node in self.samples]
        )

    def get_new_point(self, goal_bias):
        """Choose the goal or generate a random point in configuration space
        arguments:
            goal_bias - the possibility of choosing the goal
                        instead of a random point

        return:
            point - a new point in the configuration space
        """
        # Select goal
        if np.random.uniform() < goal_bias:
            point = self.goal
        # Or generate a random point
        else:
            point = []
            # sample in each dimension
            for i in range(self.robot.dof):
                point.append(
                    np.random.uniform(
                        self.robot.limits[i][0], 
                        self.robot.limits[i][1]
                    )
                )
        return point

    def get_nearest_node(self, point):
        """Find the nearest node in self.samples from the new point
        arguments:
            point - the new point in configuration space

        return:
            the nearest node in self.samples
        """
        # Use kdtree to find the neighbors within neighbor size
        _, ind = self.kdtree.query(point)
        return self.samples[ind]

    def extend(self, extension_d, goal_bias=0.05):
        """Extend a new node from the current tree
        arguments:
            extension_d - the extension distance
            goal_bias - the possibility of choosing the goal
                        instead of a random sample point

        Create and add a new node if valid.
        """
        # Generate a new point and find the nearest node
        new_point = self.get_new_point(goal_bias)
        nearest_node = self.get_nearest_node(new_point)

        # Calculate new node location by extending the nearest node
        # compute the direction to move
        diff = np.zeros(self.robot.dof)
        for i in range(self.robot.dof):
            # position
            if (self.robot.limits[i][2] != "r"):
                diff[i] = new_point[i] - nearest_node.config[i]
            # rotation
            else:
                # find the shortest angle
                diff[i] = angle_diff(
                    nearest_node.config[i], new_point[i], absolute=False
                )
        # get unit vector 
        if np.linalg.norm(diff) == 0:
            # same configuration
            return None

        new_config = (
            np.array(nearest_node.config) 
            + diff / np.linalg.norm(diff) * extension_d
        ).tolist()
        # wrap the angle if necessary
        for i in range(self.robot.dof):
            if self.robot.limits[i][2] == "r":
                new_config[i] = wrap_to_pi(new_config[i])

        # Check if the new configuration is valid
        if self.check_collision(new_config, nearest_node.config):
            return None

        # Create a new node
        new_node = Node(new_config)
        cost = self.robot.distance(new_config, nearest_node.config)
        # add the new node to the tree
        new_node.parent = nearest_node
        new_node.cost = cost
        self.samples.append(new_node)
        self.update_kdtree()

        return new_node

    def connect_to_near_goal(self, new_node):
        """Check if the new node is near and has a valid path to goal node
        If yes, connect the new node to the goal node
        """
        # Check if goal is close and there is a valid path
        dis = self.robot.distance(new_node.config, self.goal)
        if (
            dis < self.kdtree_d 
            and not self.check_collision(new_node.config, self.goal)
        ):
            # connect directly to the goal
            self.goal_node.cost = dis
            self.goal_node.parent = new_node
            self.samples.append(self.goal_node)
            self.update_kdtree()

    def get_path_and_cost(self, start_node, end_node):
        """Compute path cost starting from a start node to an end node
        arguments:
            start_node - path start node
            end_node - path end node

        return:
            cost - path cost
        """
        cost = 0
        curr_node = end_node
        path = [curr_node]

        # Keep tracing back
        # until finding the start_node
        # or no path exists
        while curr_node != start_node:
            cost += curr_node.cost
            parent = curr_node.parent
            if parent is None:
                print("There is no path from the given start to goal")
                return [], 0

            curr_node = parent
            path.append(curr_node)

        return path[::-1], cost

    # def get_neighbors(self, new_node):
    #     """Get the neighbors within the neighbor distance from the node
    #     arguments:
    #         new_node - a new node
    #         size - the neighbor size

    #     return:
    #         neighbors - list of neighbors within the neighbor distance
    #     """
    #     ### YOUR CODE HERE ###
        
    #     neighbors = []
    #     for node in self.samples:
    #         temp_distance  = self.robot.distance(node.config, new_node.config)
    #         if node != new_node and temp_distance < self.kdtree_d:
    #             neighbors.append(node)
    #             #self.update_kdtree()

    #     return neighbors
    
    def get_neighbors(self, new_node):
        """Get the neighbors within the neighbor distance from the node
        arguments:
            new_node - a new node
            size - the neighbor size

        return:
            neighbors - list of neighbors within the neighbor distance
        """
        ### YOUR CODE HERE ###
        # Use kd tree to find the neighbors within neighbor size
        samples = [node.config for node in self.samples]
        kd_tree = spatial.cKDTree(samples)
        indices = kd_tree.query_ball_point(new_node.config, self.kdtree_d)
        neighbors = [self.samples[i] for i in indices if self.samples[i] != new_node]
        
        return neighbors

    def rewire(self, new_node, neighbors):
        """Rewire the new node and all its neighbors
        arguments:
            new_node - the new node
            neighbors - a list of neighbors of the new node

        Rewire the new node if connecting to a new neighbor node
        will give least cost.
        Rewire all the other neighbor nodes.
        """
        ### YOUR CODE HERE ###

        # If neighbors == zero return 
        if neighbors == []:
            return

        # Compute the distances between the new node and its neighbors
        node_distances = [self.robot.distance(new_node.config, neighbor.config) for neighbor in neighbors]

        # Rewire the new node 
        # Compute the least cost as mentioned
        costs = []
        for i, dis  in enumerate(node_distances):
            temp_path_cost = self.get_path_and_cost(self.start_node, neighbors[i])[1]
            costs.append(dis + temp_path_cost)
    
        indices_lc = np.argsort(np.array(costs))

        for i in indices_lc:
            if not self.check_collision(new_node.config, neighbors[i].config):
                new_node.parent = neighbors[i]
                new_node.cost = node_distances[i]
                break

        # Rewire the neighbors
        for i, neighbor in enumerate(neighbors):
            new_path_cost = self.get_path_and_cost(self.start_node, new_node)[1] + node_distances[i]

            # if new cost is less than the previous cost no obstacles

            if self.get_path_and_cost(self.start_node, neighbor)[1] > new_path_cost and not self.check_collision(new_node.config, neighbor.config):
                neighbor.parent = new_node
                neighbor.cost = node_distances[i]

        
    def RRT(self):
        """RRT search function
        In each step, extend a new node if possible,
        and check if reached the goal
        """
        # Start searching
        while len(self.samples) < self.n_configs:
            # Extend a new node until
            # all the points are sampled
            # or find a path
            new_node = self.extend(extension_d=self.kdtree_d, goal_bias=0.05)

            # If goal is not found, try to connect new node to goal
            if self.goal_node.parent is None:
                if new_node is not None:
                    self.connect_to_near_goal(new_node)
            # goal found
            else:
                break

        # Output
        if self.goal_node.parent is not None:
            num_nodes = len(self.samples)
            path, length = self.get_path_and_cost(
                self.start_node, self.goal_node
            )
            self.solution = [node.config for node in path]
            print("The constructed tree has %d of nodes" % num_nodes)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        return self.solution

    def RRT_star(self):
        """RRT* search function
        In each step, extend a new node if possible,
        and rewire the node and its neighbors.
        """
        # Start searching
        while len(self.samples) < self.n_configs:
            # Extend a new node
            new_node = self.extend(extension_d=self.kdtree_d, goal_bias=0.05)

            # Rewire the new node and its neighbors
            if new_node is not None:
                neighbors = self.get_neighbors(new_node)
                self.rewire(new_node, neighbors)

            # If goal is not found, try to connect new node to goal
            if self.goal_node.parent is None and new_node is not None:
                self.connect_to_near_goal(new_node)

        # Output
        if self.goal_node.parent is not None:
            num_nodes = len(self.samples)
            path, length = self.get_path_and_cost(
                self.start_node, self.goal_node
            )
            self.solution = [node.config for node in path]
            print("The constructed tree has %d of nodes" % num_nodes)
            print("The path length is %.2f" % length)
        else:
            print("No path found")

        return self.solution

    def Informed_RRT_star(self):
        '''Informed RRT* search function
        In each step, extend a new node if possible, and rewire the node and its neighbors
        Once a path is found, an ellipsoid will be defined to constrained the sampling area
        '''
        ### YOUR CODE HERE ###
        # Start searching
        except_count = 0
        total_exp = 0
        # c_best = float('inf')
        while len(self.samples) < self.n_configs:
            total_exp += 1

            # The below conditions are written to use difference goal biases and extension distances for different robots
            if self.goal_node.parent is not None and self.robot.robot_name != "kinematic_chain":
                c_best = self.get_path_and_cost(self.start_node, self.goal_node)[1]
                new_node = self.extend_inform_rrt(extension_d=self.kdtree_d, goal_bias=0.05, c_best=c_best)
            elif self.goal_node.parent is not None and self.robot.robot_name == "kinematic_chain":
                c_best = self.get_path_and_cost(self.start_node, self.goal_node)[1]
                new_node = self.extend_inform_rrt(extension_d=self.kdtree_d, goal_bias=0.05, c_best=c_best)
            elif self.goal_node.parent is None:
                except_count += 1
                new_node = self.extend(extension_d=self.kdtree_d, goal_bias=0.05)
            else:
                c_best = self.get_path_and_cost(self.start_node, self.goal_node)[1]
                new_node = self.extend_inform_rrt(extension_d=self.kdtree_d, goal_bias=0.05, c_best=c_best)

            if new_node is not None:
                neighbors = self.get_neighbors(new_node)
                self.rewire(new_node, neighbors)

            # If goal is not found, try to connect new node to goal
            if self.goal_node.parent is None and new_node is not None:
                self.connect_to_near_goal(new_node)

        # Output
        if self.goal_node.parent is not None:
            num_nodes = len(self.samples)
            path, length = self.get_path_and_cost(
                self.start_node, self.goal_node
            )
            self.solution = [node.config for node in path]
            print("The constructed tree has %d of nodes" % num_nodes)
            print("The path length is %.2f" % length)
            print("How many times it explored through RRT:",except_count)
            print("Total exploration iterations:",total_exp)
        else:
            print("No path found")
            print("How many times it explored through RRT:",except_count)
            print("Total exploration iterations:",total_exp)

        return self.solution


    def plan(self, start, goal):
        """Search for a path in graph given start and goal location

        arguments:
            start - start configuration
            goal - goal configuration
        """
        self.start = start
        self.goal = goal
        self.init_map()

        if self.sampling_method == "RRT":
            self.solution = self.RRT()
        elif self.sampling_method == "RRT_star":
            self.solution = self.RRT_star()
        elif self.sampling_method == "Informed_RRT_star":
            self.solution = self.Informed_RRT_star()
        else: 
            raise ValueError(f"Sampling method:{self.sampling_method} does not exist!")

        return self.solution

    def visualize_sampling_result(self, ax):
        """ Visualization the sampling result."""
        # Draw Trees / sample points
        for node in self.samples:
            # node
            pos1 = self.robot.forward_kinematics(node.config)[-1]
            ax.plot(
                pos1[0], 
                pos1[1], 
                markersize=5, 
                marker=".", 
                color="g", 
                alpha=0.3
            )
            # edge
            if node == self.start_node:
                continue
            pos2 = self.robot.forward_kinematics(node.parent.config)[-1]
            ax.plot(
                [pos1[0], pos2[0]],
                [pos1[1], pos2[1]],
                color="y",
                alpha=0.3
            )

    def get_new_point_informed_sampling(self, goal_bias, c_best):
        '''
        Choose the goal or generate a random point in an ellipsoid defined by start, goal, 
        and the current best length of the path.

        Args:
            goal_bias: The possibility of choosing the goal instead of a random point.
            c_best: The length of the current best path.

        Returns:
            point: The new point.

        This code is adapted for sampling of n dimensional C Space
        '''

        n = len(self.start)  # Number of dimensions

        # Select goal
        if np.random.random() < goal_bias:
            point = self.goal

        else:
            # Calculate the minimum distance between start and goal
            c_min = self.robot.distance(self.start, self.goal)

            # Compute the center of the ellipsoid
            x_center = [(self.start[i] + self.goal[i]) / 2 for i in range(n)]

            # Compute the rotation matrix from the ellipse to the world frame - C
            direction_vector = [(self.start[i] - self.goal[i]) / c_min for i in range(n)]
            mat = np.array(direction_vector + [0])
            mat = mat.reshape(n+1 , 1)
            identity = np.zeros(n + 1)
            identity[0] = 1
            identity = identity.reshape(1, n+1)
            M = np.dot(mat, identity)
            U, S, V = np.linalg.svd(M, full_matrices=True, compute_uv=True)
            diag_list = []
            for i in range(n):
                diag_list.append(1)
            diag_list.append(np.linalg.det(U) * np.linalg.det(V.T))
            diag = np.diag(diag_list)
            C = np.dot(U, np.dot(diag, V))

            # Compute the diagonal matrix - L
            scaling_factor = 1.5
            r1 = c_best / (2 * scaling_factor)
            r_other = np.sqrt(c_best ** 2 - c_min ** 2) /(2 * scaling_factor)
            L = np.diag([r1] + [r_other] * n)
            # Cast a sample from a unit ball - x_ball
            u = np.random.random()
            v = np.random.random()
            r = u**0.5
            angles = [2 * np.pi * v] + [np.random.uniform(0, 2 * np.pi) for _ in range(n - 1)]
            x_ball = np.array([r * np.cos(theta) for theta in angles] + [0])
            x_ball = x_ball.reshape(n + 1, 1)
            # Map ball sample to the ellipsoid - x_rand
            CLx = np.dot(C, np.dot(L, x_ball))
            x_rand = np.array(x_center) + CLx[:n]

            point_temp = x_rand.tolist()
            point = []
            for i in point_temp:
                point.append(i[0])

        return point

    
    def extend_inform_rrt(self, extension_d, goal_bias, c_best):
        """Extend a new node from the current tree using informed sampling
        arguments:
            extension_d - the extension distance
            goal_bias - the possibility of choosing the goal
                        instead of a random sample point

        Create and add a new node if valid.
        """
        # Generate a new point and find the nearest node
        # Only change this function as this will call informed sampling instead of uniform sampling
        new_point = self.get_new_point_informed_sampling(goal_bias, c_best)
        # print(new_point)

        nearest_node = self.get_nearest_node(new_point)

        # Calculate new node location by extending the nearest node
        # compute the direction to move
        diff = np.zeros(self.robot.dof)
        for i in range(self.robot.dof):
            # position
            if (self.robot.limits[i][2] != "r"):
                diff[i] = new_point[i] - nearest_node.config[i]
            # rotation
            else:
                # find the shortest angle
                diff[i] = angle_diff(
                    nearest_node.config[i], new_point[i], absolute=False
                )
        # get unit vector 
        if np.linalg.norm(diff) == 0:
            # same configuration
            return None

        new_config = (
            np.array(nearest_node.config) 
            + diff / np.linalg.norm(diff) * extension_d
        ).tolist()
        # wrap the angle if necessary
        for i in range(self.robot.dof):
            if self.robot.limits[i][2] == "r":
                new_config[i] = wrap_to_pi(new_config[i])

        # Check if the new configuration is valid
        if self.check_collision(new_config, nearest_node.config):
            return None

        # Create a new node
        new_node = Node(new_config)
        cost = self.robot.distance(new_config, nearest_node.config)
        # add the new node to the tree
        new_node.parent = nearest_node
        new_node.cost = cost
        self.samples.append(new_node)
        self.update_kdtree()

        return new_node

