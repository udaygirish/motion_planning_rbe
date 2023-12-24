# Standard Algorithm Implementation
# Sampling-based Algorithms PRM

import matplotlib.pyplot as plt
import numpy as np
import networkx as nx
import math
from collections import defaultdict

# Use KD tree
from scipy.spatial import cKDTree # Ckd tree is Cython based - Faster !!

# Class for PRM
class PRM:
    # Constructor
    def __init__(self, map_array):
        self.map_array = map_array  # map array, 1->free, 0->obstacle
        self.size_row = map_array.shape[0]  # map size
        self.size_col = map_array.shape[1]  # map size
        print(self.size_row, self.size_col)
        self.samples = []  # list of sampled points
        self.graph = nx.Graph()  # constructed graph
        self.path = []  # list of nodes of the found path

    def check_collision(self, p1, p2):
        """Check if the path between two points collide with obstacles
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            True if there are obstacles between two points
        """
        # ### YOUR CODE HERE ###

        # Check collision by checking if np.any return true or not
        x1 = p1[0]
        y1 = p1[1]
        x2 = p2[0]
        y2 = p2[1]
        
        map_array = self.map_array


        if x1 < 0 and x1 >= self.size_row or y1<0 and y1>= self.size_col or \
            x2 < 0 and x2 >= self.size_row or y2 <0 and y2>=self.size_col:
            return True 
        
        dx = x2 - x1
        dy = y2 - y1

        steps = max(abs(dx), abs(dy))

        if dx ==0 and dy == 0:
            return False 
        
        for i in range(steps +1):
            x = int(x1 + i*dx/steps)
            y = int(y1 + i*dy/steps)

            if map_array[x][y] !=1:
                return True
        return False

    def dis(self, point1, point2):
        """Calculate the euclidean distance between two points
        arguments:
            p1 - point 1, [row, col]
            p2 - point 2, [row, col]

        return:
            euclidean distance between two points
        """
        # ### YOUR CODE HERE ###  
        euc_dist = math.sqrt((point1[0]-point2[0])**2 + (point1[1]-point2[1])**2)
        return euc_dist  

    def uniform_sample(self, n_pts):
        """Use uniform sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        # ### YOUR CODE HERE ###

        samples = np.random.uniform(low=(0, 0), high=(self.size_row, self.size_col), size=(n_pts,2)).astype(int)
        for i in samples:
            if self.map_array[i[0]][i[1]] == 1:
                self.samples.append(i)

    def gaussian_sample(self, n_pts):
        """Use gaussian sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        # ### YOUR CODE HERE ###
        n_samples = 0
        while n_samples <= n_pts:
            p_a = np.random.randint(low=(0, 0), high=(self.size_row, self.size_col), size=(1,2))
            x_a = p_a[0][0]
            y_a = p_a[0][1]
            mean_dist = 0
            std_dev = 2
            r_sample = np.random.normal(mean_dist, std_dev)
            angle = np.random.uniform(0,2*np.pi)
            x_b = x_a + r_sample * np.cos(angle)
            y_b = y_a + r_sample * np.sin(angle)
            x_b = math.floor(x_b)
            y_b = math.floor(y_b)
            if x_b < self.size_row and y_b < self.size_col:
                if self.map_array[x_a][y_a] == 1 and self.map_array[x_b][y_b] == 0:
                    self.samples.append([x_a,y_a])
                    n_samples = n_samples + 1
                elif self.map_array[x_a][y_a] == 0 and self.map_array[x_b][y_b] == 1:
                    self.samples.append([x_a,y_a])
                    n_samples = n_samples + 1
                else:
                    pass
            
            else:
                pass

    def bridge_sample(self, n_pts):
        """Use bridge sampling and store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        ### YOUR CODE HERE ###
        for i in range(n_pts):
            q1 = np.random.randint(low=(0, 0), high=(self.size_row, self.size_col), size=(1,2))[0]
            if self.map_array[q1[0]][q1[1]] == 1:
                continue
            q2 = np.random.multivariate_normal(q1, [[300,0],[0,300]], size=1)[0]
            q2 = [round(i) for i in q2]
            
            if((q2[0] > self.size_row-1) or (q2[1] > self.size_col-1)):
                continue
            if self.map_array[q2[0]][q2[1]] == 0:
                temp_bridge_point = [round(0.5*(q1[0]+q2[0])), round(0.5*(q1[1]+q2[1]))]
                if self.map_array[temp_bridge_point[0]][temp_bridge_point[1]] == 1:
                    self.samples.append(temp_bridge_point)


    def new_sampler(self, n_pts):
        """Use New sampling method to store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()

        n_samples = 0
        count = 0
        while n_samples <= n_pts:
            i = np.random.uniform(low=(0, 0), high=(self.size_row, self.size_col), size=(1,2)).astype(int)[0]
            j = np.random.multivariate_normal(i, [[300,0],[0,300]], 1).astype(int)[0]
            sampling_thresholds = [0.5]
            angle_threshold =  30
            angle_step = 10
            distance_threshold  = 100
            if (i[0] == j[0] and i[1] == j[1]) or (i[0] < 0 or i[0] >= self.size_row or i[1] <0 or i[1] >= self.size_col) \
                 or (j[0] < 0 or j[0] >= self.size_row or j[1] <0 or j[1] >= self.size_col) :
                pass
            else:
                if self.map_array[i[0]][i[1]] ==0 and self.map_array[j[0]][j[1]] == 0:
                    temp_theta = math.degrees(math.atan2(i[1]-j[1], i[0]-j[0]))
                    distance = self.dis(i,j)
                    if distance<= distance_threshold:
                        for k in sampling_thresholds:
                            for m in (temp_theta - angle_threshold , temp_theta + angle_threshold, angle_step):
                                temp_x  = math.floor(i[0] + k* distance * np.cos(m))
                                temp_y = math.floor(i[1] + k* distance* np.sin(m))
                                if temp_x < self.size_row and temp_y < self.size_col:
                                    if self.map_array[temp_x][temp_y] == 1:
                                        count = count +1
                                        self.samples.append([temp_x,temp_y])
                                        #n_samples = n_samples+ 1
                                    else:
                                        pass
                else:
                    if self.map_array[i[0]][i[1]] ==1 and self.map_array[j[0]][j[1]] ==0: 
                        self.samples.append(i)
                        #n_samples = n_samples + 1
                    elif self.map_array[j[0]][j[1]] == 0 and self.map_array[j[0]][j[1]] ==1:
                        self.samples.append(j)
                        #n_samples = n_samples + 1
            n_samples = n_samples +1

    
    def weighted_sampler(self, n_pts):
        """Use Weighted sampling method to store valid points
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points

        check collision and append valid points to self.samples
        as [(row1, col1), (row2, col2), (row3, col3) ...]
        """
        # Initialize graph
        self.graph.clear()
        samples = []
        sampler_ratios = {'gaussian': 0.3 , 'bridge': 0.6, 'uniform': 0.1}

        gaussian_points = int(sampler_ratios['gaussian'] * n_pts) 
        bridge_points = int(sampler_ratios['bridge'] * n_pts) 
        uniform_points = int(sampler_ratios['uniform'] * n_pts)

        self.gaussian_sample(gaussian_points)
        samples += self.samples

        self.bridge_sample(bridge_points)
        samples += self.samples

        self.uniform_sample(uniform_points)
        samples += self.samples

        self.samples = samples


    def draw_map(self):
        """Visualization of the result"""
        # Create empty map
        fig, ax = plt.subplots()
        img = 255 * np.dstack((self.map_array, self.map_array, self.map_array))
        ax.imshow(img)

        # Draw graph
        # get node position (swap coordinates)
        node_pos = np.array(self.samples)[:, [1, 0]]
        pos = dict(zip(range(len(self.samples)), node_pos))
        pos["start"] = (self.samples[-2][1], self.samples[-2][0])
        pos["goal"] = (self.samples[-1][1], self.samples[-1][0])

        # draw constructed graph
        nx.draw(
            self.graph, pos, node_size=3, node_color="y", edge_color="y", ax=ax
        )

        # If found a path
        if self.path:
            # add temporary start and goal edge to the path
            final_path_edge = list(zip(self.path[:-1], self.path[1:]))
            nx.draw_networkx_nodes(
                self.graph,
                pos=pos,
                nodelist=self.path,
                node_size=8,
                node_color="b",
            )
            nx.draw_networkx_edges(
                self.graph,
                pos=pos,
                edgelist=final_path_edge,
                width=2,
                edge_color="b",
            )

        # draw start and goal
        nx.draw_networkx_nodes(
            self.graph,
            pos=pos,
            nodelist=["start"],
            node_size=12,
            node_color="g",
        )
        nx.draw_networkx_nodes(
            self.graph, pos=pos, nodelist=["goal"], node_size=12, node_color="r"
        )

        # show image
        plt.axis("on")
        ax.tick_params(left=True, bottom=True, labelleft=True, labelbottom=True)
        plt.show()

    def sample(self, n_pts=1000, sampling_method="uniform"):
        """Construct a graph for PRM
        arguments:
            n_pts - number of points try to sample,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method

        Sample points, connect, and add nodes and edges to self.graph
        """
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
        elif sampling_method == "new_sampler":
            self.new_sampler(n_pts)
        elif sampling_method == "weighted_sampler":
            self.weighted_sampler(n_pts)

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.

        # Store them as
        # pairs = [(p_id0, p_id1, weight_01),
        #          (p_id0, p_id2, weight_02),
        #          (p_id1, p_id2, weight_12) ...]
        pairs = []
        nodes = []
        
        # Use the below method for distance based iterative approach - Slower but more control
        #distance_threshold = 20 # Add this to self better
        # for i , point1 in enumerate(self.samples):
        #     for j, point2 in enumerate(self.samples):
        #         if i!=j:
        #             distance = self.dis(point1, point2)
        #             if distance <= distance_threshold and not self.check_collision(point1, point2):
        #                 pairs.append((i, j, distance))
        #                 nodes.append(i)
        #                 nodes.append(j)

        # Use the below for KDTree based method
        sampling_radius = 18
        kd_tree = cKDTree(self.samples)
        n_ids = kd_tree.query_pairs(sampling_radius)
        
        for id in n_ids:
            p1 = id[0]
            p2 = id[1]
            dist = self.dis(self.samples[p1], self.samples[p2])
            if self.check_collision(self.samples[p1], self.samples[p2]):
                continue
            pairs.append((p1,p2,dist))


        nodes = list(set(nodes))

        # Use sampled points and pairs of points to build a graph.
        # To add nodes to the graph, use
        # self.graph.add_nodes_from([p_id0, p_id1, p_id2 ...])
        # To add weighted edges to the graph, use
        # self.graph.add_weighted_edges_from([(p_id0, p_id1, weight_01),
        #                                     (p_id0, p_id2, weight_02),
        #                                     (p_id1, p_id2, weight_12) ...])
        # 'p_id' here is an integer, representing the order of
        # current point in self.samples
        # For example, for self.samples = [(1, 2), (3, 4), (5, 6)],
        # p_id for (1, 2) is 0 and p_id for (3, 4) is 1.
        self.graph.add_nodes_from(nodes)
        self.graph.add_weighted_edges_from(pairs)

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print(
            "The constructed graph has %d nodes and %d edges"
            % (n_nodes, n_edges)
        )
    
    def incremental_sample_prm(self, n_pts=1000, sampling_method="uniform", new_samples=300):
        """Incrementally construct a graph for PRM and search for a solution
        after adding each new node.
        arguments:
            n_pts - number of points try to sample initially,
                    not the number of final sampled points
            sampling_method - name of the chosen sampling method
            new_samples - number of new samples to add incrementally

        Sample points, connect, and add nodes and edges to self.graph, and
        search for a solution after adding each new node.
        """
        # Initialize before sampling
        self.samples = []
        self.graph.clear()
        self.path = []

        # Sample methods
        if sampling_method == "uniform":
            self.uniform_sample(n_pts)
        elif sampling_method == "gaussian":
            self.gaussian_sample(n_pts)
        elif sampling_method == "bridge":
            self.bridge_sample(n_pts)
        elif sampling_method == "new_sampler":
            self.new_sampler(n_pts)
        elif sampling_method == "weighted_sampler":
            self.cluster_sampler(n_pts)

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # Store them as pairs = [(p_id0, p_id1, weight_01),
        #                         (p_id0, p_id2, weight_02),
        #                         (p_id1, p_id2, weight_12) ...]
        pairs = []
        nodes = []

        # Use the below for KDTree based method
        sampling_radius = 18
        kd_tree = cKDTree(self.samples)
        n_ids = kd_tree.query_pairs(sampling_radius)

        for id in n_ids:
            p1 = id[0]
            p2 = id[1]
            dist = self.dis(self.samples[p1], self.samples[p2])
            if self.check_collision(self.samples[p1], self.samples[p2]):
                continue
            pairs.append((p1, p2, dist))

        nodes = list(set(nodes))

        # Use sampled points and pairs of points to build a graph.
        self.graph.add_nodes_from(nodes)
        self.graph.add_weighted_edges_from(pairs)

        # Perform incremental search after adding each new sample
        for _ in range(new_samples):
            new_sample = np.random.uniform((0, 0),(self.size_row, self.size_col), size=(1, 2)).astype(int)[0]
            nearest_node_id = kd_tree.query(new_sample)[1]

            # Ensure that the new sample does not collide with the nearest node
            if not self.check_collision(new_sample, self.samples[nearest_node_id]):
                self.samples.append(new_sample)
                self.graph.add_node(len(self.samples) - 1)
                self.graph.add_edge(nearest_node_id, len(self.samples) - 1, weight=self.dis(self.samples[nearest_node_id], new_sample))

                # Search for a solution after adding the new node
                start = self.samples[-2]
                goal = self.samples[-1]
                # self.search(start, goal) - Yet to complete
        

        # Print constructed graph information
        n_nodes = self.graph.number_of_nodes()
        n_edges = self.graph.number_of_edges()
        print("The constructed graph has %d nodes and %d edges" % (n_nodes, n_edges))


    def search(self, start, goal):
        """Search for a path in graph given start and goal location
        arguments:
            start - start point coordinate [row, col]
            goal - goal point coordinate [row, col]

        Temporary add start and goal nodes, edges of them
        and their nearest neighbors to graph for
        self.graph to search for a path.
        """
        # Clear previous path
        self.path = []

        # Temporarily add start and goal to the graph
        self.samples.append(start)
        self.samples.append(goal)
        # start and goal id will be 'start' and 'goal' instead of some integer
        self.graph.add_nodes_from(["start", "goal"])

        # ### YOUR CODE HERE ###

        # Find the pairs of points that need to be connected
        # and compute their distance/weight.
        # You could store them as
        # start_pairs = [(start_id, p_id0, weight_s0),
        #                (start_id, p_id1, weight_s1),
        #                (start_id, p_id2, weight_s2) ...]
        start_pairs = []
        goal_pairs = []

        # distance_threshold = 2 # Add this to self better

        # for i , point1 in enumerate(self.samples):
        #     distance_start = self.dis(start, point1)
        #     distance_goal = self.dis(point1, goal)
        #     if distance_start < distance_threshold and self.check_collision(start, point1):
        #         start_pairs.append((len(self.samples)-2,i, distance_start))
        #     if distance_goal < distance_threshold and self.check_collision(point1, goal):
        #         goal_pairs.append((len(self.samples)-1,i, distance_goal))
        
        # # Start point
        kd_tree = cKDTree(self.samples)
        k = 30 
        nnd, nni = kd_tree.query(np.array(start), k)
        for ind, dis  in zip(nni, nnd):
                if not self.check_collision(self.samples[-2], self.samples[ind]):
                    start_pairs.append(('start', ind, dis))


        # Goal point
        nnd, nni = kd_tree.query(np.array(goal), k)
        for ind, dis  in zip(nni, nnd):
                if not self.check_collision(self.samples[-1], self.samples[ind]):
                    goal_pairs.append((ind, 'goal', dis))


        # Add the edge to graph
        self.graph.add_weighted_edges_from(start_pairs)
        self.graph.add_weighted_edges_from(goal_pairs)

        # Seach using Dijkstra
        try:
            self.path = nx.algorithms.shortest_paths.weighted.dijkstra_path(
                self.graph, "start", "goal"
            )
            self.path_length = (
                nx.algorithms.shortest_paths.weighted.dijkstra_path_length(
                    self.graph, "start", "goal"
                )
            )
            print("Number of nodes in Path:", len(self.path))
            print("The path length is %.2f" % self.path_length)
        except nx.exception.NetworkXNoPath:
            print("No path found")

        # Draw result
        self.draw_map()

        # Remove start and goal node and their edges
        self.samples.pop(-1)
        self.samples.pop(-1)
        self.graph.remove_nodes_from(["start", "goal"])
        self.graph.remove_edges_from(start_pairs)
        self.graph.remove_edges_from(goal_pairs)

    def benchmarker(self, start = (200, 75),goal = (30, 250)):
        '''
        Function to benchmark different algorithms given start and goal points'''

        list_of_samplers = ['uniform', 'gaussian', 'bridge', 'new_sampler', 'weighted_sampler']

        node_count_list = defaultdict(list, {k:[] for k in list_of_samplers})
        path_length_list = defaultdict(list, {k:[] for k in list_of_samplers})
        distance_list = defaultdict(list,{k:[] for k in list_of_samplers})

        no_of_points = 5000 # Evaluating every algorithm on same number of sample points
        # using incremental sampler here  makes much more sense
        for i in list_of_samplers:
            for j in range(0,100): # 20 iterations
                try:
                    self.sample(n_pts=no_of_points, sampling_method=i)
                    self.search(start, goal)
                    node_count_list[i] = node_count_list[i] + [self.graph.number_of_nodes()]
                    path_length_list[i] = path_length_list[i] + [len(self.path)]
                    distance_list[i] = distance_list[i] + [self.path_length]
                except Exception as e:
                    print("Exception is: {}".format(e))
                    pass

        # Node count List box plot 
        fig,ax = plt.subplots()
        ax.boxplot(node_count_list.values())
        ax.set_xticklabels(node_count_list.keys())

        fig.savefig('NodeCountPlot.png')

        # Path Count List box plot 
        fig,ax = plt.subplots()
        ax.boxplot(path_length_list.values())
        ax.set_xticklabels(path_length_list.keys())

        fig.savefig('PathCountPlot.png')

        # Node count List box plot 
        fig,ax = plt.subplots()
        ax.boxplot(distance_list.values())
        ax.set_xticklabels(distance_list.keys())

        fig.savefig('PathlengthPlot.png')