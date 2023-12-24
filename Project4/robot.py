import numpy as np
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.animation as manimation

from utils import endpoints_to_edges, angle_diff, interpolate_angle
from utils import is_in_polygon, is_intersecting, line_intersection

# This is for kinematic chain check imports
from utils import calculate_internal_angle

class Robot:
    """A parent class for all robots"""

    def __init__(self, limits):
        """Initialize by providing limits of each degree of freedom"""
        # Limits in each dof, each limit is defined as (lower, upper, name)
        self.limits = limits
        self.dof = len(limits)

    def forward_kinematics(self, config):
        """Compute the endpoints of the robot given a configuration
        The last endpoint would be used for visualization of the sampling
        """
        raise NotImplementedError

    def get_edges(self):
        """Return the edges of the robot for collision checking"""
        raise NotImplementedError

    def distance(self, config1, config2):
        """Compute the distance between two configurations"""
        raise NotImplementedError

    def interpolate(self, config1, config2, num):
        """Interpolate between two configurations"""
        raise NotImplementedError

    def check_collision(
        self, config1, config2, map_corners, obstacles, obstacle_edges
    ):
        """Check colliding with obstacles between two configurations
        First perform an interpolation between the two configurations,
        then check if any of the interpolated configurations hit obstacles.
       
        arguments:
            config1 - configuration 1
            config2 - configuration 2
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles between the two configurations
        """
        # Get intepolated configurations in between config1 and config2
        configs_between = self.interpolate(config1, config2)

        # check if any of these configurations hit obstacles
        for config in configs_between:
            if self.check_collision_config(
                config, map_corners, obstacles, obstacle_edges
            ):
                return True
        return False

    def check_collision_config(
        self, config, map_corners, obstacles, obstacle_edges
    ):
        """Check if a configuration is colliding with obstacles. Ensure that all  
        cases are checked. Even ones that might not be present in the given map. 
        arguments:
            config - configuration of the robot
            map_corners - corners of the map
            obstacles - list of obstacles
            obstacle_edges - list of edges of obstacles, including map edges
        
        return:
            True if colliding with obstacles
        """
        # Get the edges of the robot for collision checking
        robot_endpoint = self.forward_kinematics(config)[-1]
        robot_edges = self.get_edges(config)

        # Check if the robot endpoint is outside the map
        if not is_in_polygon(robot_endpoint, map_corners):
            return True

        # Check if the robot endpoint is inside any obstacle
        for obstacle in obstacles:
            if is_in_polygon(robot_endpoint, obstacle):
                return True
            
            

        ### YOUR CODE HERE ###
         # Check if any of the robot edges are intersecting with obstacle edge

        if is_intersecting(robot_edges, obstacle_edges):
            return True
        
        if self.robot_name == "omnidirectional":
            endpoints = self.forward_kinematics(config)
            # Check if the robot points are inside map or not
            for i in range(len(endpoints)):
                if not is_in_polygon(endpoints[i], map_corners):
                    return True
            
            for i in range(len(endpoints)):
                for obstacle in obstacles:
                    if is_in_polygon(endpoints[i], obstacle):
                        print(True)
                        return True
            
            for i in range(len(robot_edges)):
                for j in range(len(obstacle_edges)):
                    if is_intersecting([robot_edges[i]], [obstacle_edges[j]]):
                        return True
                        
        if self.robot_name == "kinematic_chain":
            endpoints = self.forward_kinematics(config)
            robot_edges = self.get_edges(config)

            # Check if the robot points are inside map or not
            # for i in range(len(endpoints)):
            #     if not is_in_polygon(endpoints[i], map_corners):
            #         return True
            
            # for i in range(len(endpoints)):
            #     for obstacle in obstacles:
            #         if is_in_polygon(endpoints[i], obstacle):
            #             print(True)
            #             return True
            
            for i in range(len(robot_edges)):
                for j in range(i + 1, len(robot_edges)):
                    Line1 = list(robot_edges[i])
                    A,B = Line1
                    A1, B1 = A
                    A2, B2 = B
                    Line2 = list(robot_edges[j])
                    A,B = Line2
                    A3, B3 = A
                    A4, B4 = B
                    Edge1 = [[A1,B1],[A2,B2]]
                    Edge2 = [[A3,B3],[A4,B4]]

                    # Avoid inf condition with a limiter
                    if A1 == A2:
                        A1 = A1-0.0001
                    if A3 == A4:
                        A3 = A3-0.0001
                        
                    M1 = (B2-B1)/(A2-A1)
                    M2 = (B4-B3)/(A4-A3)

                    
                    if (M1 == M2):
                        return True
                
                    Point = line_intersection(Edge1, Edge2)
                    if Point:
                        if Point not in endpoints:
                            return True

                    # # Check directions too so that one link does not fall on another
                    # direction_A1B1A2B2 = np.cross(np.array([A2-A1,B2-B1]),np.array([A3-A1,B3-B1]))
                    # direction_A1B1A2B2 = direction_A1B1A2B2/np.abs(direction_A1B1A2B2)
                    # direction_A3B3A4B4 = np.cross(np.array([A4-A3,B4-B3]),np.array([A1-A3,B1-B3]))
                    # direction_A3B3A4B4 = direction_A3B3A4B4/np.abs(direction_A3B3A4B4)

                    # if direction_A1B1A2B2 == - direction_A3B3A4B4:
                    #     return True

            # Internal Angle constraint based link collision check
            for i in range(len(robot_edges) - 1):
                internal_angle = calculate_internal_angle(robot_edges[i], robot_edges[i + 1])
                if internal_angle < self.internal_angle_constraint:
                    # This to constrain the internal_angle so that robot links wont go over one another
                    return True

            
        return False


    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        """Draw the robot given a configuration on a matplotlib axis.
        This is for visualization purpose only.
        """
        raise NotImplementedError


class PointRobot(Robot):
    """2D Point robot class"""

    def __init__(self):
        """Initialize the robot with no limits in x, y (0, 1000))"""
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y")
        ])
        self.robot_name = "point"

    def forward_kinematics(self, config):
        """Simply return the configuration as the endpoint"""
        return [config]

    def get_edges(self, config):
        """Simply return an empty list"""
        return []

    def distance(self, config1, config2):
        """Euclidean distance"""
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        return np.sqrt(x_diff**2 + y_diff**2)

    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations"""
        configs_between = zip(
            np.linspace(config1[0], config2[0], num),
            np.linspace(config1[1], config2[1], num)
        )
        return configs_between

    def draw_robot(self, ax, config, edgecolor="b", facecolor="g"):
        ax.scatter(config[0], config[1], s=20, c=edgecolor)


class OmnidirectionalRobot(Robot):
    """Omnidirectional navigation robot class
    Its shape is defined as a rectangle with a width and a height.
    The robot could move in any direction with any angle in a 2D plane.
    """

    def __init__(self, width, height):
        """Initialize the robot with a width and height."""
        self.width = width
        self.height = height
        self.robot_name = "omnidirectional"
        # Limits in each dof: (x, y, theta)
        # x, y have no limits unless bounded by the map (1000 as default)
        # theta range is (-pi, pi)
        super().__init__(limits=[
            (0, 1000, "x"),
            (0, 1000, "y"),
            (-np.pi, np.pi, "r")
        ])

    def forward_kinematics(self, config):
        """Compute the 4 corner coordinates of the robot given a configuration
        Also attach the center of the robot as the last endpoint.
        The last endpoint would be used for visualization of the sampling.
        arguments:
            config: [x, y, theta] of the rectangle

        return:
            endpoints: 4 corner coordinates of the rectangle and its center
                       [corner1, corner2, corner3, corner4, center]
        """
        # Check configuration shape
        assert len(config) == 3, "Configuration should be (x, y, theta)"

        x, y, theta = config
        endpoints = np.zeros((5, 2))

        ### YOUR CODE HERE ###
        half_width = self.width / 2
        half_height = self.height / 2

        corners = np.array([[-half_width, -half_height], [half_width, -half_height], [half_width, half_height], [-half_width, half_height]])
        # Transformation matrix
        T = np.eye(3)
        for i in range(4):
            cosine_angle = np.cos(np.pi/2- theta)
            sine_angle = np.sin(np.pi/2 - theta)
            temp_x_link = corners[i,0] * cosine_angle - corners[i,1] * sine_angle
            temp_y_link = corners[i,0] * sine_angle + corners[i,1] * cosine_angle
            temp_endpoint = (temp_x_link + x, temp_y_link + y)
            endpoints[i] = temp_endpoint

        # Center point             
        endpoints[4] = (x,y)

        return endpoints

    def get_edges(self, config):
        """Compute the edges of the robot given a configuration"""
        # Get the 4 corner coordinates

        ### YOUR CODE HERE ###
        endpoints = self.forward_kinematics(config)
        # Get the edges from endpoints
        edges = endpoints_to_edges(endpoints)
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]

        return:
            distance in R^2 x S^1 space
        """

        ### YOUR CODE HERE ###
        x_diff = config1[0] - config2[0]
        y_diff = config1[1] - config2[1]
        theta_diff = angle_diff(config1[2], config2[2])
        squared_distance = x_diff**2 + y_diff**2 + theta_diff**2
        euclidean_distance = np.sqrt(squared_distance)
        return euclidean_distance
    
    def interpolate(self, config1, config2, num=5):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [x, y, theta]
            p2 - config2, [x, y, theta]
        return:
            list with num number of configs from linear interploation in R^2 x S^1 space
        """

        ### YOUR CODE HERE ###
        interpolated_configurations = []

        for i in range(num):
            alpha = i / (num-1)
            x = (1-alpha) * config1[0] + alpha * config2[0]
            y = (1-alpha) * config1[1] + alpha * config2[1]
            theta = (1-alpha) * config1[2] + alpha * config2[2]
            interpolated_config = [x, y, theta]
            interpolated_configurations.append(interpolated_config)

        return interpolated_configurations

    def draw_robot(self, ax, config, edgecolor="b", facecolor="pink"):
        # compute corners and draw rectangle
        corners = self.forward_kinematics(config)[:4]
        polygon = Polygon(
            corners, closed=True, edgecolor=edgecolor, facecolor=facecolor
        )
        ax.add_patch(polygon)


class KinematicChain(Robot):
    """Kinematic chain robot class
    A planar robot with a fixed base and pure revolute joints.
    Each link is a line segment.
    """

    def __init__(self, link_lengths, base=[0.1, 0.1]):
        """Initialize with a list of link lengths, and a fixed base."""
        self.base = base
        self.link_lengths = link_lengths
        self.num_joints = len(link_lengths)
        self.robot_name = "kinematic_chain"
        self.internal_angle_constraint = 0  # 10 degrees - Can be changed on the constraints
        # Multi joint angle constraints can also be given - Yet to implement
        # Limits in each dof
        # assume all to be (-pi, pi)
        super().__init__(limits=[
            (-np.pi, np.pi, "r") for _ in range(self.num_joints)
        ])

    def forward_kinematics(self, config):
        """Compute the joint coordinates given a configuration of joint angles.
        The last endpoint would be used for visualization of the sampling
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of joint coordinates.
        """
        # Initialize the starting point as the fixed base
        joint_positions = [self.base]
        start_point = np.array(self.base)
        angle = 0

        # Compute the end points of each joint based on the configuration
        ### YOUR CODE HERE ###
        # Transformation matrix
        T = np.eye(3)
        T[0,2] = start_point[0]
        T[1,2] = start_point[1]
        for i in range(len(config)):
            temp_angle = config[i]
            cosine_angle = np.cos(temp_angle)
            sine_angle = np.sin(temp_angle)
            temp_x_link = self.link_lengths[i] * cosine_angle
            temp_y_link = self.link_lengths[i] * sine_angle
            temp_transformation_matrix = np.array([[cosine_angle, -sine_angle, temp_x_link], [sine_angle, cosine_angle, temp_y_link], [0, 0, 1]])
            # Multiply the transformation matrix with previous transformation matrix
            T = np.dot(T, temp_transformation_matrix)
            
        
            temp_joint_position = (T[0,2], T[1,2])

            #temp_joint_position = temp_joint_position + start_point
            # joint_position = np.dot(T, np.array([start_point[0], start_point[1],1]))
            # print(joint_position)
            # joint_position = joint_position[:2]
            # print(temp_joint_position)
            joint_positions.append(temp_joint_position)

        return joint_positions

    def get_edges(self, config):
        """Compute the link line segments of the robot given a configuration.
        arguments:
            config: A list of joint angles in radians.

        return:
            edges: A list of line segments representing the link line segments.
        """
        # Check configuration length
        assert (
            len(config) == self.num_joints
        ), "Configuration should match the number of joints"

        ### YOUR CODE HERE ###
        # Get edges of line segments
        edges = endpoints_to_edges(self.forward_kinematics(config))
        return edges

    def distance(self, config1, config2):
        """Calculate the euclidean distance between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in S^1 x S^1 x ... x S^1 space
        """
        ### YOUR CODE HERE ###

        # Euclidean distance in S^1 x S^1 x ... x S^1 space means circular space distance assumption

        # Calculate the distance between two configurations in circular space
        squared_distance = [angle_diff(config1[i], config2[i]) for i in range(len(config1))]
        squared_distance_sum = sum(distance**2 for distance in squared_distance)
        euclidean_distance = np.sqrt(squared_distance_sum)
        
        return euclidean_distance

    def interpolate(self, config1, config2, num=10):
        """Interpolate between two configurations
        arguments:
            p1 - config1, [joint1, joint2, joint3, ..., jointn]
            p2 - config2, [joint1, joint2, joint3, ..., jointn]

        return:
            A Euclidean distance in 
            list with num number of configs from linear interploation in S^1 x S^1 x ... x S^1 space.
        """

        ### YOUR CODE HERE ###
        interpolated_configurations = []

        # for i in range(1,num):
        #    temp_alpha = i / (num-1)
        #    temp_interpolated_config = [(1-temp_alpha) * config1[j] + temp_alpha * config2[j] for j in range(len(config1))]
        #    interpolated_configurations.append(temp_interpolated_config)
        interpolated_configurations = [interpolate_angle(config1[j],config2[j],num) for j in range(len(config1))]
       
        interpolated_configurations = np.array(interpolated_configurations).T
        interpolated_configurations = [i.tolist() for i in interpolated_configurations]

        interpolated_euclid_dis = [] 

        for i in range(len(interpolated_configurations)-1):
            interpolated_euclid_dis.append(self.distance(interpolated_configurations[i], interpolated_configurations[i+1]))
        
        return interpolated_configurations

    def draw_robot(self, ax, config, edgecolor="b", facecolor="black"):
        # compute joint positions and draw lines
        positions = self.forward_kinematics(config)
        # Draw lines between each joint
        for i in range(len(positions) - 1):
            line = np.array([positions[i], positions[i + 1]])
            ax.plot(line[:, 0], line[:, 1], color=edgecolor)
        # Draw joint
        for i in range(len(positions)):
            ax.scatter(positions[i][0], positions[i][1], s=5, c=facecolor)
