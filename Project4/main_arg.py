import numpy as np

from map_2d import Map2D
from planner import Planner
from map_2d import Map2D
from robot import PointRobot, OmnidirectionalRobot, KinematicChain
from RRT import RRT
from PRM import PRM
import argparse



if __name__ == "__main__":
    # Load the map
    # Map is loaded as a map size tuple map_2d.map_shape (x_lim, y_lim),
    # and a list of obstacles map_2d.obstacles [(x1, y1), (x2, y2), ...].
    parser = argparse.ArgumentParser()
    parser.add_argument('--map', type=str, default='maps/map_singleobstacle.csv')
    parser.add_argument('--robot', type=str, default='PointRobot', options=['PointRobot', 'OmnidirectionalRobot', 'KinematicChain'])
    parser.add_argument('--start', type=tuple, default=(20, 20), options=[(20, 20), (20, 20, 0), (3, -0.1, -0.5, -0.5)])
    parser.add_argument('--goal', type=tuple, default=(80, 80), options=[(80, 80), (80, 80, np.pi / 2), (2, -0.4, -0.3, -0.1)])
    parser.add_argument('--method', type=str, default='PRM', options=['PRM', 'RRT'])
    parser.add_argument('--sampling_method', type=str, default='uniform', options=['uniform', 'gaussian', 'bridge', 'RRT', 'RRT_star', 'Informed_RRT_star'])
    parser.add_argument('--n_configs', type=int, default=300)
    parser.add_argument('--kdtree_d', type=int, default=10, options=[10, np.pi])
    parser.add_argument('--make_video', type=bool, default=False)
    parser.add_argument('--video_name', type=str, default='path.mp4')
    parser.add_argument('--link_lengths', type=list, default=[15, 15, 15, 15])
    parser.add_argument('--base', type=list, default=[50, 15])

    args = parser.parse_args()
    if args.make_video == True:
        print("*****************"*2)
        print("NOTE: As the video creation enabled, it will take a considerable amount of time.")

    map_2d = Map2D(args.map)

    robot_name = args.robot

    start = args.start
    end = args.end

    # Define the robot with start and goal configurations
    if robot_name == 'PointRobot':
        robot = PointRobot()
    elif robot_name == 'OmnidirectionalRobot':
        robot = OmnidirectionalRobot(width=2.5, height=5)
    elif robot_name == "KinematicChain":
        robot = KinematicChain(args.link_lengths, args.base)
    else:
        raise ValueError("Robot name not recognized.")

    # select the planner method
    # for the pointrobot and omnidirectionalrobot
    if args.method == "PRM":
        method = PRM(sampling_method=args.sampling_method, n_configs=args.n_configs, kdtree_d=args.kdtree_d)
    elif args.method == "RRT":
        method = RRT(sampling_method=args.sampling_method, n_configs=args.n_configs, kdtree_d=args.kdtree_d)
    else:
        raise ValueError("Method not recognized for Point and Omnidirectional Robot.")

    # For the KinematicChain
    if args.method == "PRM":
        method = PRM(sampling_method=args.sampling_method, n_configs=args.n_configs, kdtree_d=args.kdtree_d)
    elif args.method == "RRT":
        method = RRT(sampling_method=args.sampling_method, n_configs=args.n_configs, kdtree_d=args.kdtree_d)
    else:
        raise ValueError("Method not recognized for Kinematic Robot.")

    # Initialize the planner
    planner = Planner(method, map_2d, robot)

    # planning
    planner.plan(start, goal)
    #If make_video true a video it will create file path.mp4. 
    #NOTE: Creating the video takes a considerable amount of time.
    full_video_name = str(args.method) + "_" + str(args.sampling_method)  + "_" + str(args.kdtree_d) + "_" + str(args.video_name)
    planner.visualize(make_video=args.make_video, video_name=full_video_name) #, show=True)
