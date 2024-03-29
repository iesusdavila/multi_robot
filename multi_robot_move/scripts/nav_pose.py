#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from NavigationClient import NavigationClient
from PoseUtils import PoseUtils
from DataRobots import DataRobots
import time

def main(args=None):
    rclpy.init(args=args)

    pose_utils = PoseUtils()
    data_nav_robots = DataRobots('/home/rov-robot/project_ws/src/multi_robot/multi_robot_move/configs/nav_robots_world.yaml')
    
    navigation_clients = []

    for nav_robot in data_nav_robots.generate_robots():
        print(f'Robot: {nav_robot["name"]} tiene {data_nav_robots.get_number_poses(nav_robot)} poses')
        name_robot = data_nav_robots.get_name(nav_robot)

        navigation_client = NavigationClient(name_robot)
        navigation_clients.append(navigation_client)

        goal_poses = []
        for i in range(data_nav_robots.get_number_poses(nav_robot)):
            x, y, z, yaw = data_nav_robots.get_pose(nav_robot, i).values()
            print(f'Pose Goal {i+1}: {x} {y} {z} {yaw} ')
            
            goal_poses.append(pose_utils.create_pose(x, y, z, yaw))

        navigation_client.send_goal_pose(goal_poses[0])

        navigation_client.destroy()
    
    print("Estoy fuera del for jejej")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
