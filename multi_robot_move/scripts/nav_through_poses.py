#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from NavigationClient import NavigationClient
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from PoseUtils import PoseUtils
from DataRobots import DataRobots
import time

def main(args=None):
    rclpy.init(args=args)

    pose_utils = PoseUtils()
    data_nav_robots = DataRobots('/home/rov-robot/project_ws/src/multi_robot/multi_robot_move/configs/nav_robots_world.yaml')

    # Define la posición de destino
    goal_pose = pose_utils.create_pose(1.5, -0.5, 0.0, 0.0)
    goal_pose2 = pose_utils.create_pose(0.0, 0.5, 0.0, 0.0)
    goal_pose3 = pose_utils.create_pose(-1.0, 0.5, 0.0, 0.0)

    goal_poses_robot_1 = [goal_pose, goal_pose2]
    goal_poses_robot_2 = [goal_pose2, goal_pose3]

    # Crear instancias de la clase NavigationClient para cada robot
    # navigation_client_1 = NavigationClient(str(data_nav_robots.generate_robots()[0]['name']))
    # navigation_client_2 = NavigationClient(str(data_nav_robots.generate_robots()[1]['name']))

    name_robot_1 = str(data_nav_robots.generate_robots()[0]['name'])
    name_robot_2 = str(data_nav_robots.generate_robots()[1]['name'])

    navigation_client_1 = BasicNavigator(node_name='basic_navigator_'+name_robot_1, namespace=name_robot_1)
    navigation_client_2 = BasicNavigator(node_name='basic_navigator_'+name_robot_2, namespace=name_robot_2)

    # Envía la acción de navegación al robot 1
    navigation_client_1.followWaypoints(goal_poses_robot_1)

    # Espera un momento antes de enviar la acción al siguiente robot (solo para ejemplo)
    time.sleep(2)

    # Envía la acción de navegación al robot 2
    navigation_client_2.followWaypoints(goal_poses_robot_2)
    
    # Destruye las instancias de NavigationClient
    # navigation_client_1.destroy()
    # navigation_client_2.destroy()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
