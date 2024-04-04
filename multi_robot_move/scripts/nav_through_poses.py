#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from NavigationClient import BasicNavigator, TaskResult
from PoseUtils import PoseUtils
from DataRobots import DataRobots
import time
from rclpy.duration import Duration
import asyncio

async def navigate_robot(nav_client, goal_poses_robot, nav_start):
    pose_utils = PoseUtils()
    
    nav_client.followWaypoints(goal_poses_robot)
    while not nav_client.isTaskComplete():
        await asyncio.sleep(1)  # Espera corta para permitir que otras tareas se ejecuten
        feedback = nav_client.getFeedback()
        if feedback:
            print(
                'Executing current waypoint '
                + str(nav_client.getNameRobot())
                + ': '
                + str(feedback.current_waypoint + 1)
                + '/'
                + str(len(goal_poses_robot))
            )
            now = nav_client.get_clock().now()

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                nav_client.cancelTask()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=35.0):
                goal_pose4 = pose_utils.create_pose(-1.5, 0.5, 0.0, 0.0)
                goal_poses_robot = [goal_pose4]
                nav_start = now
                nav_client.followWaypoints(goal_poses_robot)

async def main(args=None):
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
    name_robot_1 = str(data_nav_robots.generate_robots()[0]['name'])
    name_robot_2 = str(data_nav_robots.generate_robots()[1]['name'])

    navigation_client_1 = BasicNavigator(namespace=name_robot_1)
    navigation_client_2 = BasicNavigator(namespace=name_robot_2)

    # Iniciar el tiempo de navegación
    nav_start = navigation_client_1.get_clock().now()

    # Ejecutar las tareas de navegación de los robots de forma concurrente
    await asyncio.gather(
        navigate_robot(navigation_client_1, goal_poses_robot_1, nav_start),
        navigate_robot(navigation_client_2, goal_poses_robot_2, nav_start)
    )

    # Destruir las instancias de NavigationClient
    # navigation_client_1.destroy()
    # navigation_client_2.destroy()

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
