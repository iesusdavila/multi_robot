#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from NavigationClient import BasicNavigator, TaskResult
from PoseUtils import PoseUtils
from DataRobots import DataRobots
import time
from rclpy.duration import Duration
import asyncio
import sys

async def navigate_robot_master(nav_client, goal_poses_robot, nav_start):
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
            if now - nav_start > Duration(seconds=30.0):
                print("Tiempo de navegación excedido")

async def navigate_robot(nav_client, goal_poses_robot, nav_start, name_master_robot):
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
            if now - nav_start > Duration(seconds=50.0):
                print("Tiempo de navegación excedido")
                nav_client.cancelTask()

                nav_master = BasicNavigator(namespace=name_master_robot)
                pose_goal = pose_utils.create_pose(1.5, -0.5, 0.0, 0.0)

                nav_start = nav_master.get_clock().now()

                await asyncio.gather(navigate_robot_master(nav_master, [pose_goal], nav_start)) 

async def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) != 2:
        print("Usage: ros2 run multi_robot_move nav_through_poses.py <path_to_yaml_file>")
        return

    yaml_file_path = args[1]

    rclpy.init(args=args)

    pose_utils = PoseUtils()
    data_nav_robots = DataRobots(yaml_file_path)

    list_funciones = []
    for robot in data_nav_robots.generate_robots():

        name_robot = robot['name']

        if not robot['is_master']:
            navigation_client = BasicNavigator(namespace=name_robot)

            list_poses_wo_process = data_nav_robots.get_list_poses(robot)  # obtener lista de poses sin convertir en PoseStamped
            list_poses_w_process = pose_utils.create_poses(list_poses_wo_process)  # convertir a PoseStamped
            goal_poses_robot = list_poses_w_process
            name_master_robot = robot['name_master']

            nav_start = navigation_client.get_clock().now()

            list_funciones.append(navigate_robot(navigation_client, goal_poses_robot, nav_start, name_master_robot))
        else:
            print("Master robot: " + name_robot)

    await asyncio.gather(*list_funciones)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
