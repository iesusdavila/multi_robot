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
            if now - nav_start > Duration(seconds=120.0):
                goal_pose4 = pose_utils.create_pose(-1.5, 0.5, 0.0, 0.0)
                goal_poses_robot = [goal_pose4]
                nav_start = now
                nav_client.followWaypoints(goal_poses_robot)

async def main(args=None):
    # REALIZAR PARA PASAR LA RUTA COMO ARGUMENTO!!! PILOSKIIIIIIII!!!!

    rclpy.init(args=args)

    pose_utils = PoseUtils()
    data_nav_robots = DataRobots('/home/rov-robot/project_ws/src/multi_robot/multi_robot_move/configs/nav_robots_world.yaml')

    list_funciones = []
    for robot in data_nav_robots.generate_robots():

        name_robot = robot['name']
        navigation_client = BasicNavigator(namespace=name_robot)

        list_poses_wo_process = data_nav_robots.get_list_poses(robot) # obtener lista de poses sin convertir en PoseStamped
        list_poses_w_process = pose_utils.create_poses(list_poses_wo_process) # convertir a PoseStamped
        goal_poses_robot = list_poses_w_process

        nav_start = navigation_client.get_clock().now()

        list_funciones.append(navigate_robot(navigation_client, goal_poses_robot, nav_start))

    await asyncio.gather(*list_funciones)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
