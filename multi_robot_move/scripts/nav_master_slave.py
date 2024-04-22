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

system_master_slave = {}

async def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) != 2:
        print("Usage: ros2 run multi_robot_move nav_master_slave.py <path_to_yaml_file>")
        return

    yaml_file_path = args[1]

    rclpy.init(args=args)

    data_nav_robots = DataRobots(yaml_file_path)

    for robot in data_nav_robots.generate_robots():

        name_robot = robot['name']
        is_master = robot['is_master']
        
        if is_master and not(name_robot in system_master_slave):
            nav_master = BasicNavigator(namespace=name_robot)

            system_master_slave[name_robot] = {"nav_class": nav_master, "slaves": {}, "slave_tasks": {}, "status": True}
        
        if not is_master:
            nav_slave = BasicNavigator(namespace=name_robot)

            list_poses_wo_process = data_nav_robots.get_list_poses(robot)  # obtener lista de poses sin convertir en PoseStamped
            goal_poses_robot = PoseUtils.create_poses(list_poses_wo_process)  # convertir a PoseStamped
            
            name_master_robot = robot['name_master']                

            if not(name_master_robot in system_master_slave):
                nav_master = BasicNavigator(namespace=name_master_robot)

                system_master_slave[name_master_robot] = {"nav_class": nav_master, "slaves": {}, "slave_tasks": {}, "status": True}
            
            dict_master = system_master_slave[name_master_robot]
            dict_master["slaves"][name_robot] = {"nav_class": nav_slave, "master": name_master_robot, "tasks": goal_poses_robot, "status": True}

    print(system_master_slave)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())