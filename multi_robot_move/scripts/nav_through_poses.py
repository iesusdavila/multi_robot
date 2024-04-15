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

master_robots = {}
tasks_master_robots = {}

def generate_message(name_robot, current_waypoint, number_poses, nav_time=(0,0,0), max_time=(0,0,0), name_slave=None):
    hour_nav, min_nav, sec_nav = nav_time
    hour_max, min_max, sec_max = max_time

    msg = 'Executing current waypoint ' + str(name_robot) + ': '+ str(current_waypoint + 1) + '/' + str(number_poses) 
    msg += ' - ' + str(hour_nav-9) + ':' + str(min_nav) + ':' + str(sec_nav) + ' / ' + str(hour_max-9) + ':' + str(min_max) + ':' + str(sec_max)

    if name_slave is not None: 
        msg += " del esclavo: " + name_slave
    
    print(msg)

async def navigate_robot_master(nav_master, name_slave):

    while tasks_master_robots[nav_master.getNameRobot()]:

        name_first_slave = list(tasks_master_robots[nav_master.getNameRobot()])[0]

        if name_first_slave == name_slave:
            print("Completando tarea pendiente del robot esclavo: " + name_first_slave)

            goal_poses_robot = tasks_master_robots[nav_master.getNameRobot()][name_first_slave]
            nav_master.followWaypoints(goal_poses_robot)

            nav_start = nav_master.get_clock().now()

            while not nav_master.isTaskComplete():
                await asyncio.sleep(1)
                feedback = nav_master.getFeedback()
                if feedback:
                    now = nav_master.get_clock().now()

                    nav_time = nav_master.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
                    max_time = nav_master.getTimeNav(Duration(seconds=600.0).nanoseconds)
                    
                    generate_message(nav_master.getNameRobot(), feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, name_slave)

                    # Some navigation timeout to demo cancellation
                    if now - nav_start > Duration(seconds=600.0):
                        nav_master.cancelTask()

            print("Tarea completada")
            tasks_master_robots[nav_master.getNameRobot()].pop(name_first_slave)
            print("Tarea eliminada de la lista de tareas pendientes")

            break
        else:
            print("El esclavo " + name_slave + " esta esperando a que el esclavo " + name_first_slave + " complete su tarea")
            await asyncio.sleep(1)

async def navigate_robot_slave(nav_slave, goal_poses_robot, nav_start, name_master):

    nav_slave.followWaypoints(goal_poses_robot)

    while not nav_slave.isTaskComplete():
        await asyncio.sleep(1)  # Espera corta para permitir que otras tareas se ejecuten
        feedback = nav_slave.getFeedback()
        if feedback:
            now = nav_slave.get_clock().now()

            nav_time = nav_slave.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
            max_time = nav_slave.getTimeNav(Duration(seconds=10.0).nanoseconds)

            generate_message(nav_slave.getNameRobot(), feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time)

            # Some navigation timeout to demo cancellation
            if now - nav_start > Duration(seconds=600.0):
                nav_slave.cancelTask()

            # Some follow waypoints request change to demo preemption
            if now - nav_start > Duration(seconds=10.0):
                print("Tiempo de navegación excedido")

                routes_remaining = len(goal_poses_robot) - (feedback.current_waypoint)
                nav_slave.cancelTask()
                
                print("Rutas restantes por completar: " + str(routes_remaining))
                if routes_remaining > 0:
                    print("Master robot: " + name_master)
                    master_robot = master_robots[name_master]

                    nav_master = master_robot['nav_client']
                    
                    if nav_master.getFeedback() is None:
                        print("Master disponible, se efectuará la tarea.")
                        
                        tasks_master_robots[nav_master.getNameRobot()][nav_slave.getNameRobot()] = goal_poses_robot[feedback.current_waypoint:]

                        nav_start_master = nav_master.get_clock().now()

                        await asyncio.gather(navigate_robot_master(nav_master, nav_slave.getNameRobot()))
                    else:                
                        print("Master ocupado, no se puede efectuar la tarea.")
                        print("Colocando la respuesta de la meta a la lista de tareas pendientes del maestro")
            
                        tasks_master_robots[nav_master.getNameRobot()][nav_slave.getNameRobot()] = goal_poses_robot[feedback.current_waypoint:]

                        nav_start_master = nav_master.get_clock().now()

                        await asyncio.gather(navigate_robot_master(nav_master, nav_slave.getNameRobot()))

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

    list_nav_func = []
    for robot in data_nav_robots.generate_robots():

        name_robot = robot['name']

        if not robot['is_master']:
            nav_slave = BasicNavigator(namespace=name_robot)

            list_poses_wo_process = data_nav_robots.get_list_poses(robot)  # obtener lista de poses sin convertir en PoseStamped
            list_poses_w_process = pose_utils.create_poses(list_poses_wo_process)  # convertir a PoseStamped
            goal_poses_robot = list_poses_w_process
            name_master_robot = robot['name_master']

            nav_start = nav_slave.get_clock().now()

            list_nav_func.append(navigate_robot_slave(nav_slave, goal_poses_robot, nav_start, name_master_robot))
        else:
            nav_master = BasicNavigator(namespace=name_robot)

            master_robots[name_robot] = robot
            master_robots[name_robot]['nav_client'] = nav_master

            tasks_master_robots[name_robot] = {} 


    await asyncio.gather(*list_nav_func)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
