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

# ---------------------------------------------
# --- Imprimir mensaje de navegación actual ---
# ---------------------------------------------
def generate_message(name_robot, current_waypoint, number_poses, nav_time=(0,0,0), max_time=(0,0,0), name_slave=None):
    # tiempo transcurrido de navegación
    hour_nav, min_nav, sec_nav = nav_time
    # tiempo máximo de navegación
    hour_max, min_max, sec_max = max_time

    msg = 'Executing current waypoint ' + str(name_robot) + ': '+ str(current_waypoint + 1) + '/' + str(number_poses) 
    msg += ' - ' + str(hour_nav-9) + ':' + str(min_nav) + ':' + str(sec_nav) + ' / ' + str(hour_max-9) + ':' + str(min_max) + ':' + str(sec_max)

    # validar si la navegación es tarea pendiente de un esclavo
    if name_slave is not None: 
        msg += " del esclavo: " + name_slave
    
    print(msg)

# ---------------------------------------------
# ----- Navegación de los robots maestros -----
# ---------------------------------------------
async def navigate_robot_master(nav_master, name_slave):

    while system_master_slave[nav_master.getNameRobot()]["slave_tasks"]:

        name_first_slave = list(system_master_slave[nav_master.getNameRobot()]["slave_tasks"])[0]

        if name_first_slave == name_slave:
            nav_master.info("Completando tarea pendiente del robot esclavo: " + name_slave)

            goal_poses_robot = system_master_slave[nav_master.getNameRobot()]["slave_tasks"][name_first_slave]
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

                    if now - nav_start > Duration(seconds=600.0):
                        nav_master.cancelTask()
                        system_master_slave[name_master]["status"] = False

            nav_master.info("Tarea completada")
            system_master_slave[nav_master.getNameRobot()]["slave_tasks"].pop(name_first_slave)
            nav_master.info("Tarea eliminada de la lista de tareas pendientes")

            break
        else:
            print("El esclavo " + name_slave + " esta esperando a que el esclavo " + name_first_slave + " complete su tarea")
            await asyncio.sleep(1)

# ---------------------------------------------
# ----- Navegación de los robots esclavos -----
# ---------------------------------------------
async def navigate_robot_slave(nav_slave, name_master, name_slave_pend=None):

    slave = system_master_slave[name_master]["slaves"][nav_slave.getNameRobot()]
    while slave["task_queue"]:

        name_first_slave_task = list(slave["task_queue"])[0]

        if name_slave_pend is None:
            name_slave_pend = nav_slave.getNameRobot()

        if name_first_slave_task == name_slave_pend:
            
            goal_poses_robot = slave["task_queue"][name_first_slave_task]
            nav_slave.followWaypoints(goal_poses_robot)
            nav_start = nav_slave.get_clock().now()

            while not nav_slave.isTaskComplete():
                
                await asyncio.sleep(1)
                feedback = nav_slave.getFeedback()
                
                if feedback:
                    now = nav_slave.get_clock().now()
                    nav_time = nav_slave.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
                    
                    duration_max_time = Duration(seconds=10.0)
                    max_time = nav_slave.getTimeNav(duration_max_time.nanoseconds)
                    
                    if name_slave_pend == nav_slave.getNameRobot():
                        generate_message(nav_slave.getNameRobot(), feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time)
                    else:
                        generate_message(nav_slave.getNameRobot(), feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, name_slave_pend)

                    if now - nav_start > Duration(seconds=600.0):
                        nav_master.cancelTask()
                        slave["status"] = False
                    
                    if now - nav_start > duration_max_time:
                        routes_remaining = len(goal_poses_robot) - (feedback.current_waypoint)
                        nav_slave.cancelTask()
                        slave["status"] = False
                        
                        if routes_remaining > 0:

                            found_free_slave, free_slave = find_free_slave(name_master)
                            found_slave_with_one_task, slave_with_one_task = find_slave_with_one_task(name_master, nav_slave.getNameRobot())

                            list_slaves = system_master_slave[name_master]["slaves"]
                            if found_free_slave:
                                nav_free_slave = list_slaves[free_slave]["nav_class"]
                                list_slaves[free_slave]["task_queue"][nav_slave.getNameRobot()] = goal_poses_robot[feedback.current_waypoint:]

                                await asyncio.gather(navigate_robot_slave(nav_free_slave, name_master, nav_slave.getNameRobot()))
                            elif found_slave_with_one_task:
                                nav_slave_with_one_task = list_slaves[slave_with_one_task]["nav_class"]
                                list_slaves[slave_with_one_task]["task_queue"][nav_slave.getNameRobot()] = goal_poses_robot[feedback.current_waypoint:]

                                nav_slave.info("El esclavo " + slave_with_one_task + " tiene una tarea pendiente, se le asignará esta tarea")
                                await asyncio.gather(navigate_robot_slave(nav_slave_with_one_task, name_master, nav_slave.getNameRobot()))
                            else:
                                nav_master = system_master_slave[name_master]["nav_class"]
                                system_master_slave[name_master]["slave_tasks"][nav_slave.getNameRobot()] = goal_poses_robot[feedback.current_waypoint:]
                                
                                is_master_busy(nav_master.getFeedback())

                                await asyncio.gather(navigate_robot_master(nav_master, nav_slave.getNameRobot()))

            nav_slave.info("Tarea completada")
            system_master_slave[name_master]["slaves"][nav_slave.getNameRobot()]["task_queue"].pop(name_first_slave_task)
            nav_slave.info("Tarea eliminada de la lista de tareas pendientes")

            break
        else:
            if name_slave_pend != nav_slave.getNameRobot():
                print("El esclavo " + name_slave_pend + " esta esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna.")
            await asyncio.sleep(1)

# ---------------------------------------------
# --- Verificar si el maestro esta ocupado ----
# ---------------------------------------------
def is_master_busy(nav_master_feedback):
    if nav_master_feedback is None:
        print("Master disponible, se efectuará la tarea de manera inmediata.")
    else:                
        print("Master ocupado, esta en línea de espera.")

# ---------------------------------------------
#  Encontrar esclavo disponible con una tarea 
# ---------------------------------------------
def find_slave_with_one_task(name_master, name_slave):
    print("---> Buscando esclavo con una tarea pendiente que pueda realizar la tarea... <---")
    find_slave_with_one_task = False
    
    list_slaves = system_master_slave[name_master]["slaves"]
    for name_slave_iter in list_slaves:
        slave = list_slaves[name_slave_iter]
        status_slave = slave["status"]
        
        if status_slave and name_slave != name_slave_iter:
            nav_slave = slave["nav_class"]

            task_queue = slave["task_queue"]
            size_task_queue = len(task_queue)

            if size_task_queue == 1:
                name_slave_task_queue = list(task_queue)[0] 
                len_goal_poses = len(task_queue[name_slave_task_queue])
                current_waypoint = nav_slave.getFeedback().current_waypoint

                if (len_goal_poses - current_waypoint) <= 1:
                    print("El esclavo " + name_slave_iter + " tiene una tarea pendiente, es elegible para realizar la tarea")
                    find_slave_with_one_task = True
                    return find_slave_with_one_task, name_slave_iter
                else:
                    print("El esclavo " + name_slave_iter + " tiene mas de una tarea pendiente, no es elegible para realizar la tarea")

    print("No hay esclavos disponibles para realizar la tarea")
    return find_slave_with_one_task, None

# ---------------------------------------------
# -- Encontrar esclavo disponible sin tareas --
# ---------------------------------------------           
def find_free_slave(name_master):
    print("---> Buscando esclavo sin tareas que este disponible para realizar la tarea... <---")
    find_free_slave = False

    list_slaves = system_master_slave[name_master]["slaves"]
    for slave_name in list_slaves:
        slave = list_slaves[slave_name]
        status_slave = slave["status"]

        if status_slave:
            nav_slave = slave["nav_class"]

            if nav_slave.isTaskComplete():
                print("El esclavo " + slave_name + " esta disponible para realizar la tarea")
                find_free_slave = True
                return find_free_slave, slave_name
            else:
                print("Hay un esclavo disponible pero esta ocupado en otra tarea")

    print("No hay esclavos disponibles para realizar la tarea")
    return find_free_slave, None

# ---------------------------------------------
# ------------ Función principal --------------
# ---------------------------------------------
async def main(args=None):
    if args is None:
        args = sys.argv

    if len(args) != 2:
        print("Usage: ros2 run multi_robot_move nav_master_slave.py <path_to_yaml_file>")
        return

    yaml_file_path = args[1]

    rclpy.init(args=args)

    data_nav_robots = DataRobots(yaml_file_path)

    list_nav_func = []
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
            dict_master["slaves"][name_robot] = {"nav_class": nav_slave, "master": name_master_robot, "tasks": goal_poses_robot, "task_queue": {name_robot: goal_poses_robot},"status": True}
            
            list_nav_func.append(navigate_robot_slave(nav_slave, name_master_robot))
    
    await asyncio.gather(*list_nav_func)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
