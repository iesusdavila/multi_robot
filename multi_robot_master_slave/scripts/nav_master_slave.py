#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from NavigationClient import NavigationRobot, TaskResult
from DelegateTask import FreeSlaveHandler, SlaveWithOneTaskHandler, MasterHandler
from PoseUtils import PoseUtils
from DataRobots import DataRobots
import time
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
    msg += ' - ' + str(hour_nav-19) + ':' + str(min_nav) + ':' + str(sec_nav) + ' / ' + str(hour_max-19) + ':' + str(min_max) + ':' + str(sec_max)

    # validar si la navegación es tarea pendiente de un esclavo
    if name_slave is not None: 
        msg += " del esclavo: " + name_slave
    
    print(msg)

# ---------------------------------------------
# ----- Navegación de los robots maestros -----
# ---------------------------------------------
async def navigate_robot_master(nav_master, name_slave):
    name_master = nav_master.getNameRobot()
    list_slave_tasks = system_master_slave[name_master]["slave_tasks"]
    while list_slave_tasks:

        name_first_slave = list(list_slave_tasks)[0]

        if name_first_slave == name_slave:
            nav_master.info("Completando tarea pendiente del robot esclavo: " + name_slave)

            goal_poses_robot = list_slave_tasks[name_first_slave]
            nav_master.followWaypoints(goal_poses_robot)

            nav_start = nav_master.get_clock().now()

            while not nav_master.isTaskComplete():
                
                await asyncio.sleep(1)
                feedback = nav_master.getFeedback()
                
                if feedback:
                    now = nav_master.get_clock().now()
                    nav_time = nav_master.getTimeNav(now.nanoseconds - nav_start.nanoseconds)

                    max_time = nav_master.getTimeNav(Duration(seconds=600.0).nanoseconds)
                    
                    generate_message(name_master, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, name_slave)

                    if now - nav_start >= Duration(seconds=600.0):
                        cancel_task_master(nav_master, name_master)

            nav_master.info("Tarea completada")
            list_slave_tasks.pop(name_first_slave)
            nav_master.info("Tarea eliminada de la lista de tareas pendientes")

            break
        else:
            print("MSJ del maestro: " + name_master + " => El esclavo " + name_slave + " está en cola de espera. Ahora ejecuto la tarea del robot " + name_first_slave)
            await asyncio.sleep(1)

# ---------------------------------------------
# ----- Navegación de los robots esclavos -----
# ---------------------------------------------
async def navigate_robot_slave(nav_slave, name_master, name_slave_pend=None):

    name_slave = nav_slave.getNameRobot()
    slave = system_master_slave[name_master]["slaves"][name_slave]
    while slave["task_queue"]:

        name_first_slave_task = list(slave["task_queue"])[0]

        if name_slave_pend is None:
            name_slave_pend = name_slave

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
                    
                    if name_slave_pend == name_slave:
                        generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time)
                    else:
                        generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, name_slave_pend)

                    if now - nav_start >= Duration(seconds=600.0):
                        cancel_task_slave(nav_slave, slave)
                    
                    if now - nav_start >= duration_max_time:
                        routes_remaining = len(goal_poses_robot) - (feedback.current_waypoint)
                        cancel_task_slave(nav_slave, slave)
                        
                        if routes_remaining > 0:
                            await send_goal_other_robot(name_master, nav_slave, goal_poses_robot, feedback)

            nav_slave.info("Tarea completada")
            system_master_slave[name_master]["slaves"][name_slave]["task_queue"].pop(name_first_slave_task)
            nav_slave.info("Tarea eliminada de la lista de tareas pendientes")

            break
        else:
            if name_slave_pend != name_slave:
                print("El esclavo " + name_slave_pend + " está esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna.")
            await asyncio.sleep(1)

# ---------------------------------------------
#  Delegar tarea a otro robot para que la haga 
# ---------------------------------------------
async def send_goal_other_robot(name_master, nav_slave, goal_poses_robot, feedback):
    await asyncio.sleep(1)

    name_slave = nav_slave.getNameRobot()
    
    request = {
        'system_master_slave': system_master_slave,
        'name_master': name_master,
        'name_slave': name_slave,
        'goal_poses': goal_poses_robot,
        'current_waypoint': feedback.current_waypoint,
        'slaves': system_master_slave[name_master]["slaves"]
    }

    chain = MasterHandler(SlaveWithOneTaskHandler(FreeSlaveHandler()))
    handler, is_free_or_one_task = chain.handle_request(request)
    
    if is_free_or_one_task:
        await asyncio.gather(navigate_robot_slave(handler, name_master, name_slave))
    else:
        await asyncio.gather(navigate_robot_master(handler, name_slave))

# ---------------------------------------------
# ------- Cancelar tarea del maestro ----------
# ---------------------------------------------
def cancel_task_master(nav_master):
    nav_master.cancelTask()
    system_master_slave[nav_master.getNameRobot()]["status"] = False

# ---------------------------------------------
# -------- Cancelar tarea del esclavo ---------
# ---------------------------------------------
def cancel_task_slave(nav_slave, slave):
    nav_slave.cancelTask()
    slave["status"] = False

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
            nav_master = NavigationRobot(namespace=name_robot)

            system_master_slave[name_robot] = {"nav_class": nav_master, "slaves": {}, "slave_tasks": {}, "status": True}
        
        if not is_master:
            nav_slave = NavigationRobot(namespace=name_robot)

            list_poses_wo_process = data_nav_robots.get_list_poses(robot)  # obtener lista de poses sin convertir en PoseStamped
            goal_poses_robot = PoseUtils.create_poses(list_poses_wo_process)  # convertir a PoseStamped
            
            name_master_robot = robot['name_master']                

            if not(name_master_robot in system_master_slave):
                nav_master = NavigationRobot(namespace=name_master_robot)

                system_master_slave[name_master_robot] = {"nav_class": nav_master, "slaves": {}, "slave_tasks": {}, "status": True}
            
            dict_master = system_master_slave[name_master_robot]
            dict_master["slaves"][name_robot] = {"nav_class": nav_slave, "master": name_master_robot, "tasks": goal_poses_robot, "task_queue": {name_robot: goal_poses_robot},"status": True}
            
            list_nav_func.append(navigate_robot_slave(nav_slave, name_master_robot))
    
    await asyncio.gather(*list_nav_func)

    rclpy.shutdown()

if __name__ == '__main__':
    asyncio.run(main())
