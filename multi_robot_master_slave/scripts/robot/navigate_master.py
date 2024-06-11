#!/usr/bin/env python3

from rclpy.duration import Duration
import asyncio

class NavigateMaster():
    def __init__(self, nav_master, name_slave):
        self.nav_master = nav_master
        self.name_slave = name_slave

    async def navigate_robot_master(self, system_master_slave):
        name_master = self.nav_master.getNameRobot()
        list_slave_tasks = system_master_slave[name_master]["slave_tasks"]
        while list_slave_tasks:

            name_first_slave = list(list_slave_tasks)[0]

            if name_first_slave == self.name_slave:
                self.nav_master.info("Completando tarea pendiente del robot esclavo: " + self.name_slave)

                goal_poses_robot = list_slave_tasks[name_first_slave]
                self.nav_master.followWaypoints(goal_poses_robot)

                nav_start = self.nav_master.get_clock().now()

                while not self.nav_master.isTaskComplete():
                    
                    await asyncio.sleep(1)
                    feedback = self.nav_master.getFeedback()
                    
                    if feedback:
                        now = self.nav_master.get_clock().now()
                        nav_time = self.nav_master.getTimeNav(now.nanoseconds - nav_start.nanoseconds)

                        max_time = self.nav_master.getTimeNav(Duration(seconds=600.0).nanoseconds)
                        
                        self.generate_message(name_master, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave)

                        if now - nav_start >= Duration(seconds=600.0):
                            self.cancel_task_master(system_master_slave)

                self.nav_master.info("Tarea completada")
                list_slave_tasks.pop(name_first_slave)
                self.nav_master.info("Tarea eliminada de la lista de tareas pendientes")

                break
            else:
                print("MSJ del maestro: " + name_master + " => El esclavo " + self.name_slave + " está en cola de espera. Ahora ejecuto la tarea del robot " + name_first_slave)
                await asyncio.sleep(1)

    def cancel_task_master(self, system_master_slave):
        self.nav_master.cancelTask()
        system_master_slave[self.nav_master.getNameRobot()]["status"] = False

    def generate_message(self, name_robot, current_waypoint, number_poses, nav_time=(0,0,0), max_time=(0,0,0), name_slave=None):
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
