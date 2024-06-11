#!/usr/bin/env python3

from rclpy.duration import Duration
import asyncio
from robot import Robot

class NavigateMaster(Robot):
    def __init__(self, nav_master, name_slave):
        self.nav_master = nav_master
        self.name_slave = name_slave

    async def navigate_robot_master(self, system_master_slave, duration_max_time=Duration(seconds=600.0)):
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

                        max_time = self.nav_master.getTimeNav(duration_max_time.nanoseconds)
                        
                        super().generate_message(name_master, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave)

                        if now - nav_start >= duration_max_time:
                            system_master_slave[name_master]["status"] = super().cancel_task(self.nav_master)

                self.nav_master.info("Tarea completada")
                list_slave_tasks.pop(name_first_slave)
                self.nav_master.info("Tarea eliminada de la lista de tareas pendientes")

                break
            else:
                print("MSJ del maestro: " + name_master + " => El esclavo " + self.name_slave + " está en cola de espera. Ahora ejecuto la tarea del robot " + name_first_slave)
                await asyncio.sleep(1)