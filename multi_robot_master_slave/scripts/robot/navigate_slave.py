#!/usr/bin/env python3

from delegate_task import FreeSlaveHandler, SlaveWithOneTaskHandler, MasterHandler
from navigate_master import NavigateMaster
from navigation_client import TaskResult
from robot import Robot
from rclpy.duration import Duration
import asyncio

class NavigateSlave(Robot):
    def __init__(self, nav_slave, name_master, name_slave_pend=None):
        self.nav_slave = nav_slave
        self.name_master = name_master
        self.name_slave_pend = name_slave_pend

    async def navigate_robot_slave(self, system_master_slave):

        name_slave = self.nav_slave.getNameRobot()
        slave = system_master_slave[self.name_master]["slaves"][name_slave]
        while slave["task_queue"]:

            name_first_slave_task = list(slave["task_queue"])[0]

            if self.name_slave_pend is None:
                self.name_slave_pend = name_slave

            if name_first_slave_task == self.name_slave_pend:
                
                goal_poses_robot = slave["task_queue"][name_first_slave_task]["goal_poses"]
                self.nav_slave.followWaypoints(goal_poses_robot)
                nav_start = self.nav_slave.get_clock().now()

                while not self.nav_slave.isTaskComplete():
                    
                    await asyncio.sleep(1)
                    feedback = self.nav_slave.getFeedback()
                    
                    if feedback:
                        now = self.nav_slave.get_clock().now()
                        nav_time = self.nav_slave.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
                        
                        duration_max_time_m = slave["task_queue"][name_first_slave_task]["duration_max_time"]
                        duration_max_time=Duration(seconds=duration_max_time_m*60)
                        max_time = self.nav_slave.getTimeNav(duration_max_time.nanoseconds)
                        
                        if self.name_slave_pend == name_slave:
                            super().generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time)
                        else:
                            super().generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave_pend)

                        if now - nav_start >= Duration(seconds=600.0):
                            slave["status"] = super().cancel_task(self.nav_slave)
                            self.nav_slave.info("Tarea NO completada en el tiempo establecido.")
                            system_master_slave[self.name_master]["slaves"][name_slave]["task_queue"].pop(name_first_slave_task)
                            self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes")
                        
                        if now - nav_start >= duration_max_time:
                            routes_remaining = len(goal_poses_robot) - (feedback.current_waypoint)
                            slave["status"] = super().cancel_task(self.nav_slave)
                            
                            self.nav_slave.info("Tarea NO completada en el tiempo establecido.")
                            system_master_slave[self.name_master]["slaves"][name_slave]["task_queue"].pop(name_first_slave_task)
                            self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes")

                            if routes_remaining > 0:
                                await self.send_goal_other_robot(self.name_master, self.nav_slave, goal_poses_robot, duration_max_time_m, feedback, system_master_slave)
                
                if self.nav_slave.getResult() == TaskResult.SUCCEEDED:
                    self.nav_slave.info("Tarea completada")
                    system_master_slave[self.name_master]["slaves"][name_slave]["task_queue"].pop(name_first_slave_task)
                    self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes.")

                break
            else:
                if self.name_slave_pend != name_slave:
                    print("El esclavo " + self.name_slave_pend + " está esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna.")
                await asyncio.sleep(1)

    async def send_goal_other_robot(self, name_master, nav_slave, goal_poses_robot, duration_max_time, feedback, system_master_slave):
        await asyncio.sleep(1)

        name_slave = nav_slave.getNameRobot()
        nav_master = system_master_slave[name_master]["nav_class"]
        list_slaves = system_master_slave[name_master]["slaves"]
        
        request = {
            'dict_master': system_master_slave[name_master],
            'nav_slave': nav_slave,
            'goal_poses': goal_poses_robot,
            'current_waypoint': feedback.current_waypoint,
            'duration_max_time': duration_max_time,
        }

        free_slave_handler = FreeSlaveHandler()
        slave_with_one_task_handler = SlaveWithOneTaskHandler()
        master_handler = MasterHandler()

        free_slave_handler.set_next(slave_with_one_task_handler).set_next(master_handler)
        
        handler, is_free_or_one_task = free_slave_handler.handle(request)
        
        #name_slave: nombre del robot que esta pidiendo ayuda de su tarea pendiente
        #nav_slave: robot que esta socorriendo a la ayuda de la tarea pendiente
        if is_free_or_one_task:
            slave_robot = NavigateSlave(handler, name_master, name_slave)
            await asyncio.gather(slave_robot.navigate_robot_slave(system_master_slave))
        else:
            master_robot = NavigateMaster(handler, name_slave)
            await asyncio.gather(master_robot.navigate_robot_master(system_master_slave))