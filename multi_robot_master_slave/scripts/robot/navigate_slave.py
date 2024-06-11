#!/usr/bin/env python3

from delegate_task import FreeSlaveHandler, SlaveWithOneTaskHandler, MasterHandler
from navigate_master import NavigateMaster
from rclpy.duration import Duration
import asyncio

class NavigateSlave():
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
                
                goal_poses_robot = slave["task_queue"][name_first_slave_task]
                self.nav_slave.followWaypoints(goal_poses_robot)
                nav_start = self.nav_slave.get_clock().now()

                while not self.nav_slave.isTaskComplete():
                    
                    await asyncio.sleep(1)
                    feedback = self.nav_slave.getFeedback()
                    
                    if feedback:
                        now = self.nav_slave.get_clock().now()
                        nav_time = self.nav_slave.getTimeNav(now.nanoseconds - nav_start.nanoseconds)
                        
                        duration_max_time = Duration(seconds=10.0)
                        max_time = self.nav_slave.getTimeNav(duration_max_time.nanoseconds)
                        
                        if self.name_slave_pend == name_slave:
                            self.generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time)
                        else:
                            self.generate_message(name_slave, feedback.current_waypoint, len(goal_poses_robot), nav_time, max_time, self.name_slave_pend)

                        if now - nav_start >= Duration(seconds=600.0):
                            self.cancel_task_slave(slave)
                        
                        if now - nav_start >= duration_max_time:
                            routes_remaining = len(goal_poses_robot) - (feedback.current_waypoint)
                            self.cancel_task_slave(slave)
                            
                            if routes_remaining > 0:
                                await self.send_goal_other_robot(self.name_master, self.nav_slave, goal_poses_robot, feedback, system_master_slave)

                self.nav_slave.info("Tarea completada")
                system_master_slave[self.name_master]["slaves"][name_slave]["task_queue"].pop(name_first_slave_task)
                self.nav_slave.info("Tarea eliminada de la lista de tareas pendientes")

                break
            else:
                if self.name_slave_pend != name_slave:
                    print("El esclavo " + self.name_slave_pend + " está esperando que la tarea enviada al esclavo " + name_first_slave_task + " sea completada una vez que dicho esclavo complete su tarea interna.")
                await asyncio.sleep(1)

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

    def cancel_task_slave(self, slave):
        self.nav_slave.cancelTask()
        slave["status"] = False

    async def send_goal_other_robot(self, name_master, nav_slave, goal_poses_robot, feedback, system_master_slave):
        await asyncio.sleep(1)

        name_slave = nav_slave.getNameRobot()
        nav_master = system_master_slave[name_master]["nav_class"]
        list_slaves = system_master_slave[name_master]["slaves"]
        
        request = {
            'dict_master': system_master_slave[name_master],
            'nav_slave': nav_slave,
            'goal_poses': goal_poses_robot,
            'current_waypoint': feedback.current_waypoint,
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