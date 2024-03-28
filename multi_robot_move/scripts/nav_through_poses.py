#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses
from geometry_msgs.msg import PoseStamped, Quaternion
import time
import math
import yaml
import os
from ament_index_python.packages import get_package_share_directory

class NavigationClient(Node):
    def __init__(self, name_robot):
        super().__init__('navigation_to_poses_client_{}'.format(name_robot))
        
        self.name_robot = name_robot
        self.action_client = ActionClient(self, NavigateThroughPoses, '/{}/navigate_through_poses'.format(name_robot))
        self.action_client.wait_for_server()

    def saludar(self):
        print('Hola, soy el robot {}'.format(self.name_robot))

    def send_goal(self, goal_poses):
        print('Enviando la acción de navegación al robot {}'.format(self.name_robot))
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = goal_poses

        send_goal_future = self.action_client.send_goal_async(goal_msg)

        print('Esperando respuesta del robot {}'.format(self.name_robot))
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=1.0)

        if send_goal_future.result() is not None:
            print('¡Se recibió una respuesta del robot {}!'.format(self.name_robot))
        else:
            print('No se recibió respuesta del robot {}. Tiempo de espera agotado.'.format(self.name_robot))

    def destroy(self):
        self.action_client.destroy()

class PoseUtils:
    def __init__(self):
        pass

    def quaternion_from_euler(self, roll, pitch, yaw):
        cuaternion = Quaternion()

        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        q = [0] * 4
        q[0] = cy * cp * cr + sy * sp * sr
        q[1] = cy * cp * sr - sy * sp * cr
        q[2] = sy * cp * sr + cy * sp * cr
        q[3] = sy * cp * cr - cy * sp * sr

        cuaternion.x = q[0]
        cuaternion.y = q[1]
        cuaternion.z = q[2]
        cuaternion.w = q[3]

        return cuaternion

    def create_pose(self, x, y, z, yaw=0):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation = self.quaternion_from_euler(0, 0, yaw)

        return pose

class DataRobots:
    def __init__(self, path_file):
        self.path_file = path_file

    def generate_robots(self):
        robots_file = self.path_file 
        with open(robots_file, 'r') as file:
            robots_data = yaml.safe_load(file)

        return robots_data['robots']

def main(args=None):
    rclpy.init(args=args)

    pose_utils = PoseUtils()
    date_robots = DataRobots('/home/rov-robot/project_ws/src/multi_robot/multi_robot_gazebo/config/robots_world.yaml')
    
    print(date_robots.generate_robots()[2]['name'])
    print(date_robots.generate_robots()[1]['name'])

    # Define la posición de destino
    goal_pose = pose_utils.create_pose(1.5, -0.5, 0.0, 0.0)
    goal_pose2 = pose_utils.create_pose(0.0, 0.5, 0.0, 0.0)
    goal_pose3 = pose_utils.create_pose(-1.0, 0.5, 0.0, 0.0)

    goal_poses_robot_1 = [goal_pose, goal_pose2]
    goal_poses_robot_2 = [goal_pose2, goal_pose3]

    # Crear instancias de la clase NavigationClient para cada robot
    print('Creando instancias de NavigationClient')
    navigation_client_1 = NavigationClient(date_robots.generate_robots()[2]['name'])
    navigation_client_2 = NavigationClient(date_robots.generate_robots()[1]['name'])

    print('Enviando acciones de navegación a los robots')
    navigation_client_1.saludar()
    navigation_client_2.saludar()
    # Envía la acción de navegación al robot 1
    navigation_client_1.send_goal(goal_poses_robot_1)

    # Espera un momento antes de enviar la acción al siguiente robot (solo para ejemplo)
    time.sleep(2)

    # Envía la acción de navegación al robot 2
    navigation_client_2.send_goal(goal_poses_robot_2)
    
    # Destruye las instancias de NavigationClient
    navigation_client_1.destroy()
    navigation_client_2.destroy()

    rclpy.shutdown()

if __name__ == '__main__':
    main()
