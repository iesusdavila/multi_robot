#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses

class NavigationClient(Node):
    def __init__(self, name_robot):
        super().__init__('navigation_client_{}'.format(name_robot))
        
        self.name_robot = name_robot
        self.action_client_nav_pose = ActionClient(self, NavigateToPose, '/{}/navigate_to_pose'.format(name_robot))
        self.action_client_nav_through_poses = ActionClient(self, NavigateThroughPoses, '/{}/navigate_through_poses'.format(name_robot))
        self.action_client_follow_waypoints = ActionClient(self, NavigateThroughPoses, '/{}/follow_waypoints'.format(name_robot))

    def send_goal_pose(self, goal_pose):
        print("Waiting for 'NavigateToPose' action server")
        while not self.action_client_nav_pose.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateToPose' action server not available, waiting...")

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        send_goal_future = self.action_client_nav_pose.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if send_goal_future.result() is not None:
            print('¡Se recibió una respuesta del robot {}!'.format(self.name_robot))
        else:
            print('No se recibió respuesta del robot {}. Tiempo de espera agotado.'.format(self.name_robot))

    def send_goal_through_poses(self, goal_poses):
        print("Waiting for 'NavigateThroughPoses' action server")
        while not self.action_client_nav_through_poses.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'NavigateThroughPoses' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = goal_poses

        send_goal_future = self.action_client_nav_through_poses.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if send_goal_future.result() is not None:
            print('¡Se recibió una respuesta del robot {}!'.format(self.name_robot))
        else:
            print('No se recibió respuesta del robot {}. Tiempo de espera agotado.'.format(self.name_robot))
    
    def send_follow_waypoints(self, goal_poses):
        print("Waiting for 'FollowWaypoints' action server")
        while not self.action_client_nav_through_poses.wait_for_server(timeout_sec=1.0):
            self.get_logger().info("'FollowWaypoints' action server not available, waiting...")

        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = goal_poses

        send_goal_future = self.action_client_nav_through_poses.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)

        if send_goal_future.result() is not None:
            print('¡Se recibió una respuesta del robot {}!'.format(self.name_robot))
        else:
            print('No se recibió respuesta del robot {}. Tiempo de espera agotado.'.format(self.name_robot))
    
    def destroy(self):
        self.action_client_nav_pose.destroy()
        self.action_client_nav_through_poses.destroy()
        self.action_client_follow_waypoints.destroy()
