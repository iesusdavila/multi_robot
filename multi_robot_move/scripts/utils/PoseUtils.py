#!/usr/bin/env python3

from geometry_msgs.msg import PoseStamped, Quaternion
import math

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
    
    def create_poses(self, list_poses):
        list_poses_w_process = []
        for pose in list_poses:
            x, y, z, yaw = pose.values()
            list_poses_w_process.append(self.create_pose(x, y, z, yaw))
        return list_poses_w_process
