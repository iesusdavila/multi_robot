#!/usr/bin/env python3

import yaml

class DataRobots:
    def __init__(self, path_file):
        self.path_file = path_file
        self.robots = self.generate_robots()

    def generate_robots(self):
        robots_file = self.path_file 
        with open(robots_file, 'r') as file:
            robots_data = yaml.safe_load(file)

        return robots_data['robots']
    
    def get_name(self, robot):
        return robot['name']

    def get_number_poses(self, robot):
        return len(robot) - 3

    def get_pose(self, robot, index):
        param_goal_pose = "pose_goal_" + str(index+1)
        return robot[param_goal_pose]

    def get_list_poses(self, robot):
        poses = []
        for i in range(self.get_number_poses(robot)):
            poses.append(self.get_pose(robot, i))
        return poses