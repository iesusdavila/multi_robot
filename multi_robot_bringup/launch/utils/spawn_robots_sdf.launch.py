import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    urdf_path = LaunchConfiguration('urdf_path', default='')
    robot_namespace = LaunchConfiguration('robot_namespace', default='tb')
    robot_name = LaunchConfiguration('robot_name', default='tb')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')
    z_pose = LaunchConfiguration('z_pose', default='0.01')
    yaw = LaunchConfiguration('yaw', default='0.0')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true')
        
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path', default_value='',
        description='Urdf path of the robot')

    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='tb',
        description='Specify namespace of the robot')
    
    declare_robot_name = DeclareLaunchArgument(
        'robot_name', default_value='tb',
        description='Specify name of the robot')
    
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0',
        description='Specify x position of the robot')
    
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0',
        description='Specify y position of the robot')
    
    declare_z_pose = DeclareLaunchArgument(
        'z_pose', default_value='0.01',
        description='Specify z position of the robot')
    
    declare_yaw = DeclareLaunchArgument(
        'yaw', default_value='0.0',
        description='Specify yaw of the robot')

    spawn_robot_cmd = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-file", urdf_path,
            "-entity", robot_name,
            "-robot_namespace", robot_namespace,
            "-x", x_pose,
            "-y", y_pose,
            "-z", z_pose,
            "-Y", yaw,
            "-unpause",
        ],
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_urdf_path)
    ld.add_action(declare_robot_namespace)
    ld.add_action(declare_robot_name)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_z_pose)
    ld.add_action(declare_yaw)

    ld.add_action(spawn_robot_cmd)

    return ld
