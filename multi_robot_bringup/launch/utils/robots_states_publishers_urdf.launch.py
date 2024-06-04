import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation/Gazebo clock if true')

    urdf_path = LaunchConfiguration('urdf_path', default='')
    declare_urdf_path = DeclareLaunchArgument(
        'urdf_path', default_value='',
        description='Urdf path of the robot')

    robot_namespace = LaunchConfiguration('robot_namespace', default='tb')
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='tb',
        description='Specify namespace of the robot')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        namespace=robot_namespace,
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time,
                    "publish_frequency": 10.0}],
        remappings=remappings,
        arguments=[urdf_path],
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_urdf_path)
    ld.add_action(declare_robot_namespace)
    ld.add_action(robot_state_publisher_cmd)

    return ld
