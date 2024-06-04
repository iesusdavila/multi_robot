import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def execute_rsp(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    xacro_path = LaunchConfiguration('xacro_path', default='')
    robot_namespace = LaunchConfiguration('robot_namespace', default='tb')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    nodes_exec = []

    robot_description_config = xacro.process_file(xacro_path.perform(context))
    robot_state_publisher_cmd = Node(
        package="robot_state_publisher",
        namespace=robot_namespace,
        executable="robot_state_publisher",
        parameters=[{
                    "robot_description": robot_description_config.toxml(),
                    "use_sim_time": use_sim_time,
                    "publish_frequency": 10.0}],
        output="both",
        remappings=remappings,
    )

    nodes_exec.append(robot_state_publisher_cmd)

    return nodes_exec

def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation/Gazebo clock if true')
    declare_xacro_path = DeclareLaunchArgument(
        'xacro_path', default_value='',
        description='Xacro path of the robot')
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='tb',
        description='Specify namespace of the robot')

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_xacro_path)
    ld.add_action(declare_robot_namespace)
    ld.add_action(OpaqueFunction(function=execute_rsp))

    return ld
