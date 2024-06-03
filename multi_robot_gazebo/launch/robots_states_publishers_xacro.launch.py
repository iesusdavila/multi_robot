import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro

def execute_rsp(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    xacro_path = LaunchConfiguration('xacro_path', default=os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_burger.urdf'))
    robot_namespace = LaunchConfiguration('robot_namespace', default='tb')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    nodes_exec = []

    robot_description_config = xacro.process_file(xacro_path.perform(context))
    turtlebot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=robot_namespace,
        executable="robot_state_publisher",
        parameters=[{
                    "robot_description": robot_description_config.toxml(),
                    "use_sim_time": False,
                    "publish_frequency": 10.0}],
        output="both",
        remappings=remappings,
    )

    nodes_exec.append(turtlebot_state_publisher)

    return nodes_exec

def generate_launch_description():

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation/Gazebo clock if true')
    declare_xacro_path = DeclareLaunchArgument(
        'xacro_path', default_value=os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'urdf', 'turtlebot3_burger.urdf'),
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
