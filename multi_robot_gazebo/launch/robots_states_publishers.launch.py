import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    urdf_file_name = 'turtlebot3_burger.urdf'
    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'urdf',
        urdf_file_name)

    robot_namespace = LaunchConfiguration('robot_namespace', default='tb')

    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='tb',
        description='Specify namespace of the robot')

    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    turtlebot_state_publisher = Node(
        package="robot_state_publisher",
        namespace=robot_namespace,
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": False,
                    "publish_frequency": 10.0}],
        remappings=remappings,
        arguments=[urdf_path],
    )

    ld = LaunchDescription()

    ld.add_action(declare_robot_namespace)
    ld.add_action(turtlebot_state_publisher)

    return ld
