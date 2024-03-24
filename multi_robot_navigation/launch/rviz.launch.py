import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString


def generate_launch_description():
    bringup_dir = get_package_share_directory('nav2_bringup')

    namespace = LaunchConfiguration('namespace')
    rviz_config_file = LaunchConfiguration('rviz_config')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(bringup_dir, 'rviz', 'nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    rviz_config_file = ReplaceString(
            source_file=rviz_config_file,
            replacements={'<robot_namespace>': ('/', namespace)})

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        namespace=namespace,
        arguments=['-d', rviz_config_file],
        output='screen',
        remappings=[('/tf', 'tf'),
                    ('/tf_static', 'tf_static'),
                    ('/goal_pose', 'goal_pose'),
                    ('/clicked_point', 'clicked_point'),
                    ('/initialpose', 'initialpose')])

    exit_event_handler = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=rviz_cmd,
            on_exit=EmitEvent(event=Shutdown(reason='rviz exited'))))

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_rviz_config_file_cmd)

    ld.add_action(rviz_cmd)

    ld.add_action(exit_event_handler)

    return ld
