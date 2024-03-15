import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition
import launch.logging

def generate_launch_description():
    ld = LaunchDescription()

    package_dir = get_package_share_directory('multi_robot_navigation')
    nav_launch_dir = os.path.join(package_dir, 'launch')

    robot_namespace = LaunchConfiguration('robot_namespace', default='/tb3_0')
    declare_robot_namespace = DeclareLaunchArgument(
        'robot_namespace', default_value='/tb3_0', description='Namespace of robot'
    )

    x_pose = LaunchConfiguration('x_pose', default='0.0')
    declare_x_pose = DeclareLaunchArgument(
        'x_pose', default_value='0.0', description='Initial x position of robot'
    )

    y_pose = LaunchConfiguration('y_pose', default='0.0')
    declare_y_pose = DeclareLaunchArgument(
        'y_pose', default_value='0.0', description='Initial y position of robot'
    )

    map_yaml_file = LaunchConfiguration('map_yaml_file', default=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'))
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map_yaml_file', default_value=os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml'),
        description='Full path to map file to load'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
 
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml_file}],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'bringup_nav.launch.py')
        ),
        launch_arguments={  
            'slam': 'False',
            'namespace': robot_namespace,
            'use_namespace': 'True',
            'map': map_yaml_file,
            'map_server': 'False',
            'params_file': params_file,
            'default_bt_xml_filename': os.path.join(
                get_package_share_directory('nav2_bt_navigator'),
                'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
            'autostart': 'true',
            'use_sim_time': use_sim_time, 'log_level': 'warn'
        }.items()
    )

    message = ['{header: {frame_id: map}, pose: {pose: {position: {x: ', x_pose, ', y: ', y_pose, ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}']

    initial_pose_cmd = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', [robot_namespace, '/initialpose'],
            'geometry_msgs/PoseWithCovarianceStamped', message],
        output='screen'
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_launch_dir, 'rviz.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time, 
            'namespace': robot_namespace,
            'use_namespace': 'True',
            'rviz_config': rviz_config_file, 'log_level': 'warn'
        }.items(),
    )

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_robot_namespace)
    ld.add_action(declare_x_pose)
    ld.add_action(declare_y_pose)
    ld.add_action(declare_map_yaml_cmd)

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)
    ld.add_action(bringup_cmd)

    ld.add_action(declare_params_file_cmd)
    ld.add_action(initial_pose_cmd)
    ld.add_action(rviz_cmd)

    return ld