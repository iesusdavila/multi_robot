import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import yaml
import math

def quaternion_from_euler(yaw):

    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)  

    return qz, qw

def generate_robots():
    robots_file = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'config', 'robots_warehouse.yaml')

    with open(robots_file, 'r') as file:
        robots_data = yaml.safe_load(file)

    return robots_data['robots']

def generate_launch_description():
    ld = LaunchDescription()

    launch_file_dir = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    navigation = LaunchConfiguration('navigation', default='false')

    world = os.path.join(
        get_package_share_directory('multi_robot_gazebo'),
        'worlds',
        'warehouse.world'
    )

    robots = generate_robots()

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    for robot in robots:
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robots_states_publishers.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_namespace': robot['name'],
                'urdf_path': robot['urdf_path'],
            }.items()
        )

        spawn_entity_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_robots.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'urdf_path': robot['sdf_path'],
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'yaw': TextSubstitution(text=str(robot['yaw'])), 
                'robot_name': robot['name'],
                'robot_namespace': robot['name'],
            }.items()
        )

        navigation_robot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_multi_robot_navigation, 'launch', 'multi_navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_namespace': ['/' + robot['name']],
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'qz': TextSubstitution(text=str(quaternion_from_euler(robot['yaw'])[0])),
                'qw': TextSubstitution(text=str(quaternion_from_euler(robot['yaw'])[1])),
                'rviz_view': robot['rviz_view'],
                'rviz_config_file': os.path.join(pkg_multi_robot_navigation, 'rviz', 'multi_nav2_default_view.rviz'),
                'nav_params_file': robot['nav_param_path'],
                'map_yaml_file': os.path.join(pkg_multi_robot_navigation, 'maps', 'map_world.yaml'),
            }.items(),
            condition=IfCondition(navigation)
        )

        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_entity_cmd)
        ld.add_action(navigation_robot_cmd)

    return ld