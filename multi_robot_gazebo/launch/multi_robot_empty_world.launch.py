import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import yaml

def generate_robots():
    robots_file = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'config', 'robots_empty_world.yaml')

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
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
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
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'robot_name': robot['name'],
                'robot_namespace': robot['name'],
            }.items()
        )

        spawn_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_robots.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'yaw': TextSubstitution(text=str(robot['yaw'])), 
                'robot_name': robot['name'],
                'robot_namespace': robot['name'],
            }.items()
        )

        navigation_turtlebot_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_multi_robot_navigation, 'launch', 'multi_navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_namespace': ['/' + robot['name']],
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'rviz_config_file': os.path.join(pkg_multi_robot_navigation, 'rviz', 'multi_nav2_default_view.rviz'),
                'nav_params_file': os.path.join(pkg_multi_robot_navigation, 'params', 'nav2_params.yaml'),
                'map_yaml_file': os.path.join(pkg_multi_robot_navigation, 'maps', 'map_empty.yaml'),
            }.items(),
            condition=IfCondition(navigation)
        )

        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)
        ld.add_action(navigation_turtlebot_cmd)

    return ld