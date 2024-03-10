import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution

def generate_robots(num_robots):
    robots = []

    for i in range(num_robots):
        robot_name = "tb_"+str(i)
        x_pos = float(i)
        robots.append({'name': robot_name, 'x_pose': x_pos, 'y_pose': 0.0, 'z_pose': 0.01, 'yaw': 0.0})

    return robots 

def generate_launch_description():
    ld = LaunchDescription()

    launch_file_dir = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    robots = generate_robots(4)

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

        ld.add_action(robot_state_publisher_cmd)
        ld.add_action(spawn_turtlebot_cmd)

    return ld