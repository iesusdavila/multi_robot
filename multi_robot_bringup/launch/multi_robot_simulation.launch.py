import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.conditions import IfCondition
import yaml
import math

def quaternion_from_euler(yaw):

    qz = math.sin(yaw * 0.5)
    qw = math.cos(yaw * 0.5)  

    return qz, qw

def generate_robots(sim_param_file):
    #robots_file = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'config', 'robots_house_pro.yaml')
    robots_file = sim_param_file

    with open(robots_file, 'r') as file:
        robots_data = yaml.safe_load(file)

    print("path world: ", robots_data['world']['world_path'])
    return robots_data['robots'], robots_data['world']

def select_rsp_launch(launch_file_dir, robot, use_sim_time):
    extension_file = robot['urdf_path'].split(".")[-1]

    if extension_file == "urdf":
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robots_states_publishers_urdf.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_namespace': robot['name'],
                'urdf_path': robot['urdf_path'],
            }.items()
        )
    else:
        robot_state_publisher_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'robots_states_publishers_xacro.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_namespace': robot['name'],
                'xacro_path': robot['urdf_path'],
            }.items()
        )
    
    return robot_state_publisher_cmd

def select_spawn_launch(launch_file_dir, robot, use_sim_time):
    if robot['sdf_path'] == "":
        spawn_entity_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_robots_topic.launch.py')
            ),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'robot_description_topic': '/'+robot['name']+'/robot_description',
                'x_pose': TextSubstitution(text=str(robot['x_pose'])),
                'y_pose': TextSubstitution(text=str(robot['y_pose'])),
                'z_pose': TextSubstitution(text=str(robot['z_pose'])),
                'yaw': TextSubstitution(text=str(robot['yaw'])), 
                'robot_name': robot['name'],
                'robot_namespace': robot['name'],
            }.items()
        )
    else:
        spawn_entity_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(launch_file_dir, 'spawn_robots_sdf.launch.py')
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
    
    return spawn_entity_cmd

def execute_multi_robot(context, *args, **kwargs):
    launch_file_dir = os.path.join(get_package_share_directory('multi_robot_bringup'), 'launch', 'utils')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_multi_robot_navigation = get_package_share_directory('multi_robot_navigation')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    navigation = LaunchConfiguration('navigation', default='false')
    sim_param_file = LaunchConfiguration('sim_param_file', default='').perform(context)

    nodes_exec = []

    robots, world = generate_robots(sim_param_file)

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world['world_path']}.items()
    )
    nodes_exec.append(gzserver_cmd)

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    nodes_exec.append(gzclient_cmd)

    for robot in robots:        
        robot_state_publisher_cmd = select_rsp_launch(launch_file_dir, robot, use_sim_time)
        spawn_entity_cmd = select_spawn_launch(launch_file_dir, robot, use_sim_time)

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
                'map_yaml_file': world['map_path'],
            }.items(),
            condition=IfCondition(navigation)
        )

        nodes_exec.append(robot_state_publisher_cmd)
        nodes_exec.append(spawn_entity_cmd)
        nodes_exec.append(navigation_robot_cmd)

    return nodes_exec

def generate_launch_description():
    ld = LaunchDescription()

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')
    declare_navigation = DeclareLaunchArgument('navigation', default_value='false', description='Launch navigation')
    declare_sim_param_file = DeclareLaunchArgument('sim_param_file', default_value='', description='Simulation parameter file')

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_navigation)
    ld.add_action(declare_sim_param_file)
    ld.add_action(OpaqueFunction(function=execute_multi_robot))

    return ld