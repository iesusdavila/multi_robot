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

def generate_robots(num_robots):

    robots = [
        {'name': 'tb1', 'x_pose': '-1.5', 'y_pose': '-0.5', 'z_pose': 0.01, 'yaw': 0.0},
        {'name': 'tb2', 'x_pose': '-1.5', 'y_pose': '0.5', 'z_pose': 0.01, 'yaw': 0.0},
    ]

    return robots 

def generate_launch_description():
    ld = LaunchDescription()

    robots = generate_robots(2)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time = DeclareLaunchArgument(
        name='use_sim_time', default_value=use_sim_time, description='Use simulator time'
    )
    
    package_dir = get_package_share_directory('multi_robot_navigation')
    
    nav_launch_dir = os.path.join(package_dir, 'launch')

    launch_file_dir = os.path.join(get_package_share_directory('multi_robot_gazebo'), 'launch')

    rviz_config_file = LaunchConfiguration('rviz_config_file')
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(
            package_dir, 'rviz', 'multi_nav2_default_view.rviz'),
        description='Full path to the RVIZ config file to use')

    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'turtlebot3_world.world'
    )

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gzclient.launch.py')
        ),
    )

    params_file = LaunchConfiguration('nav_params_file')
    declare_params_file_cmd = DeclareLaunchArgument(
        'nav_params_file',
        default_value=os.path.join(package_dir, 'params', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
    
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
 
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    map_server=Node(package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': os.path.join(get_package_share_directory('turtlebot3_navigation2'), 'map', 'map.yaml')}],
        remappings=remappings)

    map_server_lifecyle=Node(package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': True},
                        {'node_names': ['map_server']}])

    ld.add_action(map_server)
    ld.add_action(map_server_lifecyle)

    last_action = None
    for robot in robots:

        namespace = [ '/' + robot['name'] ]

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

        frames = Node(
            package='tf2_ros',
            executable='tf2_echo',
            output='screen',
        )

        bringup_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'bringup_nav.launch.py')
            ),
            launch_arguments={  
                'slam': 'False',
                'namespace': [ '/' + robot['name'] ],
                'use_namespace': 'True',
                'map': '',
                'map_server': 'False',
                'params_file': params_file,
                'default_bt_xml_filename': os.path.join(
                    get_package_share_directory('nav2_bt_navigator'),
                    'behavior_trees', 'navigate_w_replanning_and_recovery.xml'),
                'autostart': 'true',
                'use_sim_time': use_sim_time, 'log_level': 'warn'
            }.items()
        )

        if last_action is None:
            ld.add_action(robot_state_publisher_cmd)
            ld.add_action(spawn_turtlebot_cmd)
            ld.add_action(bringup_cmd)
            ld.add_action(frames)
        else:
            spawn_turtlebot3_event = RegisterEventHandler(
                event_handler=OnProcessExit(
                    target_action=last_action,
                    on_exit=[spawn_turtlebot_cmd,
                            robot_state_publisher_cmd,
                            bringup_cmd,
                            frames],
                )
            )

            ld.add_action(spawn_turtlebot3_event)

        last_action = frames

    for robot in robots:

        namespace = [ '/' + robot['name'] ]

        message = '{header: {frame_id: map}, pose: {pose: {position: {x: ' + \
            robot['x_pose'] + ', y: ' + robot['y_pose'] + \
            ', z: 0.1}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0000000}}, }}'

        initial_pose_cmd = ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-t', '3', '--qos-reliability', 'reliable', namespace + ['/initialpose'],
                'geometry_msgs/PoseWithCovarianceStamped', message],
            output='screen'
        )

        rviz_cmd = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_launch_dir, 'rviz.launch.py')),
                launch_arguments={'use_sim_time': use_sim_time, 
                                  'namespace': namespace,
                                  'use_namespace': 'True',
                                  'rviz_config': rviz_config_file, 'log_level': 'warn'}.items(),
        )

        post_spawn_event = RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=last_action,
                on_exit=[initial_pose_cmd, rviz_cmd],
            )
        )

        last_action = initial_pose_cmd

        ld.add_action(post_spawn_event)
        ld.add_action(declare_params_file_cmd)

    return ld
