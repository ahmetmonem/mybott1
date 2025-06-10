import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package name
    package_name='articubot_one'
    
    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    # Gazebo parameters
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    # World file path
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','fire_world.sdf')
    
    # Include Gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world_file
                    }.items()
             )
    
    # Spawn the robot
    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'articubot',
                   '-x', '0.0',
                   '-y', '0.0',
                   '-z', '0.1'],
        output='screen'
    )
    
    # Spawn controllers with delay
    diff_drive_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["diff_cont"],
                output='screen'
            )
        ]
    )
    
    joint_broad_spawner = TimerAction(
        period=3.0,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_broad"],
                output='screen'
            )
        ]
    )
    
    # Fire detection node with delay
    fire_detector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='articubot_one',
                executable='detect_fire.py',
                name='fire_detector',
                parameters=[{
                    'use_sim_time': True,
                    'tuning_mode': False,
                    'h_min': 0,
                    'h_max': 20,
                    's_min': 150,
                    's_max': 255,
                    'v_min': 150,
                    'v_max': 255,
                    'x_min': 0,
                    'x_max': 100,
                    'y_min': 0,
                    'y_max': 100,
                    'blur': 3
                }],
                remappings=[
                    ('image_in', '/camera/image_raw')
                ],
                output='screen'
            )
        ]
    )
    
    # Modified fire follower that publishes to correct topic
    fire_follower = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='articubot_one',
                executable='follow_fire.py',
                name='fire_follower',
                parameters=[{
                    'use_sim_time': True,
                    'angular_gain': 1.5,
                    'linear_gain': 0.5,
                    'max_linear_vel': 0.3,
                    'max_angular_vel': 0.8,
                    'min_fire_area': 0.001,
                    'target_fire_area': 0.015,
                    'stop_distance_area': 0.025,
                    'search_angular_vel': 0.4
                }],
                remappings=[
                    # Try remapping to the correct topic
                    ('/diff_cont/cmd_vel_unstamped', '/cmd_vel')
                ],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        fire_detector,
        fire_follower
    ])