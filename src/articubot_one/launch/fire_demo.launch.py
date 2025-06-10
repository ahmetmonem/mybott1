import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='articubot_one'
    
    # Use the existing launch_sim.launch.py as base
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','launch_sim.launch.py'
        )]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory(package_name),'worlds','fire_world.sdf'
        )}.items()
    )
    
    # Add fire detection after 5 seconds
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
                    'h_max': 25,
                    's_min': 100,
                    's_max': 255,
                    'v_min': 100,
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
    
    # Add fire follower after 5 seconds
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
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        base_sim,
        fire_detector,
        fire_follower
    ])