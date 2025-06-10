import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'articubot_one'
    
    # Base simulation launch
    base_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name),'launch','launch_sim.launch.py'
        )]),
        launch_arguments={
            'world': os.path.join(
                get_package_share_directory(package_name),
                'worlds',
                'complex_fire_world.sdf'
            )
        }.items()
    )
    
    # Fast fire detector
    fire_detector = Node(
        package='articubot_one',
        executable='detect_fire.py',
        name='fire_detector',
        parameters=[{
            'use_sim_time': True,
            'tuning_mode': True,
            'h_min': 0,
            'h_max': 10,
            's_min': 200,
            's_max': 255,
            'v_min': 100,
            'v_max': 255,
            'x_min': 0,
            'x_max': 100,
            'y_min': 0,
            'y_max': 100,
            'blur': 3
        }],
        remappings=[('image_in', '/camera/image_raw')],
        output='screen'
    )
    
    # Fast fire follower with exploration
    fire_follower = Node(
        package='articubot_one',
        executable='follow_fire_exploration.py',  # Using exploration version
        name='fire_follower',
        output='screen'
    )
    
    return LaunchDescription([
        base_sim,
        fire_detector,
        fire_follower
    ])