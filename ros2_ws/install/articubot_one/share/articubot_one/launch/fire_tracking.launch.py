import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Get the package directory
    pkg_dir = get_package_share_directory('articubot_one')
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    tuning_mode = LaunchConfiguration('tuning_mode', default='false')
    
    # Fire detection node
    fire_detector = Node(
        package='articubot_one',
        executable='detect_fire.py',
        name='fire_detector',
        parameters=[{
            'use_sim_time': use_sim_time,
            'tuning_mode': tuning_mode,
            'h_min': 0,      # Hue range for fire (red-orange)
            'h_max': 25,
            's_min': 100,    # High saturation for fire
            's_max': 255,
            'v_min': 100,    # High value (brightness) for fire
            'v_max': 255,
            'x_min': 10,     # ROI boundaries (percentage)
            'x_max': 90,
            'y_min': 10,
            'y_max': 90,
            'blur': 3
        }],
        remappings=[
            ('image_in', '/camera/image_raw')
        ]
    )
    
    # Fire follower node
    fire_follower = Node(
        package='articubot_one',
        executable='follow_fire.py',
        name='fire_follower',
        parameters=[{
            'use_sim_time': use_sim_time,
            'angular_gain': 2.0,
            'linear_gain': 0.8,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'min_fire_area': 0.001,
            'target_fire_area': 0.02,
            'search_angular_vel': 0.3
        }]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'tuning_mode',
            default_value='false',
            description='Enable tuning mode for fire detection'
        ),
        fire_detector,
        fire_follower
    ])