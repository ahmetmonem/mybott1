import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command

def generate_launch_description():
    package_name = 'articubot_one'
    pkg_path = os.path.join(get_package_share_directory(package_name))
    
    # Get URDF
    xacro_file = os.path.join(pkg_path, 'description', 'robot.urdf.xacro')
    robot_description_config = Command(['xacro ', xacro_file])
    
    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_config,
            'use_sim_time': True
        }]
    )
    
    # Gazebo
    gazebo = ExecuteProcess(
        cmd=['gazebo', '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    
    # Spawn robot
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', '-entity', 'articubot'],
        output='screen'
    )
    
    # Fire detector (with delay)
    fire_detector = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='articubot_one',
                executable='detect_fire.py',
                name='fire_detector',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'tuning_mode': False,
                    'h_min': 0,
                    'h_max': 30,     # Increased range for red/orange
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
                remappings=[('image_in', '/camera/image_raw')]
            )
        ]
    )
    
    # Fire follower (with delay)
    fire_follower = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='articubot_one',
                executable='follow_fire.py',
                name='fire_follower',
                output='screen',
                parameters=[{
                    'use_sim_time': True,
                    'angular_gain': 1.0,
                    'linear_gain': 0.3,
                    'max_linear_vel': 0.3,
                    'max_angular_vel': 0.5,
                    'min_fire_area': 0.0005,
                    'target_fire_area': 0.01,
                    'stop_distance_area': 0.02,
                    'search_angular_vel': 0.3
                }]
            )
        ]
    )
    
    # Manual fire spawn (red box as simple fire)
    spawn_fire = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
                     '-entity', 'fire_box',
                     '-x', '2.0', '-y', '0.0', '-z', '0.5',
                     '-file', os.path.join(pkg_path, 'models', 'fire_box.sdf')],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        robot_state_publisher,
        gazebo,
        spawn_entity,
        fire_detector,
        fire_follower
    ])