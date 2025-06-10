import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name='articubot_one'
    
    # Robot state publisher without ros2_control
    robot_description_path = os.path.join(
        get_package_share_directory(package_name),
        'description',
        'robot.urdf.xacro'
    )
    
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(robot_description_path).read()
        }]
    )
    
    # Gazebo with fire world
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','fire_world.sdf')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file}.items()
    )
    
    # Spawn robot
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
    
    # Fire detection node
    fire_detector = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='articubot_one',
                executable='detect_fire.py',
                name='fire_detector',
                parameters=[{
                    'use_sim_time': True,
                    'tuning_mode': True,  # Enable tuning to see what's happening
                    'h_min': 0,
                    'h_max': 20,
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
    
    # Modified fire follower that publishes to /cmd_vel
    fire_follower = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='articubot_one',
                executable='follow_fire.py',
                name='fire_follower',
                parameters=[{
                    'use_sim_time': True,
                    'angular_gain': 1.0,
                    'linear_gain': 0.3,
                    'max_linear_vel': 0.3,
                    'max_angular_vel': 0.5,
                    'min_fire_area': 0.001,
                    'target_fire_area': 0.01,
                    'stop_distance_area': 0.02,
                    'search_angular_vel': 0.3
                }],
                remappings=[
                    ('/cmd_vel_tracker', '/cmd_vel')
                ],
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