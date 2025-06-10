import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Package name
    package_name='articubot_one'
    
    # Launch arguments
    tuning_mode = LaunchConfiguration('tuning_mode', default='false')
    
    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    # Gazebo parameters
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    # World file path - use fire world
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','fire_world.sdf')
    
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world_file
                    }.items()
             )
    
    # Spawn the robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'articubot',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.1'],
                        output='screen')
    
    # Spawn controllers with proper configuration
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont", "--controller-manager", "/controller_manager"],
        output='screen'
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad", "--controller-manager", "/controller_manager"],
        output='screen'
    )
    
    # Fire detection node
    fire_detector = Node(
        package='articubot_one',
        executable='detect_fire.py',
        name='fire_detector',
        parameters=[{
            'use_sim_time': True,
            'tuning_mode': tuning_mode,
            'h_min': 0,      # Hue range for fire (red-orange)
            'h_max': 20,     # Adjusted for better fire detection
            's_min': 150,    # Higher saturation for vivid fire
            's_max': 255,
            'v_min': 150,    # Higher value for bright fire
            'v_max': 255,
            'x_min': 0,      # Full image scan
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
    
    # Fire follower node (autonomous)
    fire_follower = Node(
        package='articubot_one',
        executable='follow_fire.py',
        name='fire_follower',
        parameters=[{
            'use_sim_time': True,
            'angular_gain': 1.5,
            'linear_gain': 0.5,
            'max_linear_vel': 0.4,
            'max_angular_vel': 1.0,
            'min_fire_area': 0.001,
            'target_fire_area': 0.015,
            'stop_distance_area': 0.025,
            'search_angular_vel': 0.4
        }],
        output='screen'
    )
    
    # Image visualization (optional)
    image_view = Node(
        package='image_view',
        executable='image_view',
        name='image_view',
        remappings=[('image', '/image_tuning')],
        condition=IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('launch'), 
                           'launch', 'if.launch.py')
            ]),
            launch_arguments={
                'arg': 'show_image',
                'default': 'false'
            }.items()
        )
    )
    
    # Launch description
    return LaunchDescription([
        DeclareLaunchArgument('tuning_mode', default_value='false',
                            description='Enable tuning mode for fire detection'),
        DeclareLaunchArgument('show_image', default_value='false',
                            description='Show image view window'),
        rsp,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        fire_detector,
        fire_follower
    ])