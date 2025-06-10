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
    use_fire_world = LaunchConfiguration('use_fire_world', default='true')
    tuning_mode = LaunchConfiguration('tuning_mode', default='false')
    
    # Include the robot_state_publisher launch file
    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
                )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )
    
    # Include joystick control
    joystick = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','joystick.launch.py'
                )]), launch_arguments={'use_sim_time': 'true'}.items()
    )
    
    # Twist mux for command velocity multiplexing
    twist_mux_params = os.path.join(get_package_share_directory(package_name),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )
    
    # Gazebo parameters
    gazebo_params_file = os.path.join(get_package_share_directory(package_name),'config','gazebo_params.yaml')
    
    # World file path - use fire world if requested
    world_file = os.path.join(get_package_share_directory(package_name),'worlds','fire_world.sdf')
    
    # Include the Gazebo launch file
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                    launch_arguments={
                        'extra_gazebo_args': '--ros-args --params-file ' + gazebo_params_file,
                        'world': world_file if use_fire_world else ''
                    }.items()
             )
    
    # Spawn the robot
    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'articubot'],
                        output='screen')
    
    # Spawn controllers
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )
    
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
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
        ],
        output='screen'
    )
    
    # Fire follower node
    fire_follower = Node(
        package='articubot_one',
        executable='follow_fire.py',
        name='fire_follower',
        parameters=[{
            'use_sim_time': True,
            'angular_gain': 2.0,
            'linear_gain': 0.8,
            'max_linear_vel': 0.5,
            'max_angular_vel': 1.0,
            'min_fire_area': 0.001,
            'target_fire_area': 0.02,
            'search_angular_vel': 0.3
        }],
        output='screen'
    )
    
    # Launch them all!
    return LaunchDescription([
        DeclareLaunchArgument('use_fire_world', default_value='true', 
                            description='Use the fire world instead of empty world'),
        DeclareLaunchArgument('tuning_mode', default_value='false',
                            description='Enable tuning mode for fire detection'),
        rsp,
        joystick,
        twist_mux,
        gazebo,
        spawn_entity,
        diff_drive_spawner,
        joint_broad_spawner,
        fire_detector,
        fire_follower
    ])