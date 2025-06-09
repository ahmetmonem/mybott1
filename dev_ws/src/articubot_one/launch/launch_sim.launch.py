import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    # Include the robot_state_publisher launch file, provided by our own package
    package_name = 'articubot_one'
    
    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory(package_name), 'launch', 'rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'use_ros2_control': 'true'}.items()
    )

    # Include the Gazebo launch file, provided by the gazebo_ros package
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': os.path.join(
            get_package_share_directory(package_name), 'worlds', 'fire_world.world')}.items()
    )

    # Run the spawner node from the gazebo_ros package
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'articubot_one'],
        output='screen'
    )

    # Launch the diff_drive_controller
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_cont"],
    )

    # Launch the water pump controller
    water_pump_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["water_pump_cont"],
    )

    # Launch the joint state broadcaster
    joint_broad_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_broad"],
    )

    # Fire detection and extinguishing node
    fire_detector = Node(
        package=package_name,
        executable='fire_detector',
        name='fire_detector',
        output='screen'
    )

    # Water pump controller node
    water_pump_controller = Node(
        package=package_name,
        executable='water_pump_controller',
        name='water_pump_controller',
        output='screen'
    )

    # Delay spawning controllers to ensure robot is loaded
    delayed_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[diff_drive_spawner],
                ),
            ],
        )
    )

    delayed_water_pump_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=3.0,
                    actions=[water_pump_spawner],
                ),
            ],
        )
    )

    delayed_joint_broad_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=2.5,
                    actions=[joint_broad_spawner],
                ),
            ],
        )
    )

    delayed_fire_detector = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=4.0,
                    actions=[fire_detector],
                ),
            ],
        )
    )

    delayed_water_pump_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=spawn_entity,
            on_start=[
                TimerAction(
                    period=4.5,
                    actions=[water_pump_controller],
                ),
            ],
        )
    )

    # Launch them all together
    return LaunchDescription([
        rsp,
        gazebo,
        spawn_entity,
        delayed_diff_drive_spawner,
        delayed_water_pump_spawner,
        delayed_joint_broad_spawner,
        delayed_fire_detector,
        delayed_water_pump_controller,
    ])