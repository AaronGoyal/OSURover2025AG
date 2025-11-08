import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration, Command

def generate_launch_description():

    package_name='auto_bot'
    ros2_control = LaunchConfiguration('ros2_control')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'false', 'ros2_control': ros2_control}.items()
    )

    
    robot_description = Command(['ros2 param get --hide-type /robot_state_publisher robot_description'])

    controller_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'controllers.yaml'
    )

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[{'robot_description': robot_description}, controller_params],
    )

    delayed_controller_manager = TimerAction(period=3.0, actions=[controller_manager])

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )

    delayed_ddc_spawner = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[diff_drive_controller_spawner]
        )
    )

    delayed_jsb_spawner = RegisterEventHandler(
         event_handler=OnProcessStart(
             target_action=controller_manager,
             on_start=[joint_state_broadcaster_spawner],
         )
    )


    return LaunchDescription([
        DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),

        rsp,
        #controller_manager,
        #diff_drive_controller_spawner,
        #joint_state_broadcaster_spawner,
        delayed_controller_manager,
        delayed_ddc_spawner,
        delayed_jsb_spawner,
    ])
