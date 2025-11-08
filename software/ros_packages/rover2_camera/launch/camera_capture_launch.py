import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Replace these with your camera serial numbers
    # You can find them with `rs-enumerate-devices`
    chassis_serial = "_318122302525"


    realsense_launch_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'serial_no': chassis_serial,
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x360x30',
            'enable_v4l2': 'false'
        }.items()
    )

    # Your rover2_camera nodes
    ir_camera_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='ir',
        parameters=[{
            'device': '/dev/rover/camera_infrared',
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42067
        }],
        respawn=True
    )

    main_nav_node = Node(
        package='rover2_camera',
        namespace='rover2_camera',
        executable='camera_capture',
        name='main_navigation',
        parameters=[{
            'device': '/dev/rover/camera_main_navigation',
            'cap_width': 1920,
            'cap_height': 1080,
            'cap_framerate': 30,
            'preset_level': 1,
            'bitrate': 4000000,
            'stream_width': 640,
            'stream_height': 480,
            'fec_percentage': 30,
            'udp_host': '192.168.1.1',
            'udp_port': 42068
        }],
        respawn=True
    )

    return LaunchDescription([
        realsense_launch_nav,
#        ir_camera_node,
#        main_nav_node
    ])

