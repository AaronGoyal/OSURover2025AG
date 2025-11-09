from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True,
        'respawn_delay': 2
    }

    return LaunchDescription([
#        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='navigation',
            parameters=[{
                'device_path': '/dev/video12',
                'base_topic': 'cameras/main_navigation',
                'fps': 10
            }],
            #prefix=["sudo taskset -c 3"],
            **config
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='chassis',
            #prefix=["sudo taskset -c 4"],
            parameters=[{
                'device_path': '/dev/video11',
                'base_topic': 'cameras/chassis'
            }],
            **config
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='infrared',
            #prefix=["sudo taskset -c 5"],
            parameters=[{
                'device_path': '/dev/video10',
                'base_topic': 'cameras/infrared'
            }],
            **config
        ),
        Node(
            package='rover_camera',
            executable='rover_camera',
            name='gripper',
            #prefix=["sudo taskset -c 6"],
            parameters=[{
                'is_rtsp_camera': True,
                'device_path': '/dev/video13',
                'base_topic': 'cameras/gripper'
            }],
            **config
        ),
        Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='d455',
        parameters=[{
            "camera_name": "d455",
            "depth_width": 1280,
            "depth_height": 720,
            "color_width": 1280,
            "color_height": 720,
            "pointcloud.enable": True,
            "align_depth.enable": True,
            "serial_no":"318122302525",
            "depth_fps": 10,
            "rgb_fps": 10,
        }],
        output='screen'
        ),
    ])
