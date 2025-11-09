import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node, SetParameter
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
import os

# To launch for testing with camera plugged into laptop
    # ros2 launch nav_autonomy mapping_launch.py test:=true viz:=true


def generate_launch_description():
    parameters=[{
        # rtab params
        'frame_id':'d455_link',
        'subscribe_depth':True,
        'subscribe_rgb':True,
        'subscribe_odom_info':True,
        'approx_sync':False,              # Set to false because images are all from single camera, so will be synced

        # Internal parameters (must be strings)
        'Grid/3D':"false",                 # We do not want octomap. Saves memory and time
        }]        

    remappings=[
        ('rgb/image', '/camera/d455/color/image_raw'),
        ('rgb/camera_info', '/camera/d455/color/camera_info'),
        ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw')]
        # KRJ TODO: Remap the odom name if necessary
        # KRJ TODO: figure out how to tell rtab the offset and orientation of the camera
    
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument('viz',        default_value='false',  description='Launch RTAB-Map UI and RVIZ.'),
        DeclareLaunchArgument('test',       default_value='false', description='Launch camera and VO for testing'),
        DeclareLaunchArgument('rviz_cfg',   default_value=config_rviz,  description='Configuration path of rviz2.'),

        # Make sure IR emitter is enabled
        SetParameter(name='depth_module.emitter_enabled', value=1),

        # Launch camera driver for testing
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
            output='screen',
            condition=IfCondition(LaunchConfiguration("test"))),

        # Launch visual odom for testing
        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            condition=IfCondition(LaunchConfiguration("test")),
            parameters=parameters,
            remappings=remappings),

 
        # Core SLAM node
        Node(
            package='rtabmap_slam', executable='rtabmap', output='screen',
            parameters=parameters,
            remappings=remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)

        # Visualization:
        Node(
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            condition=IfCondition(LaunchConfiguration("viz")),
            parameters=parameters,
            remappings=remappings),
            
        Node(
            package='rviz2', executable='rviz2', name="rviz2", output='screen',
            condition=IfCondition(LaunchConfiguration("viz")),
            arguments=[["-d"], [LaunchConfiguration("rviz_cfg")]]),
            # RViz Subscribed Topics
                #   /map
                #   /map_updates
                #   /mapData
                #   /mapGraph
                #   /jn0/base_scan
                #   /odom_last_frame
                #   /odom_local_map
                #   /initialpose
                #   /goal_pose
                #   /clicked_point
    ])