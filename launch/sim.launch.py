import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

def generate_launch_description():

    package_name='auto_bot'

    ros2_control = LaunchConfiguration('ros2_control')
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')

    rsp = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory(package_name),'launch','rsp.launch.py'
        )]), launch_arguments={'use_sim_time': 'true', 'ros2_control': ros2_control}.items()
    )

    gazebo_params = os.path.join(
        get_package_share_directory(package_name),
        'config',
        'gazebo_params.yaml'
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
                launch_arguments={'extra_gazebo_args': f'--ros-args --params-file {gazebo_params}'}.items()
             )

    spawn_entity = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py', 
        arguments=[
            '-topic', 'robot_description', 
            '-entity', 'auto_bot',
        ], 
        output='screen'
    )

   
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        remappings=[('diff_drive_controller/odom', '/odom')],
        # ros2 run topic_tools relay /diff_drive_controller/odom /odom
    )


    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
    )


    delayed_ddc_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[diff_drive_controller_spawner]
        )
    )
    delayed_jsb_spawner = RegisterEventHandler(
         event_handler=OnProcessExit(
             target_action=spawn_entity,
             on_exit=[joint_state_broadcaster_spawner],
         )
    )


    #rtabmap = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource([
    #        os.path.join(
    #            get_package_share_directory('rtabmap_demos'),
    #            'launch',
    #            'turtlebot3',
    #            'turtlebot3_rgbd.launch.py')
    #    ]),
    #    launch_arguments={'use_sim_time': 'true', 'localization': 'true'}.items()
    #)

    # rtabmap = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         os.path.join(
    #             get_package_share_directory('rtabmap_launch'),
    #             'launch',
    #             'rtabmap.launch.py')
    #     ]),
    #     launch_arguments={
    #         'use_sim_time': 'true',
    #         'frame_id': 'base_link',
    #         'map_frame_id': 'map',
    #         # 'odom_frame_id': 'odom',
    #         'rgb_topic': '/camera/image_raw',
    #         'depth_topic': '/camera/depth/image_raw',
    #         'camera_info_topic': '/camera/camera_info',
    #         'odom_topic': '/odom',
    #         'visual_odometry': 'true',
    #         'qos_image': '1', # 2 = best effort for sim
    #         'qos_camera_info': '1', # 2 = best effort for sim
    #         'approx_sync': 'true',
    #         #'topic_queue_size': '20',
    #         #'sync_queue_size': '20',
    #         #'rtabmap_viz': 'true',
    #
    #     }.items()
    # )


    # turtlebot3_rgbd launch 
    parameters={
        'frame_id':'base_footprint',
        'use_sim_time': use_sim_time,
        'subscribe_depth':True,
        'use_action_for_goal':True,
        'Reg/Force3DoF':'true',
        'Grid/RayTracing':'true', # Fill empty space
        'Grid/3D':'false', # Use 2D occupancy
        'Grid/RangeMax':'10.0', # Max obstacle detection range (meters)
        'Grid/NormalsSegmentation':'false', # Use passthrough filter to detect obstacles
        'Grid/MaxGroundHeight':'0.05', # All points above 5 cm are obstacles
        'Grid/MaxObstacleHeight':'0.4',  # All points over 1 meter are ignored
        'Optimizer/GravitySigma':'0' # Disable imu constraints (we are already in 2D)
    }

    remappings=[
        ('rgb/image', '/camera/image_raw'),
        ('rgb/camera_info', '/camera/camera_info'),
        ('depth/image', '/camera/depth/image_raw')]


    # rtabmap nodes

    # SLAM mode:
    rtabmap_slam = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters],
        remappings=remappings,
        arguments=['-d']) # This will delete the previous database (~/.ros/rtabmap.db)

    # Localization mode:
    rtabmap_localization = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        parameters=[parameters,
                    {'Mem/IncrementalMemory':'False',
                     'Mem/InitWMWithAllNodes':'True'}],
        remappings=remappings)


    rtabmap_viz = Node(
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        parameters=[parameters],
        remappings=remappings)

    # Obstacle detection with the camera for nav2 local costmap.
    # First, we need to convert depth image to a point cloud.
    # Second, we segment the floor from the obstacles.
    rtabmap_point_cload = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                     'max_depth': 10.0,
                     'voxel_size': 0.02}],
        remappings=[('depth/image', '/camera/depth/image_raw'),
                    ('depth/camera_info', '/camera/camera_info'),
                    ('cloud', '/camera/cloud')])

    rtabmap_obstacles = Node(
        package='rtabmap_util', executable='obstacles_detection', output='screen',
        parameters=[parameters],
        remappings=[('cloud', '/camera/cloud'),
                    ('obstacles', '/camera/obstacles'),
                    ('ground', '/camera/ground')])

    return LaunchDescription([
        DeclareLaunchArgument('ros2_control', default_value='true', description='Use ros2_control'),
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        DeclareLaunchArgument('localization', default_value='false', description='Launch rtabmap in localization mode.'),

        rsp,
        gazebo,
        spawn_entity,
        #diff_drive_controller_spawner,
        #joint_state_broadcaster_spawner,
        delayed_ddc_spawner,
        delayed_jsb_spawner,
        #rtabmap,
        rtabmap_slam,
        rtabmap_localization,
        rtabmap_viz,
        rtabmap_point_cload,
        rtabmap_obstacles
    ])
