from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = {
        'emulate_tty': True,
        'output': 'screen',
        'respawn': True
    }

    return LaunchDescription([
        Node(
            package='rover2_odometry',
            executable='imu',
            name='imu',
            **config
        ),
        Node(
            package='rover2_odometry',
            executable='simple_position',
            name='simple_position',
            **config
        ),
        Node(
            package='rover2_odometry',
            executable='scimech_sensors',
            name='scimech_sensors',
            **config
        ),
        Node(
            package='rover2_odometry',
            executable='odometry',
            name='odometry',
            **config
        ),
        Node(
            package='rover2_odometry',
            executable='odrive_can_info',
            name='odrive_can_info',
            **config
        ),

        # Launch visual odom
        # Node(
        #     package='rtabmap_odom', 
        #     executable='rgbd_odometry', 
        #     output='screen',
        #     parameters=[{
        #         # Frames
        #         'frame_id':'rover_base_origin',
        #         'odom_frame_id': "odom",

        #         # config params
        #         'publish_tf':False, 
        #         'approx_sync':False,

        #         # Internal Params (must be strings)
        #         'Odom/Strategy':'1',        # 0 = Frame to map , 1 = Frame to Frame
        #     }],
        #     remappings=[
        #         ('rgb/image', '/camera/d455/color/image_raw'),
        #         ('rgb/camera_info', '/camera/d455/color/camera_info'),
        #         ('depth/image', '/camera/d455/aligned_depth_to_color/image_raw'),
        #         ('odom', '/odometry/visual'),
        #     ]),

        # Global EKF (fuses local EKF and GPS - also publishes odom->base_link)
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[{
                'frequency': 50.0,
                'sensor_timeout': 0.1,
                'two_d_mode': True,
                'transform_time_offset': 0.0,
                'transform_timeout': 0.0,
                'print_diagnostics': True,
                'debug': False,
                'publish_tf': True,
                
                'map_frame': 'map',
                'odom_frame': 'odom',
                'base_link_frame': 'rover_base_origin',
                'world_frame': 'odom', 
                
                # Local odometry 
                'odom0': '/wheel_odom',
                'odom0_config': [True, True, False,
                                False, False, True,
                                True,  False,  False,
                                False, False, False,
                                False, False, False],
                'odom0_queue_size': 10,
                'odom0_differential': False,
                'odom0_relative': False,

                # Visual odometry
                # 'odom1': '/odometry/visual',
                # 'odom1_config': [True,  True,  False,   # x, y position
                #                 False, False, True,     # yaw orientation
                #                 False, False, False,
                #                 False, False, False,
                #                 False, False, False],
                # 'odom1_queue_size': 10,
                # 'odom1_differential': False,
                # 'odom1_relative': False,
                
                # GPS odometry (from navsat_transform)
#                'odom2': '/odometry/gps',
#                'odom2_config': [True,  True,  False,  # Use GPS x, y position
#                                False, False, False,
#                                False, False, False,
#                                False, False, False,
#                                False, False, False],
#                'odom2_queue_size': 10,
#                'odom2_differential': False,
#                'odom2_relative': False,
                
                # IMU (same as local)
                'imu0': '/imu/data',
                'imu0_config': [False, False, False,
                               False, False, True,
                               False, False, False,
                               False, False, True,
                               True,  False,  False],
                'imu0_queue_size': 10,
                'imu0_differential': False,
                'imu0_relative': False,
                'imu0_remove_gravitational_acceleration': True,
            }],
            remappings=[
                ('/odometry/filtered', '/odom'),
            ]
        ),
        
        # 5. NavSat Transform - converts GPS to map frame
        Node(
    package='robot_localization',
    executable='navsat_transform_node',
    name='navsat_transform',
    output='screen',
    parameters=[{
        # Frequency and timing
        'frequency': 30.0,
        'delay': 3.0,
        
        # Magnetic declination at your location (radians)
        # Find yours: https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml
        'magnetic_declination_radians': 0.0,
        'yaw_offset': 0.0,
        
        # 2D navigation
        'zero_altitude': True,
        
        # Publishing options
        'publish_filtered_gps': True,
        'broadcast_utm_transform': False,
        'broadcast_utm_transform_as_parent_frame': False,
        
        # Use odometry heading instead of IMU
        'use_odometry_yaw': True,
        
        # Let first GPS message set origin
        'wait_for_datum': False,
        
        # Manual datum (only used if wait_for_datum is true)
        'use_manual_datum': False,
        'datum': [40.0, -105.0, 0.0],  # [lat, lon, alt] - adjust if needed
        
        # Frame IDs
        'map_frame_id': 'map',
        'odom_frame_id': 'odom',
        'base_link_frame_id': 'rover_base_origin',
        'world_frame_id': 'odom',  # Match the EKFs
    }],
    remappings=[
        ('/gps/fix', '/gps/fix'),              # YOUR GPS INPUT TOPIC
        ('/imu/data', '/imu/data'),            # Your IMU topic
        ('/odometry/filtered', '/odometry/global'),  # Which EKF to use for heading
        ('/odometry/gps', '/odometry/gps'),    # GPS output topic
    ]
),
    ])
