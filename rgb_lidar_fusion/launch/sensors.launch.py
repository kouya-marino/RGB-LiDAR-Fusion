"""Start the Livox Mid-360(S) driver, the RealSense colour stream, the static
livox_frame -> camera_link transform of the mount and, by default, FAST-LIO2
lidar-inertial odometry so the rig can be carried while mapping.

    ros2 launch rgb_lidar_fusion sensors.launch.py
    ros2 launch rgb_lidar_fusion sensors.launch.py odometry:=false   # stationary rig
    ros2 launch rgb_lidar_fusion sensors.launch.py camera:=false

With odometry the driver publishes Livox CustomMsg (what FAST-LIO reads) and
the scans to use are FAST-LIO's /cloud_registered; without it the driver
publishes PointCloud2 on /livox/lidar.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, GroupAction, IncludeLaunchDescription,
                            OpaqueFunction)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('rgb_lidar_fusion')
# Mid-360S user manual: IMU at (11.0, 23.29, -44.12) mm in point-cloud
# coordinates, axes aligned -> the lidar sits at minus that in the IMU frame.
LIDAR_IN_IMU = ('-0.011', '-0.02329', '0.04412')


def _static_tf(context):
    path = LaunchConfiguration('extrinsics').perform(context)
    with open(path) as fh:
        e = yaml.safe_load(fh)
    x, y, z = (str(v) for v in e['xyz'])
    qx, qy, qz, qw = (str(v) for v in e['quat_xyzw'])
    return [Node(
        package='tf2_ros', executable='static_transform_publisher',
        name='mount_static_tf', output='log',
        arguments=['--x', x, '--y', y, '--z', z,
                   '--qx', qx, '--qy', qy, '--qz', qz, '--qw', qw,
                   '--frame-id', e['parent_frame'], '--child-frame-id', e['child_frame']],
    )]


def _realsense(context):
    # rs_launch.py turns every launch configuration in scope into a node
    # parameter, so isolate it (forwarding=False) and hand it literal values.
    profile = LaunchConfiguration('color_profile').perform(context)
    return [GroupAction(forwarding=False, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'false',     # lidar supplies all geometry
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'enable_gyro': 'false',
            'enable_accel': 'false',
            'rgb_camera.color_profile': profile,
        }.items())])]


def _livox(context):
    odometry = LaunchConfiguration('odometry').perform(context).lower() in ('true', '1')
    return [Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen',
        parameters=[{
            # 1 = livox CustomMsg (FAST-LIO), 0 = sensor_msgs/PointCloud2
            'xfer_format': 1 if odometry else 0,
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'cmdline_input_bd_code': 'livox0000000001',
            'user_config_path': LaunchConfiguration('livox_config'),
        }],
    )]


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    camera = LaunchConfiguration('camera')
    odometry = LaunchConfiguration('odometry')

    fast_lio = Node(
        package='fast_lio', executable='fastlio_mapping', name='fast_lio', output='log',
        condition=IfCondition(odometry),
        parameters=[LaunchConfiguration('fast_lio_config')],
    )
    # FAST-LIO's `body` is the IMU; connect it to the lidar (and so the camera)
    imu_to_lidar = Node(
        package='tf2_ros', executable='static_transform_publisher', name='imu_to_lidar_tf',
        output='log', condition=IfCondition(odometry),
        arguments=['--x', LIDAR_IN_IMU[0], '--y', LIDAR_IN_IMU[1], '--z', LIDAR_IN_IMU[2],
                   '--frame-id', 'body', '--child-frame-id', 'livox_frame'],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('odometry', default_value='true',
                              description='FAST-LIO2 odometry, for mapping while moving'),
        DeclareLaunchArgument('fast_lio_config', default_value=os.path.join(
            SHARE, 'config', 'fast_lio_mid360s.yaml')),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(
            SHARE, 'config', 'livox', 'MID360s_config.json')),
        DeclareLaunchArgument('extrinsics', default_value=os.path.join(
            SHARE, 'config', 'mount_extrinsics.yaml')),
        DeclareLaunchArgument('color_profile', default_value='1920x1080x15'),
        OpaqueFunction(function=_livox, condition=IfCondition(lidar)),
        fast_lio,
        imu_to_lidar,
        OpaqueFunction(function=_realsense, condition=IfCondition(camera)),
        OpaqueFunction(function=_static_tf),
    ])
