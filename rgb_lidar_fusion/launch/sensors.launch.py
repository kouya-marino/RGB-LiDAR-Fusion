"""Start the Livox Mid-360(S) driver, the RealSense colour stream, and the
static livox_frame -> camera_link transform of the mount.

    ros2 launch rgb_lidar_fusion sensors.launch.py
    ros2 launch rgb_lidar_fusion sensors.launch.py camera:=false
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


def generate_launch_description():
    lidar = LaunchConfiguration('lidar')
    camera = LaunchConfiguration('camera')

    livox = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', output='screen', condition=IfCondition(lidar),
        parameters=[{
            'xfer_format': 0,            # 0 = sensor_msgs/PointCloud2
            'multi_topic': 0,
            'data_src': 0,
            'publish_freq': 10.0,
            'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': '',
            'cmdline_input_bd_code': 'livox0000000001',
            'user_config_path': LaunchConfiguration('livox_config'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('lidar', default_value='true'),
        DeclareLaunchArgument('camera', default_value='true'),
        DeclareLaunchArgument('livox_config', default_value=os.path.join(
            SHARE, 'config', 'livox', 'MID360s_config.json')),
        DeclareLaunchArgument('extrinsics', default_value=os.path.join(
            SHARE, 'config', 'mount_extrinsics.yaml')),
        DeclareLaunchArgument('color_profile', default_value='1280x720x15'),
        livox,
        OpaqueFunction(function=_realsense, condition=IfCondition(camera)),
        OpaqueFunction(function=_static_tf),
    ])
