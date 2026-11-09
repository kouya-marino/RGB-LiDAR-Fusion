"""Sensors + live colorizer (+ RViz).

    ros2 launch rgb_lidar_fusion colorize.launch.py
    ros2 launch rgb_lidar_fusion colorize.launch.py accumulate:=true \
        output_dir:=/ws/src/rgb_lidar_fusion/output
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('rgb_lidar_fusion')


def generate_launch_description():
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'sensors.launch.py')),
        condition=IfCondition(LaunchConfiguration('sensors')),
        launch_arguments={'color_profile': LaunchConfiguration('color_profile')}.items(),
    )
    colorizer = Node(
        package='rgb_lidar_fusion', executable='colorizer', name='colorizer',
        output='screen',
        parameters=[os.path.join(SHARE, 'config', 'colorizer.yaml'), {
            'accumulate': LaunchConfiguration('accumulate'),
            'output_dir': LaunchConfiguration('output_dir'),
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(SHARE, 'rviz', 'colorize.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true',
                              description='false: use sensors already running (or a bag)'),
        DeclareLaunchArgument('accumulate', default_value='false'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'output')),
        DeclareLaunchArgument('color_profile', default_value='1920x1080x15'),
        DeclareLaunchArgument('rviz', default_value='true'),
        sensors,
        colorizer,
        rviz,
    ])
