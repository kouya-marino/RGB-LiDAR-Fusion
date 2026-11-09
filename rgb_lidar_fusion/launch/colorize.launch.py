"""Sensors + live colorizer + web UI (+ optional RViz).

    ros2 launch rgb_lidar_fusion colorize.launch.py
    # then open http://localhost:8080
    ros2 launch rgb_lidar_fusion colorize.launch.py output_dir:=/ws/src/rgb_lidar_fusion/output
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
    output_dir = LaunchConfiguration('output_dir')
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
            'output_dir': output_dir,
        }],
    )
    web_ui = Node(
        package='rgb_lidar_fusion', executable='web_ui', name='web_ui', output='screen',
        condition=IfCondition(LaunchConfiguration('web')),
        parameters=[{'output_dir': output_dir, 'port': LaunchConfiguration('port')}],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='log',
        arguments=['-d', os.path.join(SHARE, 'rviz', 'colorize.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )
    return LaunchDescription([
        DeclareLaunchArgument('sensors', default_value='true',
                              description='false: use sensors already running (or a bag)'),
        DeclareLaunchArgument('accumulate', default_value='false',
                              description='true: start recording a map at launch'),
        DeclareLaunchArgument('output_dir', default_value=os.path.join(os.getcwd(), 'output')),
        DeclareLaunchArgument('color_profile', default_value='1920x1080x15'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='false'),
        sensors,
        colorizer,
        web_ui,
        rviz,
    ])
