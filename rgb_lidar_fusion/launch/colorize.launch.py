"""Sensors + odometry + live colorizer + web UI (+ optional RViz).

    ros2 launch rgb_lidar_fusion colorize.launch.py
    # then open http://localhost:8080; maps are saved in the repo's output/ (on the host)
    ros2 launch rgb_lidar_fusion colorize.launch.py odometry:=false   # stationary rig
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE = get_package_share_directory('rgb_lidar_fusion')
# Save maps into the repo's output/ when it is mounted (docker/run.sh): that is
# a folder on the host, so maps survive the container being stopped/removed.
REPO = '/ws/src/rgb_lidar_fusion'
DEFAULT_OUTPUT = os.path.join(REPO, 'output') if os.path.isdir(REPO) \
    else os.path.join(os.getcwd(), 'output')


def generate_launch_description():
    output_dir = LaunchConfiguration('output_dir')
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(SHARE, 'launch', 'sensors.launch.py')),
        condition=IfCondition(LaunchConfiguration('sensors')),
        launch_arguments={'color_profile': LaunchConfiguration('color_profile'),
                          'odometry': LaunchConfiguration('odometry')}.items(),
    )

    def colorizer(context):
        odometry = LaunchConfiguration('odometry').perform(context).lower() in ('true', '1')
        return [Node(
            package='rgb_lidar_fusion', executable='colorizer', name='colorizer',
            output='screen',
            parameters=[os.path.join(SHARE, 'config', 'colorizer.yaml'), {
                'accumulate': LaunchConfiguration('accumulate'),
                'output_dir': output_dir,
                # FAST-LIO's registered scans (world frame, stamped at the sweep
                # END) or raw scans (lidar frame, stamped at the sweep START)
                'cloud_topic': '/cloud_registered' if odometry else '/livox/lidar',
                'scan_mid_offset': -0.05 if odometry else 0.05,
            }],
        )]

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
        DeclareLaunchArgument('odometry', default_value='true',
                              description='FAST-LIO2: map while carrying the rig'),
        DeclareLaunchArgument('accumulate', default_value='false',
                              description='true: start recording a map at launch'),
        DeclareLaunchArgument('output_dir', default_value=DEFAULT_OUTPUT),
        DeclareLaunchArgument('color_profile', default_value='1920x1080x15'),
        DeclareLaunchArgument('web', default_value='true'),
        DeclareLaunchArgument('port', default_value='8080'),
        DeclareLaunchArgument('rviz', default_value='false'),
        sensors,
        OpaqueFunction(function=colorizer),
        web_ui,
        rviz,
    ])
