from glob import glob

from setuptools import setup

package_name = 'rgb_lidar_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/livox', glob('config/livox/*.json')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/web', glob('web/*')),
        # Three.js, vendored in the repo's third_party/ (MIT, license kept)
        ('share/' + package_name + '/vendor/three', ['../third_party/three/LICENSE']),
        ('share/' + package_name + '/vendor/three/build',
         ['../third_party/three/build/three.module.min.js']),
        ('share/' + package_name + '/vendor/three/examples/jsm/controls',
         ['../third_party/three/examples/jsm/controls/OrbitControls.js']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Prashant Rawat',
    maintainer_email='prashantrawatmailbox@gmail.com',
    description=('Real-time RGB camera and Livox Mid-360 LiDAR fusion '
                 'for colorized 3D point clouds.'),
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'colorizer = rgb_lidar_fusion.colorizer_node:main',
            'web_ui = rgb_lidar_fusion.web_ui_node:main',
            'calibrate = rgb_lidar_fusion.calibrate_node:main',
        ],
    },
)
