from setuptools import setup

package_name = 'lidar_focus'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sound_lidar.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='LiDAR point cloud filtering based on sound direction',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'lidar_focus_node = lidar_focus.lidar_focus_node:main',
            'sound_motion_roi_node = lidar_focus.sound_motion_roi_node:main',
            'ground_removal_node = lidar_focus.ground_removal_node:main',
            'human_bbox_node = lidar_focus.human_bbox_node:main',
            'fall_detection_node = lidar_focus.fall_detection_node:main',
            'incident_recorder_node = lidar_focus.incident_recorder_node:main',
        ],
    },
)
