from setuptools import setup

package_name = 'emergency_fusion'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/emergency_fusion.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'Fuses LiDAR / mic array / IMU events into final fall and SOS alerts. '
        'fall_fusion_node and sos_fusion_node run independently.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'fall_fusion_node = emergency_fusion.fall_fusion_node:main',
            'sos_fusion_node = emergency_fusion.sos_fusion_node:main',
            'viz_merge_node = emergency_fusion.viz_merge_node:main',
        ],
    },
)
