from setuptools import setup

package_name = 'imu_fall_sos'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/imu_fall_sos.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description=(
        'WT901BLECL floor IMU fall-evidence and SOS tap-pattern detector. '
        'Publishes JSON events on /fall/evidence and /sos/detected.'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'imu_fall_sos_node = imu_fall_sos.imu_fall_sos_node:main',
        ],
    },
)
