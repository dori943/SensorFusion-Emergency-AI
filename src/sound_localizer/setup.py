from setuptools import setup

package_name = 'sound_localizer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/sound_localizer.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Sound source localization node using mic array',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'sound_localizer_node = sound_localizer.sound_localizer_node:main',
            'sound_source_marker_node = sound_localizer.sound_source_marker_node:main'
        ],
    },
)
