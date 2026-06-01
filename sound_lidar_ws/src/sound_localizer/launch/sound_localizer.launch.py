from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='sound_localizer',
            executable='sound_localizer_node',
            name='sound_localizer',
            output='screen',
            parameters=[{
                'mic_count':     4,
                'mic_spacing':   0.05,
                'sound_speed':   343.0,
                'threshold_db':  50.0,
                'publish_rate':  10.0,
            }]
        ),
    ])
