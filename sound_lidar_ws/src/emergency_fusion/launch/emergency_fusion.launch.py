from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='emergency_fusion',
            executable='fall_fusion_node',
            name='fall_fusion',
            output='screen',
            parameters=[{
                'lidar_topic': '/fall_events',
                'mic_topic': '/sound_source_track',
                'imu_topic': '/fall/evidence',
                'output_topic': '/emergency/fall_alert',

                'eval_rate_hz': 10.0,
                'imu_match_window_sec': 1.5,
                'mic_match_window_sec': 1.5,

                'min_lidar_state': 2,          # CONFIRMED
                'require_imu_evidence': False,  # 엄격 AND 원하면 True로
                'require_mic_match': False,

                'weight_lidar': 0.55,
                'weight_imu': 0.30,
                'weight_mic': 0.15,

                'alert_confidence_threshold': 0.60,
                'alert_cooldown_sec': 8.0,
            }]
        ),
        Node(
            package='emergency_fusion',
            executable='sos_fusion_node',
            name='sos_fusion',
            output='screen',
            parameters=[{
                'imu_topic': '/sos/detected',
                'mic_topic': '/sos/voice_detected',
                'output_topic': '/emergency/sos_alert',

                'mic_match_window_sec': 4.0,
                'require_mic_match': False,
                'mic_confidence_boost': 0.15,
                'alert_cooldown_sec': 5.0,
            }]
        ),
    ])
