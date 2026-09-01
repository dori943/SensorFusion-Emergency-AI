from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='imu_fall_sos',
            executable='imu_fall_sos_node',
            name='imu_fall_sos',
            output='screen',
            parameters=[{
                # ── 장치 / 토픽 ──────────────────────────────
                'device_address': 'DD:D6:0F:01:23:A5',
                'notify_characteristic_uuid': '',
                'sensor_id': 'floor_imu_01',
                'fall_topic': '/fall/evidence',
                'sos_topic': '/sos/detected',

                # ── 로깅 ────────────────────────────────────
                'save_csv': True,
                'log_directory': '.',

                # ── Fall 피크 검출 (높은 동적 임계값) ─────────
                'fall_threshold_k': 6.0,
                'fall_min_score_threshold': 25.0,
                'fall_max_score_threshold': 80.0,
                'fall_min_signal_g': 0.004,
                'fall_min_peak_distance_sec': 0.40,

                # ── Fall confidence / severity ──────────────
                'fall_medium_ratio': 1.50,
                'fall_strong_ratio': 2.50,
                'fall_confidence_full_ratio': 3.00,

                # ── SOS 피크 검출 (낮은 동적 임계값) ──────────
                'sos_threshold_k': 5.0,
                'sos_min_score_threshold': 15.0,
                'sos_max_score_threshold': 60.0,
                'sos_min_signal_g': 0.002,
                'sos_min_peak_distance_sec': 0.25,

                # ── SOS 시퀀스(반복 두드리기 패턴) 판정 ───────
                'min_sos_hits': 4,
                'sos_window_sec': 3.0,
                'min_ioi_sec': 0.25,
                'max_ioi_sec': 1.00,
                'min_sequence_duration_sec': 0.70,
                'max_sequence_duration_sec': 3.00,
                'min_hit_rate_hz': 1.00,
                'min_median_peak_ratio': 1.05,
                'min_sos_ioi_cv': 0.20,
                'max_sos_autocorr_score': 0.60,

                # ── 발걸음(regular repetition) 오탐 배제 ──────
                'footstep_ioi_cv_max': 0.15,
                'footstep_autocorr_min': 0.60,

                # ── 시퀀스 종료 / 재알림 방지 ─────────────────
                'sequence_quiet_reset_sec': 1.20,
                'sos_cooldown_sec': 5.0,
            }]
        ),
    ])
