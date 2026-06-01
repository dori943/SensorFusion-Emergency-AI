from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        # 마이크 배열 소리 방향 추정 노드
        Node(
            package='sound_localizer',
            executable='sound_localizer_node',
            name='sound_localizer',
            output='screen',
            parameters=[{
                'mic_count':      4,
                'mic_spacing':    0.05,
                'sound_speed':    343.0,
                'threshold_db':   50.0,
                'publish_rate':   10.0,
            }]
        ),

        # LiDAR 포커스 노드
        Node(
            package='lidar_focus',
            executable='lidar_focus_node',
            name='lidar_focus',
            output='screen',
            parameters=[{
                'sector_width':   45.0,
                'timeout_sec':     3.0,
                'lidar_topic':    '/unilidar/cloud',
                'min_confidence':  0.4,
            }]
        ),

        # Stage 1: sound-gated movement ROI
        Node(
            package='lidar_focus',
            executable='sound_motion_roi_node',
            name='sound_motion_roi',
            output='screen',
            parameters=[{
                'lidar_topic': '/unilidar/cloud',
                'sound_topic': '/sound_events',
                'sector_width_deg': 45.0,
                'sound_timeout_sec': 1.5,
                'min_confidence': 0.4,
                'background_frames': 80,
                'background_voxel_size': 0.12,
                'roi_voxel_size': 0.06,
                # Stage 2
                'high_res_voxel_size': 0.05,
                'cluster_eps': 0.25,
                'cluster_min_points': 8,
            }]
        ),
    ])
