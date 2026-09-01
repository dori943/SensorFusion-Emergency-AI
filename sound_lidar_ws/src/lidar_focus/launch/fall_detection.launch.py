#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
fall_detection.launch.py
────────────────────────────────────────────────────────────────────
기존 7개 터미널 실행을 한 번에 대체하는 launch 파일.

  [1] unitree_lidar_ros2/launch.py   LiDAR 드라이버 (/unilidar/cloud)
  [2] lidar_focus/ground_removal_node  전처리+지면제거+배경차감 → /filtered_cloud
  [3] lidar_focus/human_bbox_node      /filtered_cloud → /human_tracks
  [5] lidar_focus/fall_detection_node  /human_tracks  → /fall_events, /fall_marker
  [6] sound_localizer/sound_localizer_node        마이크어레이 DOA
  [7] sound_localizer/sound_source_marker_node    DOA 시각화 마커
  [4] rviz2                            시각화

■ 왜 launch로 같이 띄우는가 (이번 문제의 핵심)
  human_bbox_node는 부팅 시 첫 프레임들로 클러스터링 파라미터를 학습하는데,
  그 입력(/filtered_cloud)이 원본 그대로 나오는 구간은 bg_subtraction의
  '배경 수집 중(처음 100프레임 ≈ 8초)'뿐이다. 노드를 수 분 간격으로 따로
  켜면 human_bbox가 이 창을 놓쳐 프레임을 못 배운다(=지난번 증상).
  → 라이다 처리 3노드는 반드시 함께 떠야 한다.

■ 실행 (두 워크스페이스를 모두 source 해야 함)
  source /opt/ros/jazzy/setup.bash
  source ~/ros2_ws/install/setup.bash          # unitree_lidar_ros2
  source ~/sound_lidar_ws/install/setup.bash   # lidar_focus, sound_localizer
  ros2 launch lidar_focus fall_detection.launch.py

  # 옵션(기본값은 모두 true):
  ros2 launch lidar_focus fall_detection.launch.py lidar_driver:=false   # 드라이버 따로 돌릴 때
  ros2 launch lidar_focus fall_detection.launch.py rviz:=false           # rviz 없이
  ros2 launch lidar_focus fall_detection.launch.py sound:=false          # DOA 없이 라이다만

■ "permission denied"가 났던 경우 (기존 주석)
  chmod +x ~/sound_lidar_ws/install/lidar_focus/bin/ground_removal_node
  (entry-point 실행 비트가 빠졌을 때. launch도 같은 설치 실행파일을 쓰므로 동일하게 필요.)

■ 설치 등록
  이 파일을 ~/sound_lidar_ws/src/lidar_focus/launch/ 에 두고, setup.py의
  data_files에 launch 폴더 설치를 추가한 뒤 colcon build:
    (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py'))
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    lidar_driver = LaunchConfiguration('lidar_driver')
    rviz = LaunchConfiguration('rviz')
    sound = LaunchConfiguration('sound')

    args = [
        DeclareLaunchArgument('lidar_driver', default_value='true',
                              description='unitree_lidar_ros2 드라이버도 함께 실행'),
        DeclareLaunchArgument('rviz', default_value='true',
                              description='rviz2 실행'),
        DeclareLaunchArgument('sound', default_value='true',
                              description='sound_localizer DOA 노드 실행'),
    ]

    # [1] LiDAR 드라이버 (unitree_lidar_ros2/launch/launch.py 포함)
    lidar_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('unitree_lidar_ros2'), 'launch', 'launch.py'])),
        condition=IfCondition(lidar_driver),
    )

    # [2] 전처리+지면제거+배경차감 (한 실행파일 안에 3개 노드).
    #     ※ name= 을 주지 말 것: 이 실행파일은 lidar_preprocessor/ground_removal/
    #        bg_subtraction 3개 노드를 띄우므로 name 리매핑을 걸면 셋이 이름 충돌한다.
    #     ※ 파라미터를 덮어쓰려면 노드별 키를 가진 YAML을 parameters=['config.yaml']로
    #        넘겨야 한다(3-in-1 실행파일이라 단일 dict는 애매하게 적용됨).
    #        지금은 부팅 자동 캘리브레이션이 값을 잡으므로 아무것도 안 넘겨도 된다.
    ground_removal_node = Node(
        package='lidar_focus',
        executable='ground_removal_node',
        output='screen',
    )

    # [3] 사람 검출/트래킹 (단일 노드) — 필요하면 parameters=[{...}]로 덮어쓰기 가능
    human_bbox_node = Node(
        package='lidar_focus',
        executable='human_bbox_node',
        output='screen',
        # parameters=[{'calib_enabled': True}],
    )

    # [5] 낙상 판정 (단일 노드)
    fall_detection_node = Node(
        package='lidar_focus',
        executable='fall_detection_node',
        output='screen',
        # parameters=[{'protect_active_normal': False}],
    )

    # [6] 마이크어레이 DOA
    sound_localizer_node = Node(
        package='sound_localizer',
        executable='sound_localizer_node',
        output='screen',
        condition=IfCondition(sound),
    )

    # [7] DOA 결과 마커 시각화
    sound_source_marker_node = Node(
        package='sound_localizer',
        executable='sound_source_marker_node',
        output='screen',
        condition=IfCondition(sound),
    )

    # [4] rviz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(rviz),
        # 저장해둔 설정이 있으면: arguments=['-d', '/path/to/config.rviz'],
    )

    return LaunchDescription(args + [
        lidar_driver_launch,
        ground_removal_node,
        human_bbox_node,
        fall_detection_node,
        sound_localizer_node,
        sound_source_marker_node,
        rviz_node,
    ])
