#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
raw / 전처리 후 PointCloud2 토픽을 각각 독립적으로 녹화해서 .npy로 저장하는 ROS2 노드
======================================================================================

* 이전 버전은 ApproximateTimeSynchronizer로 두 토픽을 시간 동기화했지만,
  "사람 없는 정적 장면(empty scene)" 평가에는 프레임 단위 시간 동기화가
  필요하지 않음 (배경이 정적이라 어느 시점의 raw/processed든 비교 가능).
* 따라서 이 버전은 동기화 없이 두 토픽을 각각 독립적으로 구독하고,
  지정한 시간 동안 들어오는 모든 메시지를 그냥 누적한다.
  -> message_filters 관련 동기화 실패(타임스탬프 문제, QoS 문제 등) 이슈를 원천 제거.

사용법
------
python3 record_pointclouds.py \
    --raw-topic /unilidar/cloud \
    --processed-topic /filtered_cloud \
    --duration 30 \
    --output-dir ./empty_scene_data

결과: ./empty_scene_data/empty_raw.npy, ./empty_scene_data/empty_processed.npy

이후 바로 평가:
python3 eval_bg_subtraction.py \
    --raw ./empty_scene_data/empty_raw.npy \
    --processed ./empty_scene_data/empty_processed.npy \
    --mode empty

문제 진단이 필요하면 (녹화 중 메시지가 0개일 때) 다른 터미널에서:
  ros2 topic list                      # 두 토픽이 실제로 보이는지
  ros2 topic hz /unilidar/cloud        # 실제로 퍼블리시되고 있는지
  ros2 topic hz /filtered_cloud
"""

import argparse
import os
import sys

import numpy as np


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="raw/전처리 후 PointCloud2 토픽을 각각 독립적으로 녹화해서 .npy로 저장"
    )
    parser.add_argument("--raw-topic", type=str, default="/unilidar/cloud",
                         help="전처리 전 원본 포인트클라우드 토픽")
    parser.add_argument("--processed-topic", type=str, default="/filtered_cloud",
                         help="전처리(배경차감) 후 포인트클라우드 토픽")
    parser.add_argument("--duration", type=float, default=30.0,
                         help="녹화 시간(초). 이 시간이 지나면 자동 저장 후 종료 (기본 30초)")
    parser.add_argument("--output-dir", type=str, default=".",
                         help="저장 폴더")
    parser.add_argument("--prefix", type=str, default="empty",
                         help="출력 파일 접두어 (기본 'empty' -> empty_raw.npy / empty_processed.npy)")
    parser.add_argument("--qos", type=str, choices=["sensor_data", "reliable"], default="sensor_data",
                         help="구독 QoS 프로파일. 토픽이 안 잡히면 'reliable'로 바꿔서 시도해보세요.")
    return parser


def main():
    args = build_arg_parser().parse_args()

    try:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy
        from sensor_msgs.msg import PointCloud2
        from sensor_msgs_py import point_cloud2 as pc2
    except ImportError as e:
        print(f"[에러] ROS2 파이썬 패키지를 불러올 수 없습니다: {e}\n"
              f"ROS2 환경을 source 했는지 확인하세요. 예:\n"
              f"  source /opt/ros/<실제배포판이름>/setup.bash   # ls /opt/ros/ 로 확인\n"
              f"  source ~/sound_lidar_ws/install/setup.bash\n", file=sys.stderr)
        sys.exit(1)

    os.makedirs(args.output_dir, exist_ok=True)

    if args.qos == "sensor_data":
        qos_profile = qos_profile_sensor_data
    else:
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    def cloud_to_xyz(msg: "PointCloud2") -> np.ndarray:
        """PointCloud2 -> (N,3) numpy array (x,y,z만 추출)."""
        if hasattr(pc2, "read_points_numpy"):
            arr = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
            return np.asarray(arr, dtype=np.float64).reshape(-1, 3)
        else:
            points = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
            if len(points) == 0:
                return np.zeros((0, 3), dtype=np.float64)
            return np.array([[p[0], p[1], p[2]] for p in points], dtype=np.float64)

    class Recorder(Node):
        def __init__(self):
            super().__init__("pointcloud_independent_recorder")

            self.raw_frames = []
            self.processed_frames = []
            self.n_raw_msgs = 0
            self.n_proc_msgs = 0
            self._finished = False

            self.create_subscription(
                PointCloud2, args.raw_topic, self.on_raw, qos_profile)
            self.create_subscription(
                PointCloud2, args.processed_topic, self.on_processed, qos_profile)

            self.get_logger().info(
                f"구독 시작: raw='{args.raw_topic}', processed='{args.processed_topic}', "
                f"duration={args.duration}s, qos={args.qos}"
            )
            self.get_logger().info(
                "두 토픽을 독립적으로 누적합니다 (시간 동기화 없음 - 정적 배경 평가용)."
            )

            self.start_time = self.get_clock().now()
            self.timer = self.create_timer(0.5, self.check_timeout)

        def on_raw(self, msg):
            xyz = cloud_to_xyz(msg)
            self.raw_frames.append(xyz)
            self.n_raw_msgs += 1
            if self.n_raw_msgs % 20 == 1:
                self.get_logger().info(f"[raw] {self.n_raw_msgs}개 메시지 수신 (최근 {len(xyz)}pts)")

        def on_processed(self, msg):
            xyz = cloud_to_xyz(msg)
            self.processed_frames.append(xyz)
            self.n_proc_msgs += 1
            if self.n_proc_msgs % 20 == 1:
                self.get_logger().info(f"[processed] {self.n_proc_msgs}개 메시지 수신 (최근 {len(xyz)}pts)")

        def check_timeout(self):
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            if elapsed >= args.duration:
                self.finish_and_shutdown()

        def finish_and_shutdown(self):
            if self._finished:
                return
            self._finished = True

            if self.n_raw_msgs == 0 or self.n_proc_msgs == 0:
                self.get_logger().warn(
                    f"메시지 수신 부족: raw={self.n_raw_msgs}개, processed={self.n_proc_msgs}개. "
                    f"토픽 이름/QoS를 확인하세요 (다른 터미널에서 "
                    f"'ros2 topic hz {args.raw_topic}', 'ros2 topic hz {args.processed_topic}' 실행)."
                )
            else:
                raw_all = np.vstack(self.raw_frames)
                proc_all = np.vstack(self.processed_frames)

                raw_path = os.path.join(args.output_dir, f"{args.prefix}_raw.npy")
                proc_path = os.path.join(args.output_dir, f"{args.prefix}_processed.npy")
                np.save(raw_path, raw_all)
                np.save(proc_path, proc_all)

                self.get_logger().info(
                    f"저장 완료: raw {self.n_raw_msgs}msgs/{raw_all.shape} -> {raw_path}, "
                    f"processed {self.n_proc_msgs}msgs/{proc_all.shape} -> {proc_path}"
                )
            rclpy.try_shutdown()

    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish_and_shutdown()
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
