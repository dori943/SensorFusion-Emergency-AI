#!/usr/bin/env python3

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


def voxel_downsample(points, voxel_size):
    if len(points) == 0:
        return points

    voxel_indices = np.floor(points / voxel_size).astype(np.int64)
    _, unique_indices = np.unique(
        voxel_indices, axis=0, return_index=True
    )
    return points[unique_indices]


class Float32Recorder(Node):
    def __init__(self, topic, output, duration, voxel_size):
        super().__init__('float32_pointcloud_recorder')

        self.output = open(output, 'wb')
        self.voxel_size = voxel_size
        self.frame_count = 0
        self.finished = False

        self.subscription = self.create_subscription(
            PointCloud2,
            topic,
            self.on_cloud,
            qos_profile_sensor_data,
        )

        self.timer = self.create_timer(duration, self.finish)

        self.get_logger().info(
            f'녹화 시작: topic={topic}, output={output}, '
            f'duration={duration}s, voxel_size={voxel_size}m'
        )

    def on_cloud(self, msg):
        if self.finished:
            return

        points = pc2.read_points_numpy(
            msg,
            field_names=('x', 'y', 'z'),
            skip_nans=True,
        )

        if points is None or len(points) == 0:
            return

        points = np.asarray(points, dtype=np.float32).reshape(-1, 3)
        points = voxel_downsample(points, self.voxel_size)
        points = np.asarray(points, dtype='<f4')

        # 프레임 포인트 개수
        header = np.array([len(points)], dtype='<f4')

        self.output.write(header.tobytes())
        self.output.write(points.tobytes(order='C'))

        self.frame_count += 1

        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'{self.frame_count}프레임 저장, 최근 {len(points)} points'
            )

    def finish(self):
        if self.finished:
            return

        self.finished = True
        self.output.flush()
        self.output.close()

        self.get_logger().info(
            f'저장 완료: {self.frame_count}프레임'
        )

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/unilidar/cloud')
    parser.add_argument('--output', default='pointcloud_frames.bin')
    parser.add_argument('--duration', type=float, default=30.0)
    parser.add_argument('--voxel-size', type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = Float32Recorder(
        topic=args.topic,
        output=args.output,
        duration=args.duration,
        voxel_size=args.voxel_size,
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.finish()
    finally:
        if not node.finished:
            node.finish()
        node.destroy_node()


if __name__ == '__main__':
    main()
