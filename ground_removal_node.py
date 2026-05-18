import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree


def read_points(msg):
    """PointCloud2 → numpy (2D array 보장)"""
    pts = list(pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    if len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)


# ──────────────────────────────────────────────
# 1단계: 범위/높이/다운샘플/노이즈 제거
# 입력:  /unilidar/cloud
# 출력:  /preprocessed_cloud
# ──────────────────────────────────────────────
class LidarPreprocessor(Node):
    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.declare_parameter('z_min', -0.3)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('voxel_size', 0.05)

        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/preprocessed_cloud', 10)

        self.get_logger().info('LidarPreprocessor started.')

    def voxel_downsample(self, points, voxel_size):
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    def statistical_outlier_removal(self, points, nb_neighbors=20, std_ratio=2.0):
        if len(points) < nb_neighbors:
            return points
        tree = KDTree(points)
        dists, _ = tree.query(points, k=nb_neighbors + 1)
        mean_dists = dists[:, 1:].mean(axis=1)
        threshold = mean_dists.mean() + std_ratio * mean_dists.std()
        return points[mean_dists < threshold]

    def callback(self, msg):
        points = read_points(msg)
        if len(points) == 0:
            return

        # Step 1: 거리 필터
        ranges = np.linalg.norm(points[:, :2], axis=1)
        points = points[
            (ranges > self.get_parameter('range_min').value) &
            (ranges < self.get_parameter('range_max').value)]
        if len(points) == 0:
            return

        # Step 2: 높이 필터
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        points = points[(points[:, 2] > z_min) & (points[:, 2] < z_max)]
        if len(points) == 0:
            return

        # Step 3: Voxel 다운샘플
        points = self.voxel_downsample(points, self.get_parameter('voxel_size').value)
        if len(points) == 0:
            return

        # Step 4: Statistical Outlier Removal
        points = self.statistical_outlier_removal(points)
        if len(points) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))


# ──────────────────────────────────────────────
# 2단계: RANSAC 지면 제거
# 입력:  /preprocessed_cloud
# 출력:  /ground_removed_cloud
# ──────────────────────────────────────────────
class GroundRemovalNode(Node):
    def __init__(self):
        super().__init__('ground_removal')

        self.sub = self.create_subscription(
            PointCloud2, '/preprocessed_cloud', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/ground_removed_cloud', 10)

        self.get_logger().info('GroundRemovalNode started.')

    def ransac_plane(self, points, threshold=0.05, num_iterations=100):
        best_inliers = []
        n = len(points)
        for _ in range(num_iterations):
            idx = np.random.choice(n, 3, replace=False)
            p1, p2, p3 = points[idx]
            v1 = p2 - p1
            v2 = p3 - p1
            normal = np.cross(v1, v2)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal = normal / norm
            d = -np.dot(normal, p1)
            dists = np.abs(points @ normal + d)
            inliers = np.where(dists < threshold)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
        return best_inliers

    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        inliers = self.ransac_plane(points, threshold=0.05, num_iterations=100)
        mask = np.ones(len(points), dtype=bool)
        mask[inliers] = False
        points = points[mask]

        if len(points) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))


# ──────────────────────────────────────────────
# 3단계: 배경 제거
# 입력:  /ground_removed_cloud
# 출력:  /filtered_cloud  ← RViz2에서 이 토픽 구독
# ──────────────────────────────────────────────
class BackgroundSubtractionNode(Node):
    def __init__(self):
        super().__init__('bg_subtraction')

        self.bg_tree = None
        self.bg_frames = []
        self.bg_ready = False
        self.BG_FRAME_COUNT = 30

        self.sub = self.create_subscription(
            PointCloud2, '/ground_removed_cloud', self.callback, 10)
        self.pub = self.create_publisher(
            PointCloud2, '/filtered_cloud', 10)

        self.get_logger().info(
            'BackgroundSubtractionNode started. '
            'Collecting background (30 frames)...')

    def voxel_downsample(self, points, voxel_size=0.05):
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        if not self.bg_ready:
            self.bg_frames.append(points)
            self.get_logger().info(
                f'Background: {len(self.bg_frames)}/{self.BG_FRAME_COUNT}')

            if len(self.bg_frames) >= self.BG_FRAME_COUNT:
                all_bg = np.vstack(self.bg_frames)
                background = self.voxel_downsample(all_bg, voxel_size=0.05)
                self.bg_tree = KDTree(background)
                self.bg_ready = True
                self.get_logger().info('Background model ready!')
            return

        dists, _ = self.bg_tree.query(points, k=1)
        filtered = points[dists > 0.1]

        if len(filtered) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, filtered))


# ──────────────────────────────────────────────
# 메인
# ──────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)

    preprocessor = LidarPreprocessor()
    ground_removal = GroundRemovalNode()
    bg_subtraction = BackgroundSubtractionNode()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(preprocessor)
    executor.add_node(ground_removal)
    executor.add_node(bg_subtraction)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        preprocessor.destroy_node()
        ground_removal.destroy_node()
        bg_subtraction.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
