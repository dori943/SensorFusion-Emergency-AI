import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree


def read_points(msg):
    """PointCloud2 â†’ numpy (ìµœì í™”)"""
    pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    if pts is None or len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return pts.astype(np.float32)


class LidarPreprocessor(Node):
    def __init__(self):
        super().__init__('lidar_preprocessor')

        self.declare_parameter('z_min', -0.3)
        self.declare_parameter('z_max', 3.0)
        self.declare_parameter('range_min', 0.5)
        self.declare_parameter('range_max', 15.0)
        self.declare_parameter('voxel_size', 0.05)

        cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.callback, 10,
            callback_group=cb_group)
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
        
        tree = KDTree(points[:, :2])
        dists, _ = tree.query(points[:, :2], k=nb_neighbors + 1)
        mean_dists = dists[:, 1:].mean(axis=1)
        threshold = mean_dists.mean() + std_ratio * mean_dists.std()
        return points[mean_dists < threshold]

    def callback(self, msg):
        points = read_points(msg)
        if len(points) == 0:
            return

        
        ranges = np.linalg.norm(points[:, :2], axis=1)
        z_min = self.get_parameter('z_min').value
        z_max = self.get_parameter('z_max').value
        range_min = self.get_parameter('range_min').value
        range_max = self.get_parameter('range_max').value

        mask = (
            (ranges > range_min) & (ranges < range_max) &
            (points[:, 2] > z_min) & (points[:, 2] < z_max)
        )
        points = points[mask]
        if len(points) == 0:
            return

        points = self.voxel_downsample(points, self.get_parameter('voxel_size').value)
        if len(points) == 0:
            return

        points = self.statistical_outlier_removal(points)
        if len(points) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))



class GroundRemovalNode(Node):
    def __init__(self):
        super().__init__('ground_removal')

        cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/preprocessed_cloud', self.callback, 10,
            callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/ground_removed_cloud', 10)

        self.get_logger().info('GroundRemovalNode started.')

    def ransac_plane(self, points, threshold=0.05, num_iterations=100):
        best_inliers = np.array([], dtype=np.int64)
        n = len(points)

        # ëžœë¤ ìƒ˜í”Œ í•œë²ˆì— ìƒì„±
        all_idx = np.random.choice(n, (num_iterations, 3), replace=True)

        for i in range(num_iterations):
            p1, p2, p3 = points[all_idx[i]]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            dists = np.abs(points @ normal - np.dot(normal, p1))
            inliers = np.where(dists < threshold)[0]
            if len(inliers) > len(best_inliers):
                best_inliers = inliers
                # ì¡°ê¸° ì¢…ë£Œ: 40% ì´ìƒì´ë©´ ì¶©ë¶„
                if len(inliers) > n * 0.4:
                    break

        return best_inliers

    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        inliers = self.ransac_plane(points)
        mask = np.ones(len(points), dtype=bool)
        mask[inliers] = False
        points = points[mask]

        if len(points) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))


class BackgroundSubtractionNode(Node):
    def __init__(self):
        super().__init__('bg_subtraction')

        self.bg_tree = None
        self.bg_frames = []
        self.bg_ready = False
        self.BG_FRAME_COUNT = 100

        cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/ground_removed_cloud', self.callback, 10,
            callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/filtered_cloud', 10)

        self.get_logger().info(
            'BackgroundSubtractionNode started. '
            'Collecting background (100 frames)...')

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
                self.bg_frames.clear()  # ë©”ëª¨ë¦¬ í•´ì œ
                self.get_logger().info('Background model ready!')

            # ðŸ”§ í•µì‹¬ ìˆ˜ì •: ìˆ˜ì§‘ ì¤‘ì—ë„ ì›ë³¸ ë°œí–‰ (ê¸°ì¡´ì—” returnìœ¼ë¡œ ëŠê¹€)
            self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))
            return

        dists, _ = self.bg_tree.query(points, k=1)
        filtered = points[dists > 0.1]

        if len(filtered) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, filtered))



def main(args=None):
    rclpy.init(args=args)

    preprocessor = LidarPreprocessor()
    ground_removal = GroundRemovalNode()
    bg_subtraction = BackgroundSubtractionNode()

    # ìŠ¤ë ˆë“œ ìˆ˜ ëª…ì‹œì ìœ¼ë¡œ ì§€ì •
    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
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

