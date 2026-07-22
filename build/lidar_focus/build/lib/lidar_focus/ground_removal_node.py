import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
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
    """
    배경 차감 노드 (Mode A: 고정 배경 + 주기적 선택적 재학습).

    동작 방식:
      1. 시작 시 100프레임으로 배경 모델(KDTree)을 1회 학습 (기존과 동일).
      2. 학습 완료 후에는 그 배경을 그대로 고정해서 사용
         (정지한 사람도 계속 전경으로 잡힘 — 낙상 감지 요구사항).
      3. 단, 시간이 지나면서 "한 번 움직였다가 정지한 가구" 등이
         영구히 전경으로 남는 문제를 줄이기 위해, 주기적으로(relearn_interval_sec)
         배경을 처음부터 다시 100프레임 학습해서 통째로 교체한다.
      4. 재학습 도중에도 human_bbox_node가 '/protected_regions' 토픽으로
         알려주는 영역(현재 사람으로 확정된 트랙의 위치, 낙상자 포함)은
         절대 새 배경 데이터에 포함시키지 않는다.
         → 그 영역은 재학습 후에도 계속 "배경에 없는 점"으로 남아
           계속 전경(사람)으로 인식된다.
      5. 재학습 중에도 기존 배경 모델로 정상적으로 전경 검출을 계속하다가,
         새 배경 학습이 끝나는 순간 원자적으로 교체한다 (다운타임 없음).
    """

    def __init__(self):
        super().__init__('bg_subtraction')

        self.declare_parameter('bg_frame_count',        100)
        self.declare_parameter('bg_voxel_size',          0.05)
        self.declare_parameter('fg_dist_threshold',      0.1)
        # 0 이하로 설정하면 재학습 비활성화 (기존과 동일하게 영구 고정)
        self.declare_parameter('relearn_interval_sec',   300.0)
        # 보호 영역 마진 (각 축으로 추가 확장, 추정 오차/자세 변화 대비)
        self.declare_parameter('protect_margin',         0.4)
        # 마지막으로 받은 protected_regions 메시지가 이 시간(초)보다 오래되면
        # 더 이상 신뢰하지 않음 (human_bbox_node 다운 시 안전장치)
        self.declare_parameter('protect_region_timeout', 2.0)

        self.BG_FRAME_COUNT   = self.get_parameter('bg_frame_count').value
        self.bg_voxel_size    = self.get_parameter('bg_voxel_size').value
        self.fg_dist_thresh   = self.get_parameter('fg_dist_threshold').value
        self.relearn_interval = self.get_parameter('relearn_interval_sec').value
        self.protect_margin   = self.get_parameter('protect_margin').value
        self.protect_timeout  = self.get_parameter('protect_region_timeout').value

        # ── 초기 배경 학습 상태 ──
        self.bg_tree   = None
        self.bg_frames = []
        self.bg_ready  = False

        # ── 재학습(relearn) 상태 ──
        self.relearning     = False
        self.relearn_frames = []

        # ── 보호 영역 (human_bbox_node로부터 수신) ──
        # 각 원소: (min_xyz(3,), max_xyz(3,))
        self._protected_boxes = []
        self._protected_stamp_sec = -1.0

        cb_group = ReentrantCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/ground_removed_cloud', self.callback, 10,
            callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/filtered_cloud', 10)

        self.sub_protect = self.create_subscription(
            Float32MultiArray, '/protected_regions', self.on_protected_regions, 10,
            callback_group=cb_group)

        if self.relearn_interval > 0.0:
            self.relearn_timer = self.create_timer(
                self.relearn_interval, self._start_relearn,
                callback_group=cb_group)

        self.get_logger().info(
            f'BackgroundSubtractionNode started. '
            f'Collecting background ({self.BG_FRAME_COUNT} frames)...')

    # ────────────────────────────────────────────────────
    def on_protected_regions(self, msg):
        """
        human_bbox_node가 보내는 보호 영역 수신.
        포맷: 7개씩 [cx, cy, cz, sx, sy, sz, track_id] 반복.
        """
        data = np.array(msg.data, dtype=np.float64)
        boxes = []
        for i in range(0, len(data) - 6, 7):
            c = data[i:i+3]
            s = data[i+3:i+6]
            half = s / 2.0 + self.protect_margin
            boxes.append((c - half, c + half))
        self._protected_boxes = boxes
        self._protected_stamp_sec = self.get_clock().now().nanoseconds * 1e-9

    def _protected_boxes_valid(self):
        """타임아웃 안 지난 보호 영역만 반환 (안전장치)."""
        if not self._protected_boxes:
            return []
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._protected_stamp_sec > self.protect_timeout:
            return []
        return self._protected_boxes

    def _filter_out_protected(self, points):
        """
        points 중 현재 보호 영역(사람으로 확정된 트랙) 안에 있는 점을 제거.
        → 배경 학습용 데이터(bg_frames/relearn_frames)에는 이 점들이
          절대 들어가지 않도록 해서, 재학습 후에도 그 자리는 계속
          '배경에 없는 점 = 전경'으로 남는다.
        """
        boxes = self._protected_boxes_valid()
        if not boxes or len(points) == 0:
            return points
        inside = np.zeros(len(points), dtype=bool)
        for mn, mx in boxes:
            inside |= np.all((points >= mn) & (points <= mx), axis=1)
        return points[~inside]

    # ────────────────────────────────────────────────────
    def voxel_downsample(self, points, voxel_size=0.05):
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    def _start_relearn(self):
        if not self.bg_ready or self.relearning:
            return  # 초기 학습 안 끝났거나 이미 재학습 중이면 스킵
        self.relearning     = True
        self.relearn_frames = []
        self.get_logger().info(
            f'Periodic background relearn started '
            f'({len(self._protected_boxes_valid())} protected region(s) excluded).')

    def _finish_relearn(self):
        all_bg = np.vstack(self.relearn_frames)
        background = self.voxel_downsample(all_bg, voxel_size=self.bg_voxel_size)
        self.bg_tree = KDTree(background)  # 원자적 교체 (참조만 바꿔)
        self.relearning = False
        self.relearn_frames.clear()
        self.get_logger().info('Background model relearned & swapped.')

    # ────────────────────────────────────────────────────
    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        # ── 1. 초기 배경 학습 단계 ──
        if not self.bg_ready:
            self.bg_frames.append(self._filter_out_protected(points))
            self.get_logger().info(
                f'Background: {len(self.bg_frames)}/{self.BG_FRAME_COUNT}')

            if len(self.bg_frames) >= self.BG_FRAME_COUNT:
                all_bg = np.vstack(self.bg_frames)
                background = self.voxel_downsample(all_bg, voxel_size=self.bg_voxel_size)
                self.bg_tree = KDTree(background)
                self.bg_ready = True
                self.bg_frames.clear()
                self.get_logger().info('Background model ready!')

            # 수집 중에도 원본 그대로 발행 (다운스트림 끊기지 않게)
            self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))
            return

        # ── 2. 정상 동작: 고정된(혹은 직전에 재학습된) 배경으로 전경 검출 ──
        dists, _ = self.bg_tree.query(points, k=1)
        filtered = points[dists > self.fg_dist_thresh]

        if len(filtered) > 0:
            self.pub.publish(pc2.create_cloud_xyz32(msg.header, filtered))

        # ── 3. 재학습 진행 중이면, 같은 프레임의 점을 재학습 버퍼에도 누적 ──
        #     (기존 배경 기반 검출/발행은 위에서 끊김 없이 계속됨)
        if self.relearning:
            self.relearn_frames.append(self._filter_out_protected(points))
            if len(self.relearn_frames) >= self.BG_FRAME_COUNT:
                self._finish_relearn()


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

