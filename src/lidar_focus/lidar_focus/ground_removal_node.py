import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree


def read_points(msg):
    """PointCloud2 → numpy (최적화)"""
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

        # ── 부팅타임 자동 캘리브레이션 ─────────────────────
        # 엣지 디바이스에 새 장소마다 사람이 직접 z_min/z_max/range_max를
        # 재는 대신, 시작 직후 몇 프레임을 관찰해서 "이 방의 지면 높이
        # /천장 높이/유효 거리"를 스스로 추정해 파라미터를 갱신한다.
        # (사람이 서 있어야 하는 human_bbox 쪽 필터와 달리, 이 값들은
        #  방이 비어 있어도 지면/벽 형상만으로 추정 가능하다.)
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calib_num_frames', 30)       # 캘리브레이션에 쓸 프레임 수
        self.declare_parameter('calib_ground_margin', 0.05)  # 지면 위 이만큼(m)을 z_min으로
        self.declare_parameter('calib_ceiling_percentile', 99.0)  # z_max = 이 백분위 높이
        self.declare_parameter('calib_range_percentile', 95.0)    # range_max 추정 기준 백분위
        self.declare_parameter('calib_range_margin', 1.15)        # 위 값에 여유율 곱함
        self.declare_parameter('calib_range_cap', 20.0)           # 안전 상한(m)

        # ── 캘리브레이션 실패 감지용 파라미터 ──
        self.declare_parameter('calib_min_z_gap', 1.0)  # z_max-z_min이 이보다 작으면 사람이
                                                          # 들어갈 공간이 없다고 보고 캘리브레이션 무효 처리

        self.declare_parameter('calib_range_min', 5.0)  # range_max 자동 추정 하한(m).
        # 자동값이 이보다 작게 잡히면 검출 사각을 막기 위해 이 값으로 올린다.
        self.declare_parameter('calib_ground_min_inlier_ratio', 0.05)  # 지면 평면 최소 inlier 비율
        self.declare_parameter('calib_ground_z_percentile', 40.0)      # 지면 후보 median z 상한(백분위)

        self._calibrating = bool(self.get_parameter('auto_calibrate').value)
        self._calib_buffer = []
        self._last_mask_empty_log = 0.0  # 빈 결과 경고 로그 스로틀용

        # ReentrantCallbackGroup을 쓰면 MultiThreadedExecutor 하에서 같은 콜백이
        # 여러 스레드에서 동시에 실행될 수 있어 self._calib_buffer / self._calibrating
        # 같은 공유 상태가 레이스 컨디션으로 깨질 수 있다 (예: 콜백이 겹쳐 실행되며
        # 캘리브레이션이 두 번 트리거되거나 버퍼가 중간에 비워지는 문제).
        # 이 콜백은 서비스 호출 등 재진입이 필요 없으므로 MutuallyExclusive로 직렬화한다.
        cb_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/unilidar/cloud', self.callback, 10,
            callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/preprocessed_cloud', 10)

        self.get_logger().info('LidarPreprocessor started.')
        if self._calibrating:
            self.get_logger().info(
                f'자동 캘리브레이션 시작 — {self.get_parameter("calib_num_frames").value}'
                f'프레임 관찰 후 z_min/z_max/range_max를 자동 설정합니다 '
                f'(그동안은 기본값으로 정상 동작).'
            )

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

    # ──────────────────────────────────────────────────────
    # 부팅타임 자동 캘리브레이션
    # ──────────────────────────────────────────────────────
    def _estimate_ground_plane(self, points, threshold=0.05, iterations=300,
                               horizontal_min=0.85, min_inlier_ratio=0.05,
                               ground_z_percentile=40.0):
        """RANSAC으로 '거의 수평이면서 가장 낮은' 평면(=지면)을 찾는다.

        기존 구현은 수평면 후보 중 inlier가 '가장 많은' 평면을 골랐다.
        바닥이 가구/그림자로 가려 성기고 천장이 더 넓고 평평하면 천장이
        이겨서 지면으로 오인식됐다(실측: ground_z≈2.5m).
        지면은 '장면에서 가장 낮은 수평면'이어야 하므로, 여기서는
          (1) 법선이 z축과 거의 나란하고(horizontal_min 이상),
          (2) inlier가 충분하며(min_inlier_ratio 이상),
          (3) median z가 장면 하위(ground_z_percentile 백분위 이하)인
        후보들 가운데 median z가 '가장 낮은' 평면을 지면으로 택한다.

        반환: (ground_z, residual_std, found)
        """
        n = len(points)
        if n < 50:
            return 0.0, 0.0, False

        min_support = max(50, int(n * min_inlier_ratio))
        z_upper_limit = float(np.percentile(points[:, 2], ground_z_percentile))

        # 지면은 소수 포인트일 수 있어(가구/그림자로 가려짐) 전체에서 3점을
        # 무작위로 뽑으면 바닥 삼각형이 거의 안 걸린다. 시드 후보를 '장면 하위'
        # (z <= z_upper_limit)로 제한해 바닥 평면이 안정적으로 잡히게 한다.
        low_idx = np.where(points[:, 2] <= z_upper_limit)[0]
        if len(low_idx) < 3:
            return 0.0, 0.0, False
        rng = np.random.default_rng()
        idx = rng.choice(low_idx, (iterations, 3), replace=True)

        best_z = None
        best_inliers = None
        best_normal = None
        best_p1 = None
        for i in range(iterations):
            p1, p2, p3 = points[idx[i]]
            normal = np.cross(p2 - p1, p3 - p1)
            norm = np.linalg.norm(normal)
            if norm < 1e-6:
                continue
            normal /= norm
            if abs(normal[2]) < horizontal_min:
                continue  # 수평에 가깝지 않으면(벽 등) 지면 후보에서 제외
            dists = np.abs(points @ normal - np.dot(normal, p1))
            inliers = np.where(dists < threshold)[0]
            if len(inliers) < min_support:
                continue
            cand_z = float(np.median(points[inliers, 2]))
            if cand_z > z_upper_limit:
                continue  # 장면 상단(천장/선반 등)은 지면이 아니다
            if best_z is None or cand_z < best_z:
                best_z = cand_z
                best_inliers = inliers
                best_normal = normal
                best_p1 = p1

        if best_normal is None:
            return 0.0, 0.0, False

        ground_z = float(np.median(points[best_inliers, 2]))
        residual = np.abs(points[best_inliers] @ best_normal -
                          np.dot(best_normal, best_p1))
        residual_std = float(residual.std())
        return ground_z, residual_std, True

    def _run_auto_calibration(self):
        all_points = np.vstack(self._calib_buffer)
        self._calib_buffer.clear()
        self._calibrating = False

        ground_z, residual_std, found = self._estimate_ground_plane(
            all_points,
            min_inlier_ratio=self.get_parameter(
                'calib_ground_min_inlier_ratio').value,
            ground_z_percentile=self.get_parameter(
                'calib_ground_z_percentile').value)
        margin = self.get_parameter('calib_ground_margin').value

        if found:
            z_min_new = float(ground_z + margin)
        else:
            z_min_new = self.get_parameter('z_min').value
            self.get_logger().warn(
                '자동 캘리브레이션: 수평 지면 평면을 찾지 못해 z_min은 기본값 유지 '
                '(센서가 지면을 충분히 못 보는 각도/높이일 수 있음)')

        ceiling_pct = self.get_parameter('calib_ceiling_percentile').value
        z_max_new = float(np.percentile(all_points[:, 2], ceiling_pct))

        ranges = np.linalg.norm(all_points[:, :2], axis=1)
        range_pct = self.get_parameter('calib_range_percentile').value
        range_margin = self.get_parameter('calib_range_margin').value
        range_cap = self.get_parameter('calib_range_cap').value
        range_floor = self.get_parameter('calib_range_min').value
        range_raw = float(np.percentile(ranges, range_pct) * range_margin)
        range_max_new = float(np.clip(range_raw, range_floor, range_cap))
        if range_raw < range_floor:
            self.get_logger().warn(
                f'[캘리브레이션] 자동 추정 range_max({range_raw:.2f}m)가 하한'
                f'(calib_range_min={range_floor:.2f}m)보다 작아 {range_floor:.2f}m로 '
                f'올림 — 실제 방이 이보다 크면 calib_range_min을 키울 것.')

        # ── 안전장치: z_min/z_max가 깨졌으면(=사람이 들어갈 공간이 없으면)
        # 절대 적용하지 않는다. 여기서 그냥 통과시키면 이후 모든 프레임의
        # 마스크가 항상 빈 배열이 되어 파이프라인이 아무 로그도 없이
        # 조용히 멈춘다 (이게 실제로 발생했던 증상).
        min_gap = self.get_parameter('calib_min_z_gap').value
        z_min_cur = self.get_parameter('z_min').value
        z_max_cur = self.get_parameter('z_max').value
        range_min_cur = self.get_parameter('range_min').value

        z_gap_ok = (z_max_new - z_min_new) >= min_gap
        range_ok = range_max_new > range_min_cur

        if not z_gap_ok:
            self.get_logger().error(
                f'[캘리브레이션 무효화] z_min_new={z_min_new:.2f}m, '
                f'z_max_new={z_max_new:.2f}m (gap={z_max_new - z_min_new:.2f}m < '
                f'{min_gap}m). RANSAC이 지면 대신 천장/다른 수평 구조물을 '
                f'"지면"으로 오인식했을 가능성이 높다 (ground_z={ground_z:.2f}m, '
                f'잔차 std={residual_std:.3f}m). z_min/z_max는 기존 값'
                f'({z_min_cur:.2f}~{z_max_cur:.2f}m)을 그대로 유지한다.'
            )
            z_min_new = z_min_cur
            z_max_new = z_max_cur

        if not range_ok:
            self.get_logger().error(
                f'[캘리브레이션 무효화] range_max_new={range_max_new:.2f}m가 '
                f'range_min({range_min_cur:.2f}m)보다 작거나 같아 기존 range_max를 유지한다.'
            )
            range_max_new = self.get_parameter('range_max').value

        self.set_parameters([
            Parameter('z_min', value=z_min_new),
            Parameter('z_max', value=z_max_new),
            Parameter('range_max', value=range_max_new),
        ])

        self.get_logger().info(
            f'[자동 캘리브레이션 완료] z_min={z_min_new:.2f}m z_max={z_max_new:.2f}m '
            f'range_max={range_max_new:.2f}m '
            f'(지면 잔차 std={residual_std:.3f}m, {len(all_points)}pts 관찰)'
        )

    def callback(self, msg):
        points = read_points(msg)
        if len(points) == 0:
            return

        if self._calibrating:
            self._calib_buffer.append(points)
            if len(self._calib_buffer) >= self.get_parameter('calib_num_frames').value:
                self._run_auto_calibration()

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
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_mask_empty_log > 2.0:  # 2초에 한 번만 로그
                self._last_mask_empty_log = now
                self.get_logger().warn(
                    f'전처리 마스크 통과 포인트 0개 — z_min={z_min:.2f} z_max={z_max:.2f} '
                    f'range=({range_min:.2f},{range_max:.2f}) 조건을 확인할 것. '
                    f'/preprocessed_cloud가 계속 안 나오면 이 로그가 원인이다.'
                )
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

        # RANSAC 평면 제거 임계값 — 라이다 노이즈 수준에 따라 달라야 하므로
        # 고정값 대신 부팅 시 관찰로 자동 추정한다.
        self.declare_parameter('ransac_threshold', 0.05)
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calib_num_frames', 20)
        self.declare_parameter('calib_threshold_factor', 3.0)  # 잔차 std의 몇 배를 임계값으로
        self.declare_parameter('calib_threshold_min', 0.02)
        self.declare_parameter('calib_threshold_max', 0.15)

        self._calibrating = bool(self.get_parameter('auto_calibrate').value)
        self._calib_buffer = []
        self._last_empty_log = 0.0

        # LidarPreprocessor와 동일한 이유로 MutuallyExclusive 사용
        # (self._calib_buffer / self._calibrating 레이스 컨디션 방지)
        cb_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/preprocessed_cloud', self.callback, 10,
            callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/ground_removed_cloud', 10)

        self.get_logger().info('GroundRemovalNode started.')

    def ransac_plane(self, points, threshold=0.05, num_iterations=100,
                      return_residual_std=False):
        best_inliers = np.array([], dtype=np.int64)
        best_normal = None
        best_p1 = None
        n = len(points)

        # 랜덤 샘플 한번에 생성
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
                best_normal = normal
                best_p1 = p1
                # 조기 종료: 40% 이상이면 충분
                if len(inliers) > n * 0.4:
                    break

        if not return_residual_std:
            return best_inliers

        if best_normal is None or len(best_inliers) == 0:
            return best_inliers, 0.0

        residual = np.abs(points[best_inliers] @ best_normal -
                           np.dot(best_normal, best_p1))
        return best_inliers, float(residual.std())

    # ──────────────────────────────────────────────────────
    # 부팅타임 자동 캘리브레이션 (라이다 노이즈 수준에 맞춰 ransac_threshold 결정)
    # ──────────────────────────────────────────────────────
    def _run_auto_calibration(self):
        all_points = np.vstack(self._calib_buffer)
        self._calib_buffer.clear()
        self._calibrating = False

        if len(all_points) < 50:
            self.get_logger().warn(
                '자동 캘리브레이션: 포인트가 너무 적어 ransac_threshold 기본값 유지')
            return

        # 넉넉한 임시 threshold(0.1m)로 일단 가장 큰 평면을 찾고, 그 평면
        # inlier들의 잔차 표준편차로 실제 센서 노이즈 수준을 추정한다.
        _, residual_std = self.ransac_plane(
            all_points, threshold=0.1, num_iterations=150,
            return_residual_std=True)

        factor = self.get_parameter('calib_threshold_factor').value
        t_min = self.get_parameter('calib_threshold_min').value
        t_max = self.get_parameter('calib_threshold_max').value
        threshold_new = float(np.clip(residual_std * factor, t_min, t_max))

        self.set_parameters([Parameter('ransac_threshold', value=threshold_new)])
        self.get_logger().info(
            f'[자동 캘리브레이션 완료] ransac_threshold={threshold_new:.3f}m '
            f'(평면 잔차 std={residual_std:.4f}m 기반)'
        )

    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        if self._calibrating:
            self._calib_buffer.append(points)
            if len(self._calib_buffer) >= self.get_parameter('calib_num_frames').value:
                self._run_auto_calibration()

        threshold = self.get_parameter('ransac_threshold').value
        inliers = self.ransac_plane(points, threshold=threshold)
        mask = np.ones(len(points), dtype=bool)
        mask[inliers] = False
        removed_ratio = len(inliers) / len(points) if len(points) else 0.0
        points = points[mask]

        if len(points) == 0:
            now = self.get_clock().now().nanoseconds / 1e9
            if now - self._last_empty_log > 2.0:
                self._last_empty_log = now
                self.get_logger().warn(
                    f'RANSAC 지면 제거 후 포인트 0개 (제거 비율={removed_ratio:.0%}, '
                    f'threshold={threshold:.3f}m). ransac_threshold가 너무 크거나 '
                    f'수평 평면(천장 등)을 지면으로 오인식했을 가능성이 있음.'
                )
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))


class BackgroundSubtractionNode(Node):
    def __init__(self):
        super().__init__('bg_subtraction')

        self.bg_tree = None
        self.bg_frames = []
        self.bg_ready = False
        self.BG_FRAME_COUNT = 100

        # 배경(정적 장면)과 사람/물체 이동을 가르는 거리 임계값.
        # 라이다 노이즈/거리 정밀도가 장비·환경마다 다르므로 고정값 대신
        # 이미 수집 중인 배경 프레임들 자체의 프레임 간 흔들림(jitter)을
        # 이용해 자동으로 정한다 (별도 캘리브레이션 프레임이 필요 없음).
        self.declare_parameter('bg_diff_threshold', 0.1)
        self.declare_parameter('auto_calibrate', True)
        self.declare_parameter('calib_threshold_factor', 3.0)  # 노이즈 대비 배수
        self.declare_parameter('calib_threshold_min', 0.03)
        # holdout 방식(누적·다운샘플된 조밀한 배경 모델 대비 새 프레임 거리)으로
        # 노이즈를 재면 비복기 스캔의 스캔-투-스캔 지터가 대부분 흡수되므로
        # 상한을 크게 잡을 필요가 없다. 오히려 상한이 크면(기존 0.5m) 배경 표면
        # 반경 0.5m 안의 모든 점이 배경으로 삼켜져서, 바닥 잔여 포인트 근처의
        # 다리·벽/가구 근처의 팔이 통째로 사라지고 몸통 중심만 남는 문제가
        # 생긴다 (RViz에서 /filtered_cloud 팔다리 소실 증상의 주원인).
        # 배경에 가까운 팔다리는 아래 히스테리시스 복원이 살리므로, 이 상한은
        # "확실한 전경" 판정 기준으로 타이트하게 유지한다.
        self.declare_parameter('calib_threshold_max', 0.2)
        self._auto_calibrate = bool(self.get_parameter('auto_calibrate').value)

        # ── 히스테리시스 전경 복원 (이중 임계값 + 영역 성장) ──
        # 단일 임계값 하나로 자르면 배경(바닥 잔여/벽/가구)에 가까운 팔다리가
        # 몸통과 함께 있어도 잘려나간다. 대신:
        #   1) dist > threshold 인 점을 '확실한 전경' 시드로 잡고,
        #   2) 시드에서 hysteresis_link_radius 이내로 공간적으로 연결되면서
        #      dist > threshold × hysteresis_low_factor 인 점까지 전경으로
        #      살린다 (연결이 이어지는 한 반복 확장).
        # 몸통(시드)에 붙어 있는 팔다리는 배경에 다소 가까워도 복원되고,
        # 시드와 연결이 없는 고립된 배경 노이즈는 그대로 걸러진다.
        self.declare_parameter('hysteresis_enabled', True)
        self.declare_parameter('hysteresis_low_factor', 0.5)    # T_low = threshold × 이 값
        self.declare_parameter('hysteresis_link_radius', 0.25)  # 시드-후보 연결 반경 (m)
        self.declare_parameter('hysteresis_max_iters', 8)       # 영역 성장 반복 상한 (Pi 비용 제한)

        # ── 사람 트랙 보호영역 ──────────────────────────────
        # fall_detection_node가 CANDIDATE/CONFIRMED/ACTIVE(=낙상 아니지만
        # 최근 실제 이동 중)로 판단한 트랙은, 이후 잠깐 정지하더라도
        # (=배경과의 거리가 가까워지더라도) 배경으로 흡수되어 사라지지
        # 않도록 좌표 주변을 보호영역으로 유지한다.
        # fall_events 갱신이 protection_timeout_sec 이상 끊기면
        # (움직임이 멈췄거나 트랙이 사라졌다는 뜻) 보호를 해제한다.
        # NORMAL(정지) 상태 트랙은 애초에 /fall_events에 실리지 않으므로
        # 사람으로 오분류된 정지 물체는 보호 대상이 아니라 일반 정지
        # 전경과 동일하게 배경에 흡수된다.
        self.declare_parameter('fall_events_topic', '/fall_events')
        self.declare_parameter('protection_radius', 1.0)        # m, 누운 사람 크기 고려
        self.declare_parameter('protection_timeout_sec', 3.0)   # 이 시간 갱신 없으면 보호 해제
        self._protected_zones = {}  # tid -> (cx, cy, cz, last_update_sec)

        # ── 적응형 배경 갱신 ────────────────────────────────
        # 부팅 시 학습한 배경은 고정이라, 한 번이라도 움직였던 가구 등이
        # 영원히 전경(=디스플레이 대상)으로 남아 지저분해지는 문제가 있다.
        # "계속 연속으로 움직이는 물체는 사람뿐"이라는 전제로, 같은 복셀에
        # stationary_frames_to_absorb 프레임 이상 연속으로 머무는 전경
        # 포인트만 배경 모델에 편입한다. 사람처럼 매 프레임 위치가 바뀌는
        # 물체는 스트릭이 쌓이지 않아 절대 편입되지 않는다.
        # (단, 보호영역(CANDIDATE/CONFIRMED/ACTIVE) 안의 포인트는 편입
        #  후보에서 제외해 쓰러진 사람이나 이동 중인 사람이 배경으로
        #  흡수되지 않도록 한다.)
        self.declare_parameter('adaptive_bg_update', True)
        # 비복기 스캔 지터(수십 cm~1m대)가 이 복셀 크기보다 훨씬 크면, 정적인
        # 물체라도 매 프레임 다른 복셀에 찍혀 스트릭이 항상 리셋되고 절대
        # threshold_frames까지 쌓이지 못해 영원히 배경으로 편입되지 못한다.
        # 그래서 그룹핑용 복셀 크기와 "같은 자리로 볼" 매칭 반경을 지터 스케일에
        # 맞춰 크게 잡고, 아래 _update_stationary_tracking에서도 정확한 복셀 키
        # 일치가 아니라 반경 기반(KDTree) 매칭으로 흔들림을 흡수한다.
        self.declare_parameter('absorb_voxel_size', 0.5)           # 정지 판정용 클러스터링 복셀 크기
        self.declare_parameter('absorb_match_radius', 0.5)         # 이전 프레임 정지 클러스터와의 매칭 반경
        self.declare_parameter('stationary_frames_to_absorb', 30)  # ~10Hz 기준 약 3초
        self._stationary_track = []  # [{'center': (x,y,z), 'streak': n}, ...]
        self._bg_points = None       # 배경 모델의 실제 포인트 (편입 시 계속 갱신)

        # 핵심 수정: 이 노드는 self.bg_frames / self.bg_ready / self._bg_points /
        # self.bg_tree / self._stationary_track / self._protected_zones를 콜백 간에
        # 공유한다. ReentrantCallbackGroup + MultiThreadedExecutor(4 threads) 조합에서는
        # callback()이 여러 스레드에서 동시에 실행될 수 있어, 배경 수집 카운트가
        # 100을 넘어가거나(예: "101/100") 한 스레드가 self.bg_frames.clear()  # 메모리 해제
        # 직후 다른 스레드가 이미 비워진 리스트로 캘리브레이션을 시도해 "표본부족으로
        # 실패" 로그가 이어지는 문제가 발생했다. callback()과 _on_fall_events()도
        # 같은 상태(_protected_zones, _bg_points)를 건드리므로 같은
        # MutuallyExclusiveCallbackGroup에 묶어 직렬화한다.
        cb_group = MutuallyExclusiveCallbackGroup()
        self.sub = self.create_subscription(
            PointCloud2, '/ground_removed_cloud', self.callback, 10,
            callback_group=cb_group)
        self.sub_fall = self.create_subscription(
            Float32MultiArray, self.get_parameter('fall_events_topic').value,
            self._on_fall_events, 10, callback_group=cb_group)
        self.pub = self.create_publisher(
            PointCloud2, '/filtered_cloud', 10)

        self.get_logger().info(
            'BackgroundSubtractionNode started. '
            'Collecting background (100 frames)...')

    def voxel_downsample(self, points, voxel_size=0.05):
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    # ──────────────────────────────────────────────────────
    # 부팅타임 자동 캘리브레이션
    # ──────────────────────────────────────────────────────
    def _estimate_bg_noise(self, frames):
        """정적 장면의 노이즈 수준을 holdout 방식으로 추정한다.

        런타임에는 매 프레임을 '100프레임을 누적·다운샘플한 조밀한 배경 모델'과
        비교한다. 그런데 이전 구현은 성긴 단일 프레임 두 장을 직접 비교(frame i
        vs i+1)해서 실제보다 훨씬 큰 지터를 얻었고, 그 결과 임계값이 1m대로 튀어
        배경차감이 사실상 모든 점을 걸러 /filtered_cloud가 비었다.

        여기서는 프레임의 앞 80%로 배경 모델을 만들고, 나머지 20% 프레임의 점이
        그 모델에서 얼마나 떨어지는지(95퍼센타일)를 잰다. 곧 '학습에 안 쓰인 새
        정적 프레임 vs 배경 모델' 거리로, 런타임과 동일한 양을 측정한다.
        """
        if len(frames) < 4:
            return None
        split = max(1, int(len(frames) * 0.8))
        model_frames = frames[:split]
        query_frames = frames[split:]
        if not query_frames:
            return None
        model_pts = self.voxel_downsample(
            np.vstack(model_frames), voxel_size=0.05)
        if len(model_pts) < 10:
            return None
        tree = KDTree(model_pts)
        sample_dists = []
        for f in query_frames:
            if len(f) < 10:
                continue
            d, _ = tree.query(f, k=1)
            sample_dists.append(d)
        if not sample_dists:
            return None
        return float(np.percentile(np.concatenate(sample_dists), 95))

    def _run_bg_threshold_calibration(self, frames):
        """holdout 방식으로 측정한 노이즈(모델 대비 95퍼센타일)를 근거로
        bg_diff_threshold를 자동 산출해 적용한다.
        """
        noise = self._estimate_bg_noise(frames)
        if noise is None:
            self.get_logger().warn(
                '[배경 노이즈 캘리브레이션] 표본 부족으로 실패, '
                'bg_diff_threshold 기본값 유지')
            return

        factor = self.get_parameter('calib_threshold_factor').value
        t_min = self.get_parameter('calib_threshold_min').value
        t_max = self.get_parameter('calib_threshold_max').value
        raw_threshold = noise * factor
        threshold_new = float(np.clip(raw_threshold, t_min, t_max))

        if raw_threshold > t_max:
            self.get_logger().warn(
                f'[배경 노이즈 캘리브레이션] 계산값({raw_threshold:.3f}m = 모델대비 '
                f'노이즈 95pct {noise:.3f}m × factor{factor:.1f})이 상한'
                f'(calib_threshold_max={t_max:.3f}m)보다 커서 {t_max:.3f}m로 clip됨. '
                f'배경 수집 프레임 수/voxel_size를 점검할 것.')
        elif raw_threshold < t_min:
            self.get_logger().warn(
                f'[배경 노이즈 캘리브레이션] 계산값({raw_threshold:.3f}m)이 '
                f'하한(calib_threshold_min={t_min:.3f}m)보다 작아 {t_min:.3f}m로 clip됨.')

        self.set_parameters([Parameter('bg_diff_threshold', value=threshold_new)])
        self.get_logger().info(
            f'[배경 노이즈 캘리브레이션 완료] bg_diff_threshold={threshold_new:.3f}m '
            f'(모델대비 노이즈 95pct={noise:.3f}m 기반, clip 전 계산값={raw_threshold:.3f}m)')

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ──────────────────────────────────────────────────────
    # 히스테리시스 전경 복원
    # ──────────────────────────────────────────────────────
    def _hysteresis_foreground(self, points, dists, threshold):
        """이중 임계값 + 영역 성장으로 전경 마스크를 만든다.

        dist > threshold          : 확실한 전경(시드) — 무조건 유지
        T_low < dist <= threshold : 후보 — 시드와 공간적으로 연결될 때만 복원
        dist <= T_low             : 배경 — 항상 제거

        복원은 시드에서 시작해 link_radius 이내 후보를 반복적으로 흡수하는
        BFS 영역 성장. 몸통에 이어진 팔다리는 살아나고, 시드와 연결되지 않은
        고립 노이즈(배경 표면의 잔여 지터)는 복원되지 않는다.
        """
        seed_mask = dists > threshold
        if not self.get_parameter('hysteresis_enabled').value:
            return seed_mask
        if not seed_mask.any():
            return seed_mask

        low = threshold * float(self.get_parameter('hysteresis_low_factor').value)
        link_r = float(self.get_parameter('hysteresis_link_radius').value)
        max_iters = int(self.get_parameter('hysteresis_max_iters').value)

        cand_idx = np.where((dists > low) & (~seed_mask))[0]
        if len(cand_idx) == 0:
            return seed_mask

        cand_tree = KDTree(points[cand_idx])
        rescued = np.zeros(len(cand_idx), dtype=bool)
        frontier = points[seed_mask]

        for _ in range(max_iters):
            newly = set()
            for lst in cand_tree.query_ball_point(frontier, link_r):
                newly.update(lst)
            newly = [i for i in newly if not rescued[i]]
            if not newly:
                break
            newly = np.asarray(newly, dtype=int)
            rescued[newly] = True
            frontier = points[cand_idx[newly]]  # 새로 살린 점에서 계속 확장

        fg_mask = seed_mask.copy()
        fg_mask[cand_idx[rescued]] = True
        return fg_mask

    # ──────────────────────────────────────────────────────
    # 낙상 확정 보호영역 관리
    # ──────────────────────────────────────────────────────
    def _on_fall_events(self, msg):
        """fall_detection_node의 /fall_events 수신.

        포맷: 6개씩 반복 [track_id,cx,cy,cz,confidence,state_code]
        state_code: 1=CANDIDATE, 2=CONFIRMED, 3=ACTIVE(낙상 아니지만
        최근 실제 이동 중)

        CONFIRMED(state=2)만 보호영역으로 등록/갱신한다. CANDIDATE/ACTIVE는
        아직 낙상이 확정되지 않았거나 애초에 낙상이 아닌(이동 중) 트랙이라
        여기서 보호를 걸면 실제로 정지 흡수가 필요한 일반 정지 전경(가구 등)과의
        경계가 흐려진다. 확정된 낙상만 "쓰러진 뒤 정지해도 배경에 흡수되면
        안 되는 트랙"으로 간주해 보호한다.
        """
        now = self._now()
        data = msg.data
        for i in range(0, len(data) - 5, 6):
            tid = int(data[i])
            cx, cy, cz = data[i + 1:i + 4]
            state = int(data[i + 5])
            if state == 2:
                self._protected_zones[tid] = (float(cx), float(cy), float(cz), now)

        timeout = self.get_parameter('protection_timeout_sec').value
        stale = [tid for tid, (*_pos, t) in self._protected_zones.items()
                 if (now - t) > timeout]
        for tid in stale:
            del self._protected_zones[tid]
            self.get_logger().info(f'[보호영역 해제] track {tid} (낙상 이벤트 갱신 끊김)')

    def _protected_mask(self, points: np.ndarray) -> np.ndarray:
        """points 중 낙상 확정 보호영역 반경 내에 있는 점 마스크."""
        if not self._protected_zones:
            return np.zeros(len(points), dtype=bool)

        centers = np.array([(cx, cy, cz)
                             for (cx, cy, cz, _t) in self._protected_zones.values()])
        radius = self.get_parameter('protection_radius').value
        tree = KDTree(centers)
        dists, _ = tree.query(points, k=1)
        return dists <= radius

    # ──────────────────────────────────────────────────────
    # 적응형 배경 갱신 (정지 상태 트래킹 → 배경 편입)
    # ──────────────────────────────────────────────────────
    def _update_stationary_tracking(self, points: np.ndarray, fg_mask: np.ndarray,
                                     protected_mask: np.ndarray):
        """전경이면서 보호영역이 아닌 점들을 복셀 단위로 클러스터링하고, 그
        클러스터 중심이 이전 프레임의 정지 클러스터와 absorb_match_radius
        이내로 "근접"하면 같은 물체가 계속 그 자리에 있는 것으로 보고
        스트릭을 누적한다. 임계 프레임 이상 유지되면 배경으로 편입한다.

        360도 비복기 스캔은 완전히 정적인 물체라도 프레임마다 정확히 같은
        좌표를 찍지 않으므로, 이전 버전처럼 "정확히 같은 복셀 키"로 매칭하면
        스트릭이 항상 리셋되어 절대 편입되지 않는 문제가 있었다. 여기서는
        정확한 키 일치 대신 반경 기반(KDTree) 매칭으로 그 흔들림을 흡수한다.

        매 프레임 반경 밖으로 위치가 바뀌는(=계속 이동하는) 물체는 스트릭이
        절대 threshold_frames까지 쌓이지 않아 편입되지 않는다.
        """
        voxel_size = self.get_parameter('absorb_voxel_size').value
        match_radius = self.get_parameter('absorb_match_radius').value
        threshold_frames = int(self.get_parameter('stationary_frames_to_absorb').value)

        candidates = points[fg_mask & (~protected_mask)]
        if len(candidates) == 0:
            self._stationary_track = []
            return

        # 이번 프레임 후보들을 voxel_size 단위로 클러스터링해 대표 좌표(중심) 산출
        keys = np.floor(candidates / voxel_size).astype(np.int64)
        order = np.lexsort((keys[:, 2], keys[:, 1], keys[:, 0]))
        keys_sorted = keys[order]
        pts_sorted = candidates[order]
        split_idx = np.where(np.any(np.diff(keys_sorted, axis=0) != 0, axis=1))[0] + 1
        groups_pts = np.split(pts_sorted, split_idx)
        centers = np.array([gp.mean(axis=0) for gp in groups_pts])

        # 이전 프레임의 정지 클러스터와 반경 매칭 (근접하면 "같은 자리"로 간주)
        if self._stationary_track:
            prev_centers = np.array([t['center'] for t in self._stationary_track])
            tree = KDTree(prev_centers)
            dists, idxs = tree.query(centers, k=1)
            is_matched = dists <= match_radius
        else:
            is_matched = np.zeros(len(centers), dtype=bool)
            idxs = np.zeros(len(centers), dtype=int)

        new_track = []
        to_absorb = []
        used_prev = set()
        for i, center in enumerate(centers):
            prev_idx = int(idxs[i])
            if is_matched[i] and prev_idx not in used_prev:
                streak = self._stationary_track[prev_idx]['streak'] + 1
                used_prev.add(prev_idx)
            else:
                streak = 1
            if streak >= threshold_frames:
                to_absorb.append(center)  # 편입되면 더 이상 추적 불필요
            else:
                new_track.append({'center': center, 'streak': streak})

        # 이번 프레임에 근접 매칭되지 않은 이전 클러스터는 자연히 new_track에서
        # 빠짐 → 물체가 반경 밖으로 움직이면 스트릭이 리셋되는 효과는 유지된다.
        self._stationary_track = new_track

        if to_absorb:
            self._absorb_into_background(np.array(to_absorb))

    def _absorb_into_background(self, new_points: np.ndarray):
        self._bg_points = self.voxel_downsample(
            np.vstack([self._bg_points, new_points]), voxel_size=0.05)
        self.bg_tree = KDTree(self._bg_points)
        self.get_logger().info(
            f'[배경 편입] {len(new_points)}개 지점이 배경으로 흡수됨 '
            f'(정지 상태 {self.get_parameter("stationary_frames_to_absorb").value}'
            f'프레임 이상 유지)')

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
                self._bg_points = background
                self.bg_tree = KDTree(background)

                if self._auto_calibrate:
                    self._run_bg_threshold_calibration(self.bg_frames)

                self.bg_ready = True
                self.bg_frames.clear()  # ë©”ëª¨ë¦¬ í•´ì œ
                self.get_logger().info('Background model ready!')

            # 핵심 수정: 수집 중에도 원본 발행 (기존엔 return으로 끊김)
            self.pub.publish(pc2.create_cloud_xyz32(msg.header, points))
            return

        dists, _ = self.bg_tree.query(points, k=1)
        threshold = self.get_parameter('bg_diff_threshold').value
        # 이중 임계값(히스테리시스) 전경 판정 — 배경에 가까워 단일 임계값으로는
        # 잘렸을 팔다리를, 몸통(확실한 전경)과의 공간 연결성으로 복원한다.
        fg_mask = self._hysteresis_foreground(points, dists, threshold)

        # 낙상 확정 보호영역 안의 점은 배경과 가깝다는 이유로 걸러지지
        # 않도록 강제로 살린다 (정적으로 누워있는 사람이 배경에 흡수되는 것 방지).
        protected = self._protected_mask(points)

        if self.get_parameter('adaptive_bg_update').value:
            self._update_stationary_tracking(points, fg_mask, protected)

        keep_mask = fg_mask | protected
        filtered = points[keep_mask]

        if len(filtered) == 0:
            return

        self.pub.publish(pc2.create_cloud_xyz32(msg.header, filtered))



def main(args=None):
    rclpy.init(args=args)

    preprocessor = LidarPreprocessor()
    ground_removal = GroundRemovalNode()
    bg_subtraction = BackgroundSubtractionNode()

    # 스레드 수 명시적으로 지정
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
