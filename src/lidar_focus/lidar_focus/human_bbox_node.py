

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Float32MultiArray
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
from scipy.spatial import KDTree
from collections import deque


def read_points(msg):
    """PointCloud2 → numpy (N,3) float32 (벡터화 버전)"""
    pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    if pts is None or len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return pts.astype(np.float32)


class SimpleTracker:
    """프레임 간 중심점 최근접 매칭 기반의 경량 트래커.

    실내 소수 인원 시나리오 전제의 그리디 매칭. 낙상 감지 노드가
    속도·자세 변화 이력을 추적하는 데 필요한 track_id를 유지한다.
    """

    def __init__(self, max_match_dist=0.8, max_missed=10):
        self.max_match_dist = max_match_dist  # 같은 트랙으로 볼 최대 xy 이동거리 (m)
        self.max_missed = max_missed          # 연속 미검출 허용 프레임 수
        self.tracks = {}   # track_id -> {'center': np.ndarray(3,), 'missed': int}
        self._next_id = 0

    def update(self, centers):
        assigned_ids = [None] * len(centers)

        pairs = []
        for ci, c in enumerate(centers):
            for tid, tr in self.tracks.items():
                d = float(np.linalg.norm(c[:2] - tr['center'][:2]))
                if d <= self.max_match_dist:
                    pairs.append((d, ci, tid))
        pairs.sort(key=lambda x: x[0])

        used_tracks = set()
        matched_centers = set()
        for d, ci, tid in pairs:
            if ci in matched_centers or tid in used_tracks:
                continue
            assigned_ids[ci] = tid
            used_tracks.add(tid)
            matched_centers.add(ci)
            self.tracks[tid]['center'] = centers[ci]
            self.tracks[tid]['missed'] = 0

        for ci, tid in enumerate(assigned_ids):
            if tid is None:
                new_id = self._next_id
                self._next_id += 1
                self.tracks[new_id] = {'center': centers[ci], 'missed': 0}
                assigned_ids[ci] = new_id
                used_tracks.add(new_id)

        for tid in list(self.tracks.keys()):
            if tid not in used_tracks:
                self.tracks[tid]['missed'] += 1
                if self.tracks[tid]['missed'] > self.max_missed:
                    del self.tracks[tid]

        return assigned_ids


class HumanBBoxNode(Node):
    """
    /filtered_cloud → DBSCAN → 사람 크기/형태 필터(서있음+누움 2중 모드)
    → SimpleTracker → /human_cloud, /human_bbox, /human_tracks,
      /protected_regions
    """

    def __init__(self):
        super().__init__('human_bbox_node')

        # ── 클러스터링 파라미터 ──────────────────────────
        self.declare_parameter('cluster_eps', 0.3)
        self.declare_parameter('cluster_min_points', 5)

        # ── 부팅타임 자동 캘리브레이션 (기하/노이즈 통계만 사용) ──
        self.declare_parameter('calib_enabled', True)
        self.declare_parameter('calib_frames', 30)
        self.declare_parameter('calib_eps_percentile', 90.0)
        self.declare_parameter('calib_eps_margin_factor', 2.5)
        self.declare_parameter('calib_min_points_ratio', 0.3)
        self.declare_parameter('calib_eps_min', 0.05)
        # 상한 1.0→0.5m: eps가 이보다 크면 실내에서 사람이 인접 가구/벽과
        # 한 클러스터로 붙어버려 크기 필터를 통과하지 못한다.
        self.declare_parameter('calib_eps_max', 0.5)

        self.calib_enabled = self.get_parameter('calib_enabled').value
        self.calib_frames_needed = int(self.get_parameter('calib_frames').value)
        self.calibrated = not self.calib_enabled
        self.calib_frame_count = 0
        self.calib_nn_dists = []
        self.calib_density_counts = []

        # ── 사람 크기 필터: 서있음/앉음 모드 ─────────────
        # dz(높이)가 person_height_min 이상이면 이 모드로 검사.
        # 앉은 사람(높이 0.8~1.3m)도 여기에 포함된다.
        self.declare_parameter('person_height_min', 0.75)
        self.declare_parameter('person_height_max', 2.2)
        self.declare_parameter('person_width_min', 0.2)
        self.declare_parameter('person_width_max', 1.2)
        self.declare_parameter('person_points_min', 5)

        # ── 사람 크기 필터: 누움 모드 ────────────────────
        # dz가 lying_height_min~person_height_min 사이면 이 모드로 검사.
        # 누운 성인: 높이 0.15~0.45m, 긴 축 1.2~1.9m, 짧은 축 0.3~0.8m.
        # (웅크리고 쓰러진 경우를 고려해 긴 축 하한은 0.7m로 완화)
        self.declare_parameter('lying_height_min', 0.12)
        self.declare_parameter('lying_major_min', 0.7)   # 긴 수평축 최소 (m)
        self.declare_parameter('lying_major_max', 2.2)   # 긴 수평축 최대 (m)
        self.declare_parameter('lying_minor_max', 1.2)   # 짧은 수평축 최대 (m)

        # ── PCA 형태 필터 ────────────────────────────────
        # 서있음 모드: 주축이 수직(verticality 높음) + 판형 아님(planarity 낮음)
        # 누움 모드: 주축이 수평인 것이 정상이므로 verticality '상한'을 두고,
        #            planarity로 테이블 상판/문짝 등 얇은 판만 배제한다.
        self.declare_parameter('person_verticality_min', 0.55)  # 서있음 모드 하한
        self.declare_parameter('person_planarity_max', 0.6)     # 서있음 모드 상한
        self.declare_parameter('lying_verticality_max', 0.7)    # 누움 모드 상한
        self.declare_parameter('lying_planarity_max', 0.7)      # 누움 모드 상한
        self.declare_parameter('pca_min_points', 15)

        # ── bbox 포인트 복원 (re-fetch) ──────────────────
        # /filtered_cloud는 배경차감을 거치며 배경(바닥/벽/가구)에 가까운
        # 팔다리 포인트가 깎여 성기다. 사람 클러스터가 '검출'된 뒤에는 그
        # bbox를 refetch_margin만큼 확장한 영역 안의 포인트를 배경차감 이전
        # 토픽(/ground_removed_cloud)에서 다시 가져와 /human_cloud로 발행한다.
        # → 검출은 깨끗한 filtered_cloud로, 시각화/후단 품질은 원본 밀도로.
        # (마커/트랙 좌표는 기존대로 filtered 클러스터 기준을 유지해
        #  주변 가구 포인트가 bbox 크기를 부풀리지 않게 한다.)
        self.declare_parameter('refetch_enabled', True)
        self.declare_parameter('refetch_source_topic', '/ground_removed_cloud')
        self.declare_parameter('refetch_margin', 0.15)       # bbox 확장 여유 (m)
        self.declare_parameter('refetch_max_age_sec', 0.5)   # 이보다 오래된 프레임은 미사용

        # ── 멀티프레임 누적 refetch ──────────────────────
        # 비반복 스캔 라이다는 프레임마다 서로 다른 지점을 찍으므로, 단일
        # 프레임 대신 최근 N프레임을 버퍼에 쌓아 bbox 영역 포인트를 누적
        # 수집하면 사람 위 포인트 밀도가 N배 가까이 올라간다 (raw가 조밀해
        # 보이는 것도 RViz의 프레임 잔상 누적 효과와 같은 원리).
        # 걷는 사람은 프레임 수 × 주기만큼 잔상이 생기지만(4프레임@10Hz ≈
        # 0.4초, 보행 1m/s 기준 ~40cm) bbox+margin 영역 안에서만 뽑으므로
        # 번짐은 그 안으로 제한된다. 1로 두면 기존 단일 프레임 동작과 동일.
        self.declare_parameter('refetch_accumulate_frames', 4)
        self.declare_parameter('refetch_dedup_voxel', 0.03)  # 누적 중복 제거 복셀 (m), 0이면 끔
        self._refetch_buffer = deque(maxlen=16)  # (stamp_sec, points) — 실사용 개수는 파라미터로 제한

        # ── 트래킹 파라미터 ──────────────────────────────
        self.declare_parameter('track_max_match_dist', 0.8)
        self.declare_parameter('track_max_missed', 10)
        self.tracker = SimpleTracker(
            max_match_dist=self.get_parameter('track_max_match_dist').value,
            max_missed=self.get_parameter('track_max_missed').value,
        )

        # ── ROS I/O ──────────────────────────────────────
        self.sub = self.create_subscription(
            PointCloud2, '/filtered_cloud', self.callback, 10)
        # bbox 포인트 복원용 원본(배경차감 이전) 클라우드 구독.
        # 기본 single-threaded executor에서 callback()과 직렬 실행되므로
        # self._refetch_points 접근에 별도 락은 필요 없다.
        self.sub_refetch = self.create_subscription(
            PointCloud2, self.get_parameter('refetch_source_topic').value,
            self._on_refetch_cloud, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/human_cloud', 10)
        self.pub_bbox = self.create_publisher(MarkerArray, '/human_bbox', 10)
        self.pub_tracks = self.create_publisher(Float32MultiArray, '/human_tracks', 10)
        self.pub_protected_regions = self.create_publisher(
            Float32MultiArray, '/protected_regions', 10)

        self.get_logger().info('HumanBBoxNode started (indoor / fall-aware).')
        self.get_logger().info(
            f'Standing filter | h {self.get_parameter("person_height_min").value}'
            f'~{self.get_parameter("person_height_max").value}m, '
            f'V>={self.get_parameter("person_verticality_min").value}, '
            f'P<={self.get_parameter("person_planarity_max").value}')
        self.get_logger().info(
            f'Lying filter    | h {self.get_parameter("lying_height_min").value}'
            f'~{self.get_parameter("person_height_min").value}m, '
            f'major {self.get_parameter("lying_major_min").value}'
            f'~{self.get_parameter("lying_major_max").value}m, '
            f'V<={self.get_parameter("lying_verticality_max").value}, '
            f'P<={self.get_parameter("lying_planarity_max").value}')
        if self.calib_enabled:
            self.get_logger().info(
                f'[Calib] Boot-time auto calibration ENABLED '
                f'({self.calib_frames_needed} frames, static scene assumed).')
        else:
            self.get_logger().info('[Calib] DISABLED. Using fixed parameters.')

    # ──────────────────────────────────────────────────────
    # 부팅타임 자동 캘리브레이션
    # ──────────────────────────────────────────────────────
    def _accumulate_calibration(self, points):
        tree = KDTree(points)

        k = min(2, len(points))
        dists, _ = tree.query(points, k=k)
        if dists.ndim == 1:
            return
        nn_dist = dists[:, 1]
        self.calib_nn_dists.append(nn_dist)

        provisional_eps = max(float(np.median(nn_dist)) * 3.0, 0.02)
        counts = tree.query_ball_point(points, provisional_eps, return_length=True)
        self.calib_density_counts.append(np.asarray(counts))

        self.calib_frame_count += 1
        if self.calib_frame_count % 10 == 0 or \
                self.calib_frame_count == self.calib_frames_needed:
            self.get_logger().info(
                f'[Calib] frame {self.calib_frame_count}/{self.calib_frames_needed} '
                f'({len(points)} pts, median NN={float(np.median(nn_dist)):.4f}m)')

        if self.calib_frame_count >= self.calib_frames_needed:
            self._finalize_calibration()

    def _finalize_calibration(self):
        all_nn = np.concatenate(self.calib_nn_dists)
        all_counts = np.concatenate(self.calib_density_counts)

        pct = self.get_parameter('calib_eps_percentile').value
        margin = self.get_parameter('calib_eps_margin_factor').value
        eps_min = self.get_parameter('calib_eps_min').value
        eps_max = self.get_parameter('calib_eps_max').value
        min_pts_ratio = self.get_parameter('calib_min_points_ratio').value

        noise_scale = float(np.percentile(all_nn, pct))
        raw_eps = noise_scale * margin
        new_eps = float(np.clip(raw_eps, eps_min, eps_max))
        if raw_eps > eps_max:
            self.get_logger().warn(
                f'[Calib] 계산된 eps({raw_eps:.3f}m)가 상한({eps_max:.2f}m)보다 커서 '
                f'clip됨. 포인트가 매우 성긴 환경 — 사람 클러스터가 조각날 수 '
                f'있으니 상류 voxel_size/필터를 점검할 것.')

        median_density = float(np.median(all_counts))
        new_min_points = int(np.clip(round(median_density * min_pts_ratio), 3, 20))

        self.set_parameters([
            Parameter('cluster_eps', Parameter.Type.DOUBLE, new_eps),
            Parameter('cluster_min_points', Parameter.Type.INTEGER, new_min_points),
        ])

        self.calibrated = True
        self.get_logger().info(
            f'[Calib] DONE → cluster_eps={new_eps:.3f}m '
            f'(noise p{pct:.0f}={noise_scale:.3f}m x{margin}), '
            f'cluster_min_points={new_min_points} '
            f'(median local density={median_density:.1f})')

        self.calib_nn_dists = []
        self.calib_density_counts = []

    # ──────────────────────────────────────────────────────
    # bbox 포인트 복원 (배경차감 이전 클라우드에서 재수집)
    # ──────────────────────────────────────────────────────
    def _on_refetch_cloud(self, msg):
        pts = read_points(msg)
        if len(pts) == 0:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        self._refetch_buffer.append((now, pts))

    def _refetch_snapshot(self):
        """최근 refetch_accumulate_frames개(그리고 max_age 이내) 프레임을
        하나로 합친 누적 클라우드. 콜백당 한 번만 만들어 재사용한다.
        사용할 프레임이 없으면 None."""
        if not self.get_parameter('refetch_enabled').value:
            return None
        if not self._refetch_buffer:
            return None
        n_frames = max(1, int(self.get_parameter('refetch_accumulate_frames').value))
        max_age = float(self.get_parameter('refetch_max_age_sec').value)
        # 누적 시에는 "가장 오래 허용되는 프레임"이 N×주기만큼 과거이므로
        # max_age를 프레임 수에 비례해 늘려 판정한다.
        now = self.get_clock().now().nanoseconds / 1e9
        frames = [pts for (t, pts) in list(self._refetch_buffer)[-n_frames:]
                  if (now - t) <= max_age * n_frames]
        if not frames:
            return None
        if len(frames) == 1:
            return frames[0]
        return np.vstack(frames)

    def _voxel_dedup(self, points, voxel_size):
        if voxel_size <= 0 or len(points) == 0:
            return points
        voxel_idx = np.floor(points / voxel_size).astype(np.int32)
        _, unique_idx = np.unique(voxel_idx, axis=0, return_index=True)
        return points[unique_idx]

    def _refetch_cluster(self, cluster, raw):
        """검출된 클러스터의 bbox(+margin) 안 포인트를 누적 원본 클라우드에서
        다시 가져온다. 누적으로 인한 중복은 복셀 dedupe로 정리한다.
        원본이 없거나 오히려 점이 줄면 클러스터 그대로 반환 (안전 fallback)."""
        if raw is None:
            return cluster

        margin = float(self.get_parameter('refetch_margin').value)
        mn = cluster.min(axis=0) - margin
        mx = cluster.max(axis=0) + margin
        in_box = np.all((raw >= mn) & (raw <= mx), axis=1)
        restored = raw[in_box]
        restored = self._voxel_dedup(
            restored, float(self.get_parameter('refetch_dedup_voxel').value))
        # 복원 결과가 기존보다 빈약하면(타이밍 어긋남 등) 원래 클러스터 유지
        if len(restored) <= len(cluster):
            return cluster
        return restored

    # ──────────────────────────────────────────────────────
    # DBSCAN (Pi5 최적화: 일괄 이웃 질의 + 불리언 집합)
    # ──────────────────────────────────────────────────────
    def dbscan(self, points, eps, min_points):
        n = len(points)
        labels = -np.ones(n, dtype=int)
        visited = np.zeros(n, dtype=bool)
        tree = KDTree(points)
        # 이웃 리스트 일괄 계산 (점별 query보다 훨씬 빠름)
        neighbors_all = tree.query_ball_point(points, eps)

        cluster_id = 0
        for i in range(n):
            if visited[i]:
                continue
            visited[i] = True
            neigh = neighbors_all[i]
            if len(neigh) < min_points:
                continue  # 노이즈 (labels[i]는 -1 유지)

            labels[i] = cluster_id
            seed = list(neigh)
            in_seed = np.zeros(n, dtype=bool)
            in_seed[neigh] = True

            j = 0
            while j < len(seed):
                q = seed[j]
                if not visited[q]:
                    visited[q] = True
                    nq = neighbors_all[q]
                    if len(nq) >= min_points:
                        for x in nq:
                            if not in_seed[x]:
                                in_seed[x] = True
                                seed.append(x)
                if labels[q] == -1:
                    labels[q] = cluster_id
                j += 1

            cluster_id += 1

        return labels

    # ──────────────────────────────────────────────────────
    # PCA 형태 특징
    # ──────────────────────────────────────────────────────
    def compute_pca_features(self, cluster):
        """공분산 고유분해 기반 형태 특징.
        verticality: 주축과 z축 사이 각의 |cos| (1=수직으로 섬, 0=수평으로 누움)
        planarity  : (λ1-λ0)/λ2 (1에 가까울수록 얇은 판 형태)
        """
        centered = cluster - cluster.mean(axis=0)
        cov = np.cov(centered.T)
        eigvals, eigvecs = np.linalg.eigh(cov)
        eigvals = np.clip(eigvals, 0.0, None)

        lam0, lam1, lam2 = eigvals
        denom = lam2 if lam2 > 1e-9 else 1e-9

        principal_axis = eigvecs[:, 2]
        cos_angle = float(abs(principal_axis[2]))
        verticality = cos_angle
        planarity = float((lam1 - lam0) / denom)
        return verticality, planarity, cos_angle

    # ──────────────────────────────────────────────────────
    # 사람 크기/형태 필터 (서있음·앉음 + 누움 2중 모드)
    # ──────────────────────────────────────────────────────
    def is_human(self, cluster):
        """반환: (is_human, (verticality, planarity, cos_angle), posture)
        posture: 'standing' | 'lying' | None
        """
        if len(cluster) < self.get_parameter('person_points_min').value:
            return False, (None, None, None), None

        min_pt = cluster.min(axis=0)
        max_pt = cluster.max(axis=0)
        dx = max_pt[0] - min_pt[0]
        dy = max_pt[1] - min_pt[1]
        dz = max_pt[2] - min_pt[2]

        h_min = self.get_parameter('person_height_min').value
        h_max = self.get_parameter('person_height_max').value
        w_min = self.get_parameter('person_width_min').value
        w_max = self.get_parameter('person_width_max').value

        posture = None
        if h_min <= dz < h_max:
            # 서있음/앉음 후보
            if w_min < dx < w_max and w_min < dy < w_max:
                posture = 'standing'
        elif self.get_parameter('lying_height_min').value <= dz < h_min:
            # 누움 후보: 긴 축(몸 길이) / 짧은 축(몸 폭) 검사
            major = max(dx, dy)
            minor = min(dx, dy)
            maj_min = self.get_parameter('lying_major_min').value
            maj_max = self.get_parameter('lying_major_max').value
            minor_max = self.get_parameter('lying_minor_max').value
            if maj_min < major < maj_max and w_min < minor < minor_max:
                posture = 'lying'

        if posture is None:
            return False, (None, None, None), None

        pca_min_pts = self.get_parameter('pca_min_points').value
        if len(cluster) < pca_min_pts:
            # 포인트 부족 → PCA 신뢰 불가, bbox 결과만으로 통과
            return True, (None, None, None), posture

        verticality, planarity, cos_angle = self.compute_pca_features(cluster)

        if posture == 'standing':
            shape_ok = (
                verticality >= self.get_parameter('person_verticality_min').value and
                planarity <= self.get_parameter('person_planarity_max').value
            )
        else:  # lying — 주축이 수평인 게 정상, 얇은 판(가구)만 배제
            shape_ok = (
                verticality <= self.get_parameter('lying_verticality_max').value and
                planarity <= self.get_parameter('lying_planarity_max').value
            )

        return shape_ok, (verticality, planarity, cos_angle), posture

    # ──────────────────────────────────────────────────────
    # BBox 마커 생성
    # ──────────────────────────────────────────────────────
    def make_bbox_marker(self, cluster, marker_id, header,
                         shape_feats=(None, None, None), posture='standing'):
        min_pt = cluster.min(axis=0)
        max_pt = cluster.max(axis=0)
        center = (min_pt + max_pt) / 2.0
        dx = max_pt[0] - min_pt[0]
        dy = max_pt[1] - min_pt[1]
        dz = max_pt[2] - min_pt[2]

        # 낙상 '판정'은 fall_detection_node 전담. 여기서는 자세를 색으로만
        # 구분한다 (서있음=초록, 누움=주황 — RViz 디버깅용).
        if posture == 'lying':
            color = ColorRGBA(r=1.0, g=0.55, b=0.0, a=0.5)
        else:
            color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=0.4)

        bbox = Marker()
        bbox.header = header
        bbox.ns = 'human_bbox'
        bbox.id = marker_id
        bbox.type = Marker.CUBE
        bbox.action = Marker.ADD
        bbox.pose.position.x = float(center[0])
        bbox.pose.position.y = float(center[1])
        bbox.pose.position.z = float(center[2])
        bbox.pose.orientation.w = 1.0
        bbox.scale.x = float(max(dx, 0.01))
        bbox.scale.y = float(max(dy, 0.01))
        bbox.scale.z = float(max(dz, 0.01))
        bbox.color = color
        bbox.lifetime.sec = 1

        text = Marker()
        text.header = header
        text.ns = 'human_text'
        text.id = marker_id + 100000
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = float(center[0])
        text.pose.position.y = float(center[1])
        text.pose.position.z = float(max_pt[2]) + 0.2
        text.pose.orientation.w = 1.0
        text.scale.z = 0.2
        text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)

        tag = 'LYING ' if posture == 'lying' else ''
        verticality, planarity, _ = shape_feats
        if verticality is not None:
            text.text = (f'P{marker_id} {tag}H:{dz:.1f}m W:{dx:.1f}m '
                         f'V:{verticality:.2f} P:{planarity:.2f}')
        else:
            text.text = f'P{marker_id} {tag}H:{dz:.1f}m W:{dx:.1f}m'
        text.lifetime.sec = 1

        return bbox, text

    # ──────────────────────────────────────────────────────
    # 메인 콜백
    # ──────────────────────────────────────────────────────
    def callback(self, msg):
        points = read_points(msg)
        if len(points) < 10:
            return

        # 부팅타임 캘리브레이션 중이면 통계만 누적
        if self.calib_enabled and not self.calibrated:
            self._accumulate_calibration(points)
            return

        eps = self.get_parameter('cluster_eps').value
        min_pts = self.get_parameter('cluster_min_points').value
        labels = self.dbscan(points, eps, min_pts)

        clusters = []
        for label in set(labels):
            if label < 0:
                continue
            clusters.append(points[labels == label])

        if not clusters:
            return

        # 사람 크기/형태 필터
        human_clusters = []
        human_feats = []
        human_postures = []
        for c in clusters:
            ok, feats, posture = self.is_human(c)
            if ok:
                human_clusters.append(c)
                human_feats.append(feats)
                human_postures.append(posture)

        if not human_clusters:
            return

        n_lying = sum(1 for p in human_postures if p == 'lying')
        self.get_logger().info(
            f'Detected {len(human_clusters)} person(s)'
            + (f' ({n_lying} lying)' if n_lying else ''))

        # bbox 중심/크기 계산 (트래킹 + 퍼블리시 재사용)
        centers = []
        sizes = []
        for c in human_clusters:
            min_pt = c.min(axis=0)
            max_pt = c.max(axis=0)
            centers.append((min_pt + max_pt) / 2.0)
            sizes.append(max_pt - min_pt)

        track_ids = self.tracker.update(centers)

        # 사람 포인트클라우드 — 배경차감으로 깎인 팔다리를 원본(배경차감 이전)
        # 최근 N프레임 누적 클라우드에서 bbox 영역 재수집으로 복원해 발행한다.
        raw_snapshot = self._refetch_snapshot()
        display_clusters = [self._refetch_cluster(c, raw_snapshot)
                            for c in human_clusters]
        all_points = np.vstack(display_clusters)
        self.pub_cloud.publish(pc2.create_cloud_xyz32(msg.header, all_points))

        # /human_tracks (후단 fall_detection 등, 포맷 유지)
        tracks_msg = Float32MultiArray()
        tracks_data = []
        for tid, center, size, feats in zip(track_ids, centers, sizes, human_feats):
            verticality, planarity, _ = feats
            v_out = float(verticality) if verticality is not None else -1.0
            p_out = float(planarity) if planarity is not None else -1.0
            tracks_data += [
                float(tid),
                float(center[0]), float(center[1]), float(center[2]),
                float(size[0]), float(size[1]), float(size[2]),
                v_out, p_out,
            ]
        tracks_msg.data = tracks_data
        self.pub_tracks.publish(tracks_msg)

        # /protected_regions (기존 소비자 호환)
        regions_msg = Float32MultiArray()
        regions_data = []
        for tid, center, size in zip(track_ids, centers, sizes):
            regions_data += [
                float(center[0]), float(center[1]), float(center[2]),
                float(size[0]), float(size[1]), float(size[2]),
                float(tid),
            ]
        regions_msg.data = regions_data
        self.pub_protected_regions.publish(regions_msg)

        # BBox 마커
        marker_array = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        marker_array.markers.append(clear)

        for tid, cluster, feats, posture in zip(
                track_ids, human_clusters, human_feats, human_postures):
            bbox, text = self.make_bbox_marker(
                cluster, tid, msg.header, shape_feats=feats, posture=posture)
            marker_array.markers.append(bbox)
            marker_array.markers.append(text)

        self.pub_bbox.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = HumanBBoxNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


