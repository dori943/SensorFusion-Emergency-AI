"""
human_bbox_node_final.py
─────────────────────────
파이프라인:
  /filtered_cloud (전처리 완료 포인트클라우드)
      ↓ DBSCAN 클러스터링
      ↓ 크기 필터 (height / width / depth)
      ↓ Kalman Filter (위치 스무딩 + 속도 추정 + 오탐 억제)
      ↓
  /human_cloud   PointCloud2
  /human_bbox    MarkerArray (wireframe bbox + 속도 화살표 + 텍스트)
"""

# ════════════════════════════════════════════════════════
# 의존 라이브러리
# ════════════════════════════════════════════════════════
import numpy as np
from collections import deque
from dataclasses import dataclass
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import ColorRGBA, Float32MultiArray
from geometry_msgs.msg import Point
import sensor_msgs_py.point_cloud2 as pc2


# ════════════════════════════════════════════════════════
# 유틸
# ════════════════════════════════════════════════════════
def read_points(msg: PointCloud2) -> np.ndarray:
    """PointCloud2 → (N, 3) float32"""
    pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    if pts is None or len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    # ROS 버전에 따라 structured array(dtype에 field명 있음)로 반환될 수 있음
    if pts.dtype.names:
        pts = np.column_stack([pts['x'], pts['y'], pts['z']])
    return pts.astype(np.float32)


# ════════════════════════════════════════════════════════
# DBSCAN
# ════════════════════════════════════════════════════════
def dbscan(points: np.ndarray, eps: float, min_pts: int) -> np.ndarray:
    """
    순수 numpy + scipy KDTree DBSCAN.
    Returns: labels (N,) — 노이즈=-1, 클러스터=0~
    """
    n = len(points)
    labels  = -np.ones(n, dtype=int)
    visited = np.zeros(n, dtype=bool)
    tree    = KDTree(points)
    cid     = 0

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        nbrs = tree.query_ball_point(points[i], eps)

        if len(nbrs) < min_pts:
            continue  # 노이즈

        labels[i] = cid
        seed_set: set[int] = set(nbrs)
        seed_set.discard(i)
        queue = list(seed_set)

        qi = 0
        while qi < len(queue):
            q = queue[qi]; qi += 1
            if not visited[q]:
                visited[q] = True
                new_nbrs = tree.query_ball_point(points[q], eps)
                if len(new_nbrs) >= min_pts:
                    for x in new_nbrs:
                        if x not in seed_set:
                            seed_set.add(x)
                            queue.append(x)
            if labels[q] == -1:
                labels[q] = cid

        cid += 1
    return labels


def extract_clusters(points: np.ndarray, labels: np.ndarray,
                     max_clusters: int = 50) -> list[np.ndarray]:
    unique = [l for l in set(labels) if l >= 0][:max_clusters]
    return [points[labels == l] for l in unique]


# ════════════════════════════════════════════════════════
# BBox
# ════════════════════════════════════════════════════════
@dataclass
class BBoxParams:
    height_min: float = 0.5
    height_max: float = 2.2
    width_min:  float = 0.2
    width_max:  float = 1.2
    depth_min:  float = 0.2
    depth_max:  float = 1.2


@dataclass
class BBoxInfo:
    center:      np.ndarray  # (3,)
    min_pt:      np.ndarray  # (3,)
    max_pt:      np.ndarray  # (3,)
    dx: float
    dy: float
    dz: float
    point_count: int


def compute_bbox(cluster: np.ndarray) -> BBoxInfo:
    mn = cluster.min(axis=0)
    mx = cluster.max(axis=0)
    dx, dy, dz = mx - mn
    return BBoxInfo(
        center=(mn + mx) / 2.0,
        min_pt=mn, max_pt=mx,
        dx=float(dx), dy=float(dy), dz=float(dz),
        point_count=len(cluster),
    )


def is_human(bi: BBoxInfo, p: BBoxParams) -> bool:
    return (
        p.height_min < bi.dz < p.height_max and
        p.width_min  < bi.dx < p.width_max  and
        p.depth_min  < bi.dy < p.depth_max
    )


# ════════════════════════════════════════════════════════
# Kalman Filter 트래커
# ════════════════════════════════════════════════════════
@dataclass
class KalmanParams:
    process_noise_pos:  float = 0.1   # 위치 프로세스 노이즈 (m)
    process_noise_vel:  float = 1.0   # 속도 프로세스 노이즈 (m/s)
    measure_noise:      float = 0.3   # LiDAR 관측 노이즈 (m)
    max_missed_frames:  int   = 5     # N프레임 미감지 시 트랙 삭제
    min_confirm_frames: int   = 3     # N프레임 연속 감지 시 확정 (오탐 억제)
    max_speed:          float = 3.0   # 최대 보행 속도 (m/s)
    match_gate:         float = 1.5   # 매칭 최대 거리 (m) — R 캘리브레이션에 맞춰 조정

    # ── 슬라이딩 윈도우 (사람 vs 정적/일시적 움직임 객체 구분) ──
    sw_window_size:      int   = 15    # 윈도우 길이 (프레임). kf_dt=0.1 → 1.5초
    sw_min_avg_speed:    float = 0.15  # 윈도우 평균 속력 최소값 (m/s)
    sw_active_speed_thr: float = 0.10  # "움직였다"고 판단할 프레임당 속력 임계값 (m/s)
    sw_min_active_ratio: float = 0.5   # 윈도우 내 "움직인 프레임" 비율 최소값
    sw_min_displacement: float = 0.3   # 윈도우 시작-끝 순변위 최소값 (m) — 제자리 흔들림 배제


class KalmanTrack:
    """
    단일 트랙.
    상태 벡터: [x, y, z, vx, vy, vz]
    관측 벡터: [x, y, z]  (centroid)
    """

    def __init__(self, tid: int, init_pos: np.ndarray,
                 dt: float, p: KalmanParams):
        self.track_id = tid
        self.p        = p

        # 상태
        self.x = np.array([*init_pos, 0., 0., 0.], dtype=float)

        # 상태 전이 행렬 F (등속 운동 모델)
        self.F      = np.eye(6)
        self.F[0,3] = self.F[1,4] = self.F[2,5] = dt

        # 관측 행렬 H: 위치만 추출
        self.H      = np.zeros((3, 6))
        self.H[0,0] = self.H[1,1] = self.H[2,2] = 1.0

        # 공분산
        self.P = np.eye(6)
        q_p    = p.process_noise_pos ** 2
        q_v    = p.process_noise_vel ** 2
        self.Q = np.diag([q_p, q_p, q_p, q_v, q_v, q_v])
        r      = p.measure_noise ** 2
        self.R = np.eye(3) * r

        # 트랙 상태
        self.missed_frames  = 0
        self.confirm_frames = 1
        self.is_confirmed   = False

        # bbox 크기 EMA 스무딩
        self.smoothed_size: np.ndarray | None = None
        self._alpha = 0.4  # 낮을수록 더 많이 스무딩

        # ── 슬라이딩 윈도우 히스토리 (사람=지속적 움직임 판별용) ──
        # update()가 호출될 때(=실제 관측 매칭)마다 위치/속력을 기록.
        # missed 프레임에는 기록하지 않음 → "관측된 움직임"만 평가.
        self._pos_hist:   deque[np.ndarray] = deque(maxlen=p.sw_window_size)
        self._speed_hist: deque[float]      = deque(maxlen=p.sw_window_size)
        self._pos_hist.append(init_pos.copy())

    def predict(self, dt: float | None = None):
        if dt is not None:
            self.F[0,3] = self.F[1,4] = self.F[2,5] = dt
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

    def update(self, pos: np.ndarray, size: np.ndarray):
        z   = pos.reshape(3, 1)
        y   = z - (self.H @ self.x).reshape(3, 1)        # 혁신
        S   = self.H @ self.P @ self.H.T + self.R        # 혁신 공분산
        K   = self.P @ self.H.T @ np.linalg.inv(S)       # 칼만 게인
        self.x = self.x + (K @ y).flatten()
        I_KH   = np.eye(6) - K @ self.H
        self.P = I_KH @ self.P @ I_KH.T + K @ self.R @ K.T  # Joseph form

        self.missed_frames   = 0
        self.confirm_frames += 1
        if self.confirm_frames >= self.p.min_confirm_frames:
            self.is_confirmed = True

        if self.smoothed_size is None:
            self.smoothed_size = size.copy()
        else:
            self.smoothed_size = self._alpha * size + (1 - self._alpha) * self.smoothed_size

        # 슬라이딩 윈도우 기록 (관측된 프레임만)
        self._pos_hist.append(self.x[:3].copy())
        self._speed_hist.append(self.speed)

    def mark_missed(self):
        self.missed_frames += 1

    @property
    def position(self) -> np.ndarray:
        return self.x[:3].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[3:].copy()

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.x[3:5]))  # 수평 속도

    @property
    def is_alive(self) -> bool:
        return self.missed_frames <= self.p.max_missed_frames

    @property
    def is_valid_speed(self) -> bool:
        return self.speed <= self.p.max_speed

    @property
    def is_moving(self) -> bool:
        """
        슬라이딩 윈도우 기반 '지속적 움직임' 판별.
        의자 등 정적 객체가 한두 프레임 흔들려 트랙이 생성되더라도,
        윈도우가 충분히 쌓이기 전까지는 보수적으로 통과시키고
        (min_confirm_frames 가 1차 방어선), 윈도우가 다 차면
        아래 3개 조건을 모두 만족해야 '사람처럼 움직인다'고 판단한다.
          1) 평균 속력이 임계값 이상
          2) 윈도우 내 '활동(속력>임계값) 프레임' 비율이 임계값 이상
          3) 윈도우 시작~끝 순변위가 임계값 이상 (제자리 진동/노이즈 배제)
        """
        p = self.p
        n = len(self._speed_hist)
        if n < p.sw_window_size:
            # 윈도우가 아직 안 찼으면 판단을 보류하고 통과
            # (너무 빨리 죽이면 신규 사람 트랙까지 잘릴 수 있음)
            return True

        avg_speed = float(np.mean(self._speed_hist))
        if avg_speed < p.sw_min_avg_speed:
            return False

        active_ratio = float(np.mean(
            [s > p.sw_active_speed_thr for s in self._speed_hist]))
        if active_ratio < p.sw_min_active_ratio:
            return False

        net_disp = float(np.linalg.norm(
            self._pos_hist[-1][:2] - self._pos_hist[0][:2]))  # xy 평면 기준
        if net_disp < p.sw_min_displacement:
            return False

        return True


class KalmanTracker:
    """
    멀티 트랙 관리.
    매 프레임: predict → nearest-centroid 매칭 → update / mark_missed → 트랙 정리
    """

    def __init__(self, dt: float, params: KalmanParams):
        self.dt      = dt
        self.params  = params
        self._tracks: dict[int, KalmanTrack] = {}
        self._nid    = 0
        self._MATCH  = params.match_gate  # 매칭 최대 거리 (m), 파라미터화됨

    def update(self, detections: list[tuple[np.ndarray, np.ndarray]]
               ) -> list[dict]:
        """
        detections: [(centroid(3,), bbox_size(3,)), ...]
        returns:    [{'track_id', 'position', 'velocity', 'speed',
                      'smoothed_size', 'is_confirmed'}, ...]
        """
        # 1. 예측
        for t in self._tracks.values():
            t.predict()

        # 2. nearest-centroid 매칭
        matched, unmatched_dets, unmatched_trks = self._match(detections)

        # 3. 업데이트
        for di, tid in matched:
            self._tracks[tid].update(*detections[di])

        # 4. 미매칭 트랙 missed 처리
        for tid in unmatched_trks:
            self._tracks[tid].mark_missed()

        # 5. 새 트랙 생성
        for di in unmatched_dets:
            t = KalmanTrack(self._nid, detections[di][0], self.dt, self.params)
            t.smoothed_size = detections[di][1].copy()
            self._tracks[self._nid] = t
            self._nid += 1

        # 6. 죽은 트랙 삭제
        for tid in [tid for tid, t in self._tracks.items() if not t.is_alive]:
            del self._tracks[tid]

        # 7. confirmed + 속도 유효 + 슬라이딩 윈도우상 '지속 움직임' 트랙만 반환
        results = []
        for t in self._tracks.values():
            if not t.is_confirmed or not t.is_valid_speed:
                continue
            if not t.is_moving:
                continue
            sz = t.smoothed_size if t.smoothed_size is not None \
                 else np.array([0.5, 0.5, 1.7])
            results.append({
                'track_id'     : t.track_id,
                'position'     : t.position,
                'velocity'     : t.velocity,
                'speed'        : t.speed,
                'smoothed_size': sz,
            })
        return results

    def get_protected_regions(self) -> list[dict]:
        """
        배경 재학습 시 절대 배경으로 흡수되면 안 되는 영역을 반환.

        is_moving / is_valid_speed 와는 무관하게, '한 번이라도 사람으로
        확정된 적 있고(is_confirmed) 아직 살아있는(is_alive) 트랙'은 전부
        포함한다. 이게 핵심: 낙상해서 멈춘 사람은 is_moving=False가 되어
        화면 표시(track_results)에서는 제외될 수 있어도, 배경 재학습
        보호 대상에서는 절대 빠지면 안 된다.
        """
        regions = []
        for t in self._tracks.values():
            if not t.is_confirmed or not t.is_alive:
                continue
            sz = t.smoothed_size if t.smoothed_size is not None \
                 else np.array([0.5, 0.5, 1.7])
            regions.append({
                'track_id': t.track_id,
                'position': t.position,
                'size'    : sz,
            })
        return regions

    def _match(self, detections):
        trk_ids  = list(self._tracks.keys())
        det_idxs = list(range(len(detections)))
        if not trk_ids or not det_idxs:
            return [], det_idxs, trk_ids

        matched, used_t, used_d = [], set(), set()
        for di in det_idxs:
            best_t, best_d = None, self._MATCH
            for ti in trk_ids:
                if ti in used_t:
                    continue
                d = float(np.linalg.norm(
                    detections[di][0] - self._tracks[ti].position))
                if d < best_d:
                    best_d, best_t = d, ti
            if best_t is not None:
                matched.append((di, best_t))
                used_t.add(best_t); used_d.add(di)

        return (matched,
                [i for i in det_idxs if i not in used_d],
                [t for t in trk_ids  if t not in used_t])


# ════════════════════════════════════════════════════════
# 마커 헬퍼
# ════════════════════════════════════════════════════════
def _wireframe(bi: BBoxInfo, mid: int, header, color: ColorRGBA) -> Marker:
    mn, mx = bi.min_pt, bi.max_pt
    corners = np.array([
        [mn[0],mn[1],mn[2]], [mx[0],mn[1],mn[2]],
        [mx[0],mx[1],mn[2]], [mn[0],mx[1],mn[2]],
        [mn[0],mn[1],mx[2]], [mx[0],mn[1],mx[2]],
        [mx[0],mx[1],mx[2]], [mn[0],mx[1],mx[2]],
    ])
    edges = [(0,1),(1,2),(2,3),(3,0),
             (4,5),(5,6),(6,7),(7,4),
             (0,4),(1,5),(2,6),(3,7)]
    m = Marker()
    m.header = header; m.ns = 'human_bbox'
    m.id = mid; m.type = Marker.LINE_LIST; m.action = Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x = 0.02; m.color = color; m.lifetime.sec = 1
    for a, b in edges:
        for idx in (a, b):
            p = Point()
            p.x, p.y, p.z = float(corners[idx][0]), float(corners[idx][1]), float(corners[idx][2])
            m.points.append(p)
    return m


def _velocity_arrow(tr: dict, mid: int, header) -> Marker:
    pos, vel = tr['position'], tr['velocity']
    m = Marker()
    m.header = header; m.ns = 'human_vel'
    m.id = mid + 30_000; m.type = Marker.ARROW; m.action = Marker.ADD
    m.points = [
        Point(x=float(pos[0]), y=float(pos[1]), z=float(pos[2])),
        Point(x=float(pos[0]+vel[0]), y=float(pos[1]+vel[1]), z=float(pos[2]+vel[2])),
    ]
    m.scale.x = 0.05; m.scale.y = 0.10; m.scale.z = 0.10
    m.color = ColorRGBA(r=1.0, g=1.0, b=0.0, a=0.9)
    m.lifetime.sec = 1
    return m


def _text(tr: dict, mid: int, header) -> Marker:
    pos, sz = tr['position'], tr['smoothed_size']
    m = Marker()
    m.header = header; m.ns = 'human_text'
    m.id = mid + 10_000; m.type = Marker.TEXT_VIEW_FACING; m.action = Marker.ADD
    m.pose.position.x = float(pos[0])
    m.pose.position.y = float(pos[1])
    m.pose.position.z = float(pos[2] + sz[2] / 2 + 0.2)
    m.pose.orientation.w = 1.0
    m.scale.z = 0.2
    m.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
    m.text = (f"T{tr['track_id']}\n"
              f"H:{sz[2]:.2f}m  W:{sz[0]:.2f}m  D:{sz[1]:.2f}m\n"
              f"spd:{tr['speed']:.2f}m/s")
    m.lifetime.sec = 1
    return m


# ════════════════════════════════════════════════════════
# ROS2 노드
# ════════════════════════════════════════════════════════
class HumanBBoxNode(Node):

    def __init__(self):
        super().__init__('human_bbox_node')

        # 클러스터링
        self.declare_parameter('cluster_eps',        0.3)
        self.declare_parameter('cluster_min_points', 5)
        self.declare_parameter('max_clusters',       50)

        # 크기 필터
        self.declare_parameter('person_height_min', 0.5)
        self.declare_parameter('person_height_max', 2.2)
        self.declare_parameter('person_width_min',  0.2)
        self.declare_parameter('person_width_max',  1.2)
        self.declare_parameter('person_depth_min',  0.2)
        self.declare_parameter('person_depth_max',  1.2)

        # Kalman Filter
        self.declare_parameter('kf_dt',                0.1)
        self.declare_parameter('kf_process_noise_pos', 0.1)
        self.declare_parameter('kf_process_noise_vel', 1.0)
        self.declare_parameter('kf_measure_noise',     0.3)
        self.declare_parameter('kf_max_missed_frames', 5)
        self.declare_parameter('kf_min_confirm_frames',3)
        self.declare_parameter('kf_max_speed',         3.0)
        self.declare_parameter('kf_match_gate',        1.5)

        # 슬라이딩 윈도우 (사람 vs 정적/일시적 움직임 객체 구분)
        self.declare_parameter('sw_window_size',       15)
        self.declare_parameter('sw_min_avg_speed',     0.15)
        self.declare_parameter('sw_active_speed_thr',  0.10)
        self.declare_parameter('sw_min_active_ratio',  0.5)
        self.declare_parameter('sw_min_displacement',  0.3)

        # 시각화
        self.declare_parameter('show_velocity', True)  # 속도 화살표 on/off

        self._cache_params()

        self._tracker = KalmanTracker(
            dt=self.p_kf_dt, params=self.p_kf)

        self.sub       = self.create_subscription(
            PointCloud2, '/filtered_cloud', self.callback, 10)
        self.pub_cloud = self.create_publisher(PointCloud2, '/human_cloud', 10)
        self.pub_bbox  = self.create_publisher(MarkerArray, '/human_bbox',  10)
        self.pub_protect = self.create_publisher(
            Float32MultiArray, '/protected_regions', 10)

        self.get_logger().info('HumanBBoxNode started.')

    # ────────────────────────────────────────────────────
    def _cache_params(self):
        g = self.get_parameter
        self.p_eps     = g('cluster_eps').value
        self.p_min_pts = g('cluster_min_points').value
        self.p_max_cls = g('max_clusters').value
        self.p_bbox    = BBoxParams(
            height_min=g('person_height_min').value,
            height_max=g('person_height_max').value,
            width_min =g('person_width_min').value,
            width_max =g('person_width_max').value,
            depth_min =g('person_depth_min').value,
            depth_max =g('person_depth_max').value,
        )
        self.p_kf_dt   = g('kf_dt').value
        self.p_kf      = KalmanParams(
            process_noise_pos =g('kf_process_noise_pos').value,
            process_noise_vel =g('kf_process_noise_vel').value,
            measure_noise     =g('kf_measure_noise').value,
            max_missed_frames =g('kf_max_missed_frames').value,
            min_confirm_frames=g('kf_min_confirm_frames').value,
            max_speed         =g('kf_max_speed').value,
            match_gate        =g('kf_match_gate').value,
            sw_window_size      =g('sw_window_size').value,
            sw_min_avg_speed    =g('sw_min_avg_speed').value,
            sw_active_speed_thr =g('sw_active_speed_thr').value,
            sw_min_active_ratio =g('sw_min_active_ratio').value,
            sw_min_displacement =g('sw_min_displacement').value,
        )
        self.p_show_vel = g('show_velocity').value

    # ────────────────────────────────────────────────────
    def _publish_protected_regions(self, header):
        """
        confirmed 트랙(움직임 무관)을 '/protected_regions'로 퍼블리시.
        BackgroundSubtractionNode가 주기적 재학습 시 이 영역을
        배경 후보에서 제외해서, 낙상/장시간 정지 트랙이 배경으로
        흡수되지 않도록 보호한다.
        포맷: 7개씩 [cx, cy, cz, sx, sy, sz, track_id] 평탄화.
        """
        regions = self._tracker.get_protected_regions()
        msg = Float32MultiArray()
        flat: list[float] = []
        for r in regions:
            pos, sz = r['position'], r['size']
            flat.extend([
                float(pos[0]), float(pos[1]), float(pos[2]),
                float(sz[0]),  float(sz[1]),  float(sz[2]),
                float(r['track_id']),
            ])
        msg.data = flat
        self.pub_protect.publish(msg)

    # ────────────────────────────────────────────────────
    def callback(self, msg: PointCloud2):
        points = read_points(msg)
        if len(points) < 10:
            return

        # 1. 클러스터링
        labels   = dbscan(points, self.p_eps, self.p_min_pts)
        clusters = extract_clusters(points, labels, self.p_max_cls)

        # 2. 크기 필터
        detections: list[tuple[np.ndarray, np.ndarray]] = []
        human_clusters: list[np.ndarray] = []
        for cluster in clusters:
            bi = compute_bbox(cluster)
            if not is_human(bi, self.p_bbox):
                continue
            detections.append((bi.center, np.array([bi.dx, bi.dy, bi.dz])))
            human_clusters.append(cluster)

        # 3. Kalman Filter
        track_results = self._tracker.update(detections)

        # 3-1. 배경 재학습 보호 영역 퍼블리시
        #      (is_moving=False인 정지/낙상 트랙도 confirmed면 반드시 포함—
        #       track_results 가 비어도 이 퍼블리시는 항상 수행한다)
        self._publish_protected_regions(msg.header)

        if not track_results:
            return

        self.get_logger().info(
            f'Tracked {len(track_results)} person(s)  '
            f'(raw: {len(detections)})')

        # 4. 포인트클라우드 퍼블리시
        if human_clusters:
            self.pub_cloud.publish(
                pc2.create_cloud_xyz32(msg.header, np.vstack(human_clusters)))

        # 5. 마커 퍼블리시
        ma = MarkerArray()
        clear = Marker(); clear.header = msg.header
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        wf_color = ColorRGBA(r=0.0, g=1.0, b=0.2, a=1.0)

        for i, tr in enumerate(track_results):
            pos = tr['position']
            sz  = tr['smoothed_size']
            half = sz / 2.0
            bi = BBoxInfo(
                center=pos, min_pt=pos-half, max_pt=pos+half,
                dx=float(sz[0]), dy=float(sz[1]), dz=float(sz[2]),
                point_count=0,
            )
            ma.markers.append(_wireframe(bi, i, msg.header, wf_color))
            if self.p_show_vel:
                ma.markers.append(_velocity_arrow(tr, i, msg.header))
            ma.markers.append(_text(tr, i, msg.header))

        self.pub_bbox.publish(ma)


# ════════════════════════════════════════════════════════
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
