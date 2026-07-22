"""
r_calibration_node.py
─────────────────────────
목적:
  라이다를 설치 위치에 고정한 뒤, 방을 비운 상태(사람 없음)에서
  일정 시간 정지 캘리브레이션을 돌려서 "이 설치 위치에서 실제로
  정적 물체가 얼마나 흔들려 찍히는가"를 거리 구간별로 측정하고,
  human_bbox_node의 칼만필터 R(measure_noise)에 쓸 수 있는
  값을 YAML로 저장한다.

  라이다 위치/환경이 바뀌면(설치 위치 이동, 방 구조 변경 등) 이 노드를
  다시 돌려서 재보정하면 된다. 즉 R은 센서 고유 상수가 아니라
  "이 위치에서 실측한 값"으로 취급한다.

동작 방식:
  1. /filtered_cloud를 구독해서 human_bbox_node와 동일한 방식으로
     DBSCAN 클러스터링 (사람 크기 필터는 적용하지 않음 — 캘리브레이션
     중에는 사람이 없다는 전제이므로, 잡히는 모든 정적 클러스터가
     "노이즈/가구 표면" 샘플임).
  2. 프레임 간 클러스터를 단순 최근접 매칭으로 이어붙여
     "같은 물체가 프레임마다 centroid가 얼마나 흔들리는지" 누적.
  3. calibration_duration_sec 이 지나면, 각 blob의 위치 표준편차를
     계산하고, 라이다로부터의 평균 거리 기준으로 구간(distance bin)에
     묶어서 구간별 대표 표준편차(sigma)를 산출.
  4. 결과를 output_path (기본 ~/r_calibration.yaml)에 저장하고 종료.

주의:
  - 캘리브레이션 중에는 반드시 방에 사람이 없어야 한다. 사람이
    지나가면 그 클러스터도 "정적 노이즈"로 잘못 섞여 들어가서
    sigma가 과대 추정된다.
  - 설치 위치를 옮기면 반드시 재실행해서 새 YAML을 만들어야 한다.
"""

import time
import datetime

import numpy as np
import yaml
from scipy.spatial import KDTree

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2


# ════════════════════════════════════════════════════════
# human_bbox_node.py 와 동일한 DBSCAN (독립 실행을 위해 중복 포함)
# ════════════════════════════════════════════════════════
def read_points(msg: PointCloud2) -> np.ndarray:
    pts = pc2.read_points_numpy(msg, field_names=("x", "y", "z"), skip_nans=True)
    if pts is None or len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    if pts.dtype.names:
        pts = np.column_stack([pts['x'], pts['y'], pts['z']])
    return pts.astype(np.float32)


def dbscan(points: np.ndarray, eps: float, min_pts: int) -> np.ndarray:
    n = len(points)
    labels = -np.ones(n, dtype=int)
    visited = np.zeros(n, dtype=bool)
    tree = KDTree(points)
    cid = 0

    for i in range(n):
        if visited[i]:
            continue
        visited[i] = True
        nbrs = tree.query_ball_point(points[i], eps)

        if len(nbrs) < min_pts:
            continue

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


def extract_centroids(points: np.ndarray, labels: np.ndarray,
                       max_clusters: int = 50) -> list[np.ndarray]:
    unique = [l for l in set(labels) if l >= 0][:max_clusters]
    return [points[labels == l].mean(axis=0) for l in unique]


# ════════════════════════════════════════════════════════
# 정적 blob 추적 (Kalman 없이 단순 최근접 매칭만)
# ════════════════════════════════════════════════════════
class StaticBlob:
    __slots__ = ("last_pos", "positions")

    def __init__(self, pos: np.ndarray):
        self.last_pos = pos.copy()
        self.positions: list[np.ndarray] = [pos.copy()]

    def update(self, pos: np.ndarray):
        self.last_pos = pos.copy()
        self.positions.append(pos.copy())


class RCalibrationNode(Node):

    def __init__(self):
        super().__init__('r_calibration_node')

        # 클러스터링 (human_bbox_node와 동일 기본값 권장)
        self.declare_parameter('cluster_eps', 0.3)
        self.declare_parameter('cluster_min_points', 5)
        self.declare_parameter('max_clusters', 50)

        # blob 매칭 게이트 — 정적 물체라 사람보다 훨씬 좁게 잡음
        self.declare_parameter('match_gate_m', 0.5)
        # 이 프레임 수 이상 관측된 blob만 유효 샘플로 사용
        self.declare_parameter('min_samples_per_blob', 20)

        # 캘리브레이션 진행 시간(초)
        self.declare_parameter('calibration_duration_sec', 60.0)

        # 거리 구간 경계 (m). 예: [0,2,4,6,8,100] → 5구간
        self.declare_parameter(
            'distance_bin_edges', [0.0, 2.0, 4.0, 6.0, 8.0, 100.0])

        # 결과 저장 경로
        self.declare_parameter('output_path', '/tmp/r_calibration.yaml')

        self.p_eps = self.get_parameter('cluster_eps').value
        self.p_min_pts = self.get_parameter('cluster_min_points').value
        self.p_max_cls = self.get_parameter('max_clusters').value
        self.p_gate = self.get_parameter('match_gate_m').value
        self.p_min_samples = self.get_parameter('min_samples_per_blob').value
        self.p_duration = self.get_parameter('calibration_duration_sec').value
        self.p_bins = list(self.get_parameter('distance_bin_edges').value)
        self.p_output = self.get_parameter('output_path').value

        self._blobs: list[StaticBlob] = []
        self._start_time = time.monotonic()
        self._last_log_time = self._start_time
        self._n_frames = 0
        self._done = False

        self.sub = self.create_subscription(
            PointCloud2, '/filtered_cloud', self.callback, 10)

        self.get_logger().warn(
            '=== R 캘리브레이션 시작 ===\n'
            f'  ⚠ 지금부터 {self.p_duration:.0f}초 동안 방에 사람이 없어야 합니다.\n'
            f'  거리구간: {self.p_bins}\n'
            f'  결과 저장 위치: {self.p_output}'
        )

    # ────────────────────────────────────────────────────
    def callback(self, msg: PointCloud2):
        if self._done:
            return

        elapsed = time.monotonic() - self._start_time
        if elapsed >= self.p_duration:
            self._finish()
            return

        if elapsed - (self._last_log_time - self._start_time) >= 10.0:
            remaining = self.p_duration - elapsed
            self.get_logger().info(
                f'캘리브레이션 진행 중... 남은 시간 {remaining:.0f}초 '
                f'(누적 blob {len(self._blobs)}개)')
            self._last_log_time = time.monotonic()

        points = read_points(msg)
        if len(points) < 10:
            return
        self._n_frames += 1

        labels = dbscan(points, self.p_eps, self.p_min_pts)
        centroids = extract_centroids(points, labels, self.p_max_cls)

        used_blob = set()
        for c in centroids:
            best_i, best_d = None, self.p_gate
            for i, b in enumerate(self._blobs):
                if i in used_blob:
                    continue
                d = float(np.linalg.norm(c - b.last_pos))
                if d < best_d:
                    best_d, best_i = d, i
            if best_i is not None:
                self._blobs[best_i].update(c)
                used_blob.add(best_i)
            else:
                self._blobs.append(StaticBlob(c))

    # ────────────────────────────────────────────────────
    def _finish(self):
        self._done = True
        self.get_logger().warn('캘리브레이션 시간 종료. 결과 계산 중...')

        valid_blobs = [b for b in self._blobs
                       if len(b.positions) >= self.p_min_samples]

        if not valid_blobs:
            self.get_logger().error(
                '유효한 blob이 하나도 없습니다. '
                '(min_samples_per_blob 조건을 만족하는 정적 클러스터 없음) '
                '방이 너무 비어있거나(반사체 없음) eps/min_points 설정을 '
                '확인하세요. YAML을 저장하지 않고 종료합니다.')
            self._shutdown()
            return

        n_bins = len(self.p_bins) - 1
        bin_sigmas: list[list[float]] = [[] for _ in range(n_bins)]

        for b in valid_blobs:
            arr = np.array(b.positions)                      # (N,3)
            mean_dist = float(np.linalg.norm(arr[:, :2].mean(axis=0)))
            # 물체 중심 대비 편차의 표준편차 (3축 평균)
            sigma = float(np.mean(np.std(arr, axis=0)))

            bin_idx = np.digitize(mean_dist, self.p_bins) - 1
            if 0 <= bin_idx < n_bins:
                bin_sigmas[bin_idx].append(sigma)

        sigma_per_bin = []
        for i, vals in enumerate(bin_sigmas):
            if vals:
                sigma_per_bin.append(round(float(np.median(vals)), 4))
            else:
                sigma_per_bin.append(None)  # 이 구간엔 샘플이 없었음
                lo, hi = self.p_bins[i], self.p_bins[i + 1]
                self.get_logger().warn(
                    f'거리구간 [{lo:.1f}, {hi:.1f})m 에 샘플이 없습니다. '
                    f'이 구간은 캘리브레이션 결과에서 null로 남습니다.')

        result = {
            'r_calibration': {
                'distance_bin_edges': self.p_bins,
                'sigma_m': sigma_per_bin,
                'note': 'R(measure_noise) = sigma_m 값 그대로 사용 '
                        '(KalmanParams.measure_noise). '
                        'R_matrix = eye(3) * sigma_m**2',
                'calibrated_at': datetime.datetime.now().isoformat(),
                'num_blobs_total': len(self._blobs),
                'num_blobs_valid': len(valid_blobs),
                'num_frames': self._n_frames,
                'duration_sec': self.p_duration,
            }
        }

        with open(self.p_output, 'w') as f:
            yaml.dump(result, f, allow_unicode=True, sort_keys=False)

        self.get_logger().warn(
            f'=== 캘리브레이션 완료 ===\n'
            f'  유효 blob: {len(valid_blobs)}/{len(self._blobs)}\n'
            f'  구간별 sigma(m): {sigma_per_bin}\n'
            f'  저장 위치: {self.p_output}\n'
            f'  (구간에 null이 있으면 해당 거리대 표본이 부족했다는 뜻 — '
            f'가구를 그 거리에 두고 재측정하거나 인접 구간 값으로 대체하세요)'
        )
        self._shutdown()

    def _shutdown(self):
        # 콜백 안에서 바로 destroy_node/shutdown 하면 executor가 불안정할 수
        # 있어서, 타이머로 한 틱 뒤에 종료시킨다.
        self.create_timer(0.5, self._do_shutdown)

    def _do_shutdown(self):
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = RCalibrationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
