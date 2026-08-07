"""
fall_detection_node.py
─────────────────────────────
목적:
  human_bbox_node가 트랙별로 퍼블리시하는 /human_tracks
  ([track_id,cx,cy,cz,sx,sy,sz,verticality,planarity]*N)를 받아,
  트랙별 최근 이력(위치/속도/PCA 형태)을 버퍼에 쌓고 규칙 기반으로
  낙상을 판정한다.

  "낙상"을 하나의 순간 프레임 값(예: bbox ratio<0.8)만으로 판정하면
  원래 앉아있거나 엎드려 운동하는 사람도 오탐되기 쉽다. 그래서 이
  노드는 아래 신호를 시간축으로 함께 본다 (사람은 대부분
  "서 있거나 걷다가" 넘어지고, 넘어진 뒤엔 한동안 잘 움직이지 못한다는
  전제).

    1) 사전 기립 자세
       — 낙상 전에 verticality가 충분히 높게 유지되어 실제로 서 있었는가.
    2) 수평 속도 급증 또는 bbox 중심의 빠른 z축 하강
       — 걷다가 넘어지는 동작과 제자리에서 무너지는 동작을 함께 검출한다.
         속도는 최근 여러 구간의 중앙값을 사용해 트래킹 좌표 튐을 줄인다.
    3) PCA 주축(verticality) 급락
       — human_bbox_node가 계산한 verticality(주축-z축 cos)가
         "서 있음(1에 가까움)"에서 "누움(0에 가까움)"으로 짧은 시간
         안에 크게 떨어짐. 사람의 몸통 방향이 수직→수평으로
         바뀌었다는 가장 직접적인 자세 신호.
    4) bbox 높이(sz) 급감
       — 이전 높이들의 강건한 기준값 대비 bbox 세로 크기가 짧은 시간
         안에 크게 줄었는지 확인하는 보조 신호.

  1)+2)와 (3 또는 4) 조건이 겹치면 FALL_CANDIDATE로 올리고, 이후 PCA로
  일정 시간 "누운 자세 + 정지 상태"가 유지됐을 때 LiDAR 신뢰도가 높거나
  후보 시각 근처에 유효한 IMU 충격 증거가 있으면 FALL_CONFIRMED로 확정한다
  (단순 순간 휘청임/재빨리 앉기와 구분하기 위한 확인 지연).
  확정 뒤 사람이 다시 일어나 기립 자세를 유지하면 자동으로 NORMAL로 복귀한다.

입력:
  /human_tracks (std_msgs/Float32MultiArray)
      human_bbox_node가 퍼블리시.
      포맷: 9개씩 반복 [track_id,cx,cy,cz,sx,sy,sz,verticality,planarity]
      verticality/planarity가 -1.0이면 "PCA 미계산(정보 없음)"을 의미.

  /imu/impact_peak (std_msgs/String)
      IMU 낙상 증거 JSON. confidence는 융합 점수에 사용하고,
      modality_data.peak_ratio는 충격 임계값 통과 여부에 사용한다.

출력:
  /fall_events  (std_msgs/Float32MultiArray)
      상태가 CANDIDATE 이상인 트랙 + (protect_active_normal=True일 때만)
      낙상은 아니지만 최근 실제로 이동 중인(ACTIVE) 트랙을 담아 퍼블리시.
      포맷: 6개씩 반복 [track_id,cx,cy,cz,confidence,state_code]
      state_code: 1=CANDIDATE(의심), 2=CONFIRMED(확정),
                  3=ACTIVE(낙상 아님, 최근 실제 이동 중 — 배경 흡수 방지용,
                  protect_active_normal=True일 때만 발행),
                  4=RECOVERED(회복, 보호영역 즉시 해제용 1회 이벤트)
      해당 트랙이 없으면 빈 배열([]) 퍼블리시.
      (정지 상태인 NORMAL 트랙은 여기 포함되지 않는다 — 예를 들어
       사람으로 오분류된 정지 물체가 계속 배경 흡수를 면제받지
       않도록 하기 위함.)

      주의: bg_subtraction_node는 이 토픽의 각 트랙 주변 protection_radius
      이내 포인트를 배경 여부와 무관하게 전부 강제로 살려서 출력한다.
      ACTIVE(state_code=3)까지 여기 포함시키면, 정상적으로 걷는 사람 주변의
      진짜 배경(바닥/벽/가구)까지 그 사람이 지나갈 때마다 같이 되살아나
      "움직이는 물체 주변에 배경이 계속 다시 나타나는" 문제가 생긴다.
      정지 스트릭 판정이 반경 매칭 기반으로 바뀌어 계속 움직이는 물체는
      어차피 배경으로 편입되지 않으므로, ACTIVE 보호는 기본적으로 끄고
      (protect_active_normal=False) CANDIDATE/CONFIRMED만 보호한다.

  /fall_marker  (visualization_msgs/MarkerArray)
      CANDIDATE(주황) / CONFIRMED(빨강, 큰 원통+텍스트) 시각화 — rviz2 확인용.
"""

from collections import deque
import json

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, ColorRGBA, String
from visualization_msgs.msg import Marker, MarkerArray


STATE_NORMAL = 0
STATE_CANDIDATE = 1
STATE_CONFIRMED = 2
# 낙상 상태머신의 실제 state는 아니고, /fall_events 출력 전용 코드.
# NORMAL이지만 최근에 실제로 이동한 트랙(=배경 흡수 방지 대상)을 표시.
EVENT_CODE_ACTIVE_NORMAL = 3
EVENT_CODE_RECOVERED = 4


class TrackHistory:
    """트랙 하나의 최근 시계열(시각, 위치, 크기, PCA 형태)을 보관."""

    __slots__ = ("samples", "state", "state_since", "last_seen",
                 "candidate_confidence", "confirmed_confidence")

    def __init__(self, now_sec: float):
        self.samples = deque()   # (t, cx, cy, cz, sz, verticality)
        self.state = STATE_NORMAL
        self.state_since = now_sec      # 현재 state로 바뀐 시각
        self.last_seen = now_sec        # /human_tracks에 마지막으로 잡힌 시각 (트랙 청소용)
        self.candidate_confidence = 0.0
        self.confirmed_confidence = 0.0


class FallDetectionNode(Node):

    def __init__(self):
        super().__init__('fall_detection_node')

        # ── 입출력 토픽 ──
        self.declare_parameter('tracks_topic', '/human_tracks')
        self.declare_parameter('output_topic', '/fall_events')
        self.declare_parameter('marker_topic', '/fall_marker')
        self.declare_parameter('frame_id', 'unilidar_lidar')

        # ── IMU 융합 ──
        self.declare_parameter('imu_topic', '/imu/impact_peak')
        self.declare_parameter('imu_buffer_sec', 4.0)
        self.declare_parameter('imu_min_confidence', 0.7)
        self.declare_parameter('imu_min_peak_ratio', 1.0)
        self.declare_parameter('imu_before_candidate_sec', 1.5)
        self.declare_parameter('imu_after_candidate_sec', 0.3)
        self.declare_parameter('lidar_standalone_confidence', 0.85)
        self.declare_parameter('imu_confidence_weight', 0.25)

        # ── 이력 버퍼 ──
        self.declare_parameter('history_window_sec', 3.0)     # 트랙별로 보관할 최대 이력 길이

        # ── 1) 사전 이동 게이트 ──
        self.declare_parameter('motion_speed_threshold', 0.15)   # m/s, 이 이상이면 "움직였다"
        self.declare_parameter('motion_lookback_sec', 2.0)       # 이 기간 내 이동 여부를 확인

        # ── 2) 낙상성 속도 급증 ──
        self.declare_parameter('speed_spike_threshold', 0.8)     # m/s, 낙상성 순간 속도
        self.declare_parameter('speed_spike_lookback_sec', 1.2)  # 스파이크를 찾는 최근 창
        self.declare_parameter('speed_median_samples', 5)        # 순간 좌표 튐을 줄일 속도 중앙값 구간 수
        self.declare_parameter('vertical_fall_speed_threshold', 0.5)  # m/s, bbox 중심의 낙상성 하강 속도
        self.declare_parameter('confirm_vertical_speed_max', 0.15)    # m/s, 확정 시 z축 정지 속도 상한

        # ── 3) PCA verticality 급락 ──
        self.declare_parameter('verticality_drop_threshold', 0.35)  # 이 이상 떨어지면 "누움"
        self.declare_parameter('verticality_drop_window_sec', 1.2)  # 이 시간 안에 떨어져야 "급락"
        self.declare_parameter('verticality_lying_max', 0.45)       # 현재 verticality가 이 밑이면 "누운 상태"로 간주
        self.declare_parameter('standing_verticality_min', 0.65)    # 낙상 전에 이 이상이어야 서 있던 것으로 간주
        self.declare_parameter('standing_min_samples', 3)           # 서 있던 자세를 확인할 최소 유효 PCA 샘플 수

        # ── 4) bbox 높이(sz) 급감 (PCA 보조/대체) ──
        self.declare_parameter('height_drop_ratio', 0.4)      # 강건한 이전 sz 기준 대비 감소 비율
        self.declare_parameter('height_drop_window_sec', 1.2)

        # ── 상태 확정/해제 ──
        self.declare_parameter('candidate_timeout_sec', 4.0)     # 이 시간 내 확정 못 하면 오탐으로 보고 NORMAL 복귀
        self.declare_parameter('confirm_stillness_sec', 1.5)     # 누운+정지가 이만큼 유지되면 CONFIRMED
        self.declare_parameter('confirm_lying_ratio', 0.7)       # 확인 구간 중 누운 자세 최소 비율
        self.declare_parameter('confirm_speed_max', 0.2)         # 확인 구간의 허용 수평 변위 환산 속도 (m/s)
        self.declare_parameter('recovery_verticality_min', 0.6)  # 다시 이 이상 서면 회복 신호
        self.declare_parameter('recovery_hold_sec', 1.0)         # 회복 신호가 이만큼 유지돼야 NORMAL 복귀 (순간 오검출 방지)

        # ── 트랙 청소 ──
        self.declare_parameter('track_timeout_sec', 5.0)  # 이 시간 이상 /human_tracks에서 안 보이면 이력 삭제
        self.declare_parameter('track_reassociate_sec', 2.0)   # 낙상 중 ID 변경을 이어 붙일 최대 시간차
        self.declare_parameter('track_reassociate_dist', 1.0)  # 이전/새 트랙 중심의 최대 xy 거리

        # ── 진단 로그 ──
        self.declare_parameter('diagnostic_log_interval_sec', 1.0)

        # ── ACTIVE_NORMAL(정상 이동 중) 보호영역 발행 여부 ──
        # bg_subtraction_node는 이 토픽에 실린 트랙 주변 protection_radius(기본 1.0m)
        # 이내의 "모든" 포인트를 배경 여부와 무관하게 강제로 살려서 출력한다
        # (사람인지 아닌지 구분하지 않음). 그런데 걷고 있는 사람도 계속 이 토픽에
        # 실리면(state_code=3), 사람이 지나갈 때마다 주변 1m 안의 진짜 배경(바닥/벽/
        # 가구)까지 같이 되살아나 "객체가 움직이면 배경이 다시 나타난다"는 문제가
        # 생긴다. 정지 스트릭이 반경 매칭 + 30프레임 연속 조건으로 바뀌면서 계속
        # 움직이는 물체는 어차피 배경으로 절대 편입되지 않으므로, ACTIVE_NORMAL
        # 보호는 실익보다 부작용이 커서 기본값을 끈다. 필요하면 True로 켤 수 있다.
        self.declare_parameter('protect_active_normal', False)

        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.imu_topic = self.get_parameter('imu_topic').value

        self._histories: dict[int, TrackHistory] = {}
        self._track_aliases: dict[int, int] = {}  # raw track_id -> 고정 logical track_id
        self._recovery_since: dict[int, float] = {}  # CONFIRMED 트랙별 "회복 신호 지속 시작 시각"
        self._trigger_log_times: dict[int, float] = {}  # 후보 진입 실패 로그 제한용
        self._diagnostic_log_times: dict[int, float] = {}
        self._imu_evidence_buffer = deque()

        self.create_subscription(
            Float32MultiArray, self.tracks_topic, self._on_tracks, 10)
        self.create_subscription(
            String, self.imu_topic, self._on_imu_evidence, 10)

        self.pub_events = self.create_publisher(
            Float32MultiArray, self.output_topic, 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, self.marker_topic, 10)

        self.get_logger().info(
            f'FallDetectionNode 시작 | tracks={self.tracks_topic} | '
            f'imu={self.imu_topic} | '
            f'motion>={self.get_parameter("motion_speed_threshold").value}m/s '
            f'spike>={self.get_parameter("speed_spike_threshold").value}m/s '
            f'v_drop>={self.get_parameter("verticality_drop_threshold").value} '
            f'confirm={self.get_parameter("confirm_stillness_sec").value}s '
            f'protect_active_normal={self.get_parameter("protect_active_normal").value}'
        )

    # ────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_imu_evidence(self, msg: String):
        """공통 envelope 형식의 IMU 낙상 증거를 검증해 보관한다."""
        try:
            payload = json.loads(msg.data)
            if (payload.get('event_type') != 'fall_evidence'
                    or payload.get('modality') != 'imu'):
                return

            stamp = payload['ros_stamp']
            modality_data = payload['modality_data']
            sec = int(stamp['sec'])
            nanosec = int(stamp['nanosec'])
            confidence = float(payload['confidence'])
            peak_ratio = float(modality_data['peak_ratio'])
            boosted_score = float(modality_data['boosted_score'])
            acc_mag_g = float(modality_data['acc_mag_g'])
        except (json.JSONDecodeError, KeyError, TypeError, ValueError,
                AttributeError):
            self.get_logger().warning('잘못된 IMU fall_evidence JSON, 건너뜀')
            return

        values = (confidence, peak_ratio, boosted_score, acc_mag_g)
        if (nanosec < 0 or nanosec >= 1_000_000_000
                or not all(np.isfinite(value) for value in values)
                or confidence < 0.0 or confidence > 1.0
                or peak_ratio < 0.0):
            self.get_logger().warning('범위를 벗어난 IMU fall_evidence, 건너뜀')
            return

        self._imu_evidence_buffer.append({
            't_ros': sec + nanosec * 1e-9,
            'sensor_id': str(payload.get('sensor_id', '')),
            'confidence': confidence,
            'peak_ratio': peak_ratio,
            'boosted_score': boosted_score,
            'acc_mag_g': acc_mag_g,
        })
        self._prune_imu_buffer(self._now())

    def _prune_imu_buffer(self, now: float):
        buffer_sec = self.get_parameter('imu_buffer_sec').value
        self._imu_evidence_buffer = deque(
            evidence for evidence in self._imu_evidence_buffer
            if now - evidence['t_ros'] <= buffer_sec)

    def _find_imu_evidence(self, candidate_time: float, now: float):
        """LiDAR 후보 시각과 맞는 유효한 IMU 증거 중 가장 신뢰도 높은 값 반환."""
        self._prune_imu_buffer(now)
        before = self.get_parameter('imu_before_candidate_sec').value
        after = self.get_parameter('imu_after_candidate_sec').value
        min_confidence = self.get_parameter('imu_min_confidence').value
        min_peak_ratio = self.get_parameter('imu_min_peak_ratio').value

        matches = []
        for evidence in self._imu_evidence_buffer:
            # 양수면 IMU 충격이 LiDAR 후보보다 먼저 발생한 것이다.
            delta = candidate_time - evidence['t_ros']
            if (-after <= delta <= before
                    and evidence['confidence'] >= min_confidence
                    and evidence['peak_ratio'] >= min_peak_ratio):
                matches.append(evidence)

        if not matches:
            return None
        return max(
            matches,
            key=lambda evidence: (
                evidence['confidence'], evidence['peak_ratio']))

    @staticmethod
    def _parse_tracks(msg: Float32MultiArray):
        data = np.asarray(msg.data, dtype=np.float64)
        tracks = []
        for i in range(0, len(data) - 8, 9):
            tid = int(data[i])
            cx, cy, cz = data[i + 1:i + 4]
            sx, sy, sz = data[i + 4:i + 7]
            verticality = data[i + 7]
            planarity = data[i + 8]
            tracks.append((tid, cx, cy, cz, sx, sy, sz, verticality, planarity))
        return tracks

    # ────────────────────────────────────────────────────
    def _on_tracks(self, msg: Float32MultiArray):
        now = self._now()
        tracks = self._parse_tracks(msg)
        self._reassociate_histories(tracks, now)
        tracks = self._normalize_tracks(tracks)
        seen_ids = set()

        events = []  # (track_id, cx, cy, cz, confidence, state_code)
        markers_info = []  # (track_id, cx, cy, cz, sz, state)

        for (tid, cx, cy, cz, sx, sy, sz, verticality, planarity) in tracks:
            seen_ids.add(tid)
            hist = self._histories.get(tid)
            if hist is None:
                hist = TrackHistory(now)
                self._histories[tid] = hist

            hist.last_seen = now
            v = None if verticality < 0.0 else float(verticality)
            hist.samples.append((now, cx, cy, cz, sz, v))
            self._trim_history(hist, now)

            confidence, state = self._update_state(tid, hist, now)

            marker_state = state
            if state == EVENT_CODE_RECOVERED:
                events += [float(tid), float(cx), float(cy), float(cz),
                           0.0, float(EVENT_CODE_RECOVERED)]
                marker_state = STATE_NORMAL
            elif state != STATE_NORMAL:
                events += [float(tid), float(cx), float(cy), float(cz),
                           float(confidence), float(state)]
            elif (self.get_parameter('protect_active_normal').value
                  and self._recently_moving(hist, now)):
                # 낙상은 아니지만 실제로 움직이고 있는 트랙 → 배경 흡수
                # 방지 대상(ground_removal_node의 BackgroundSubtractionNode가
                # /fall_events를 구독해 이 코드도 보호영역으로 취급한다).
                # 주의: protect_active_normal=True로 켜면 이 트랙 주변 반경
                # 안의 모든 포인트(배경 포함)가 강제로 출력에 다시 실린다 —
                # 즉 사람이 움직일 때마다 그 주변 배경이 같이 되살아나는
                # 부작용이 있다. 기본은 False.
                events += [float(tid), float(cx), float(cy), float(cz),
                           0.0, float(EVENT_CODE_ACTIVE_NORMAL)]
            markers_info.append((tid, cx, cy, cz, sz, marker_state))

        # 이번 프레임에 안 보인 트랙은 last_seen만 유지, 오래 안 보이면 정리
        self._cleanup_stale(now, seen_ids)

        out = Float32MultiArray()
        out.data = events
        self.pub_events.publish(out)

        self._publish_markers(markers_info)

    # ────────────────────────────────────────────────────
    def _trim_history(self, hist: TrackHistory, now: float):
        window = self.get_parameter('history_window_sec').value
        while hist.samples and (now - hist.samples[0][0]) > window:
            hist.samples.popleft()

    def _cleanup_stale(self, now: float, seen_ids: set):
        timeout = self.get_parameter('track_timeout_sec').value
        stale = [tid for tid, h in self._histories.items()
                 if tid not in seen_ids and (now - h.last_seen) > timeout]
        for tid in stale:
            del self._histories[tid]
            self._recovery_since.pop(tid, None)
            self._trigger_log_times.pop(tid, None)
            self._diagnostic_log_times.pop(tid, None)
        if stale:
            stale_set = set(stale)
            self._track_aliases = {
                raw_tid: logical_tid
                for raw_tid, logical_tid in self._track_aliases.items()
                if logical_tid not in stale_set
            }

    def _resolve_track_id(self, raw_tid: int) -> int:
        """트래커의 raw ID를 낙상 상태머신의 고정 logical ID로 변환."""
        return self._track_aliases.get(raw_tid, raw_tid)

    def _normalize_tracks(self, tracks):
        """같은 logical ID의 관측은 이전 중심에 가장 가까운 하나만 사용."""
        grouped = {}
        for track in tracks:
            logical_tid = self._resolve_track_id(track[0])
            normalized = (logical_tid, *track[1:])
            grouped.setdefault(logical_tid, []).append(normalized)

        result = []
        for logical_tid, candidates in grouped.items():
            if len(candidates) == 1:
                result.append(candidates[0])
                continue

            hist = self._histories.get(logical_tid)
            if hist is None or not hist.samples:
                result.append(candidates[0])
                continue

            _, old_x, old_y, *_rest = hist.samples[-1]
            result.append(min(
                candidates,
                key=lambda track: np.hypot(track[1] - old_x, track[2] - old_y)))
        return result

    def _has_standing_evidence(self, hist: TrackHistory) -> bool:
        values = [s[5] for s in hist.samples if s[5] is not None]
        min_samples = int(self.get_parameter('standing_min_samples').value)
        if len(values) < min_samples:
            return False
        sorted_values = np.sort(values)
        baseline = float(np.median(
            sorted_values[len(sorted_values) // 2:]))
        return baseline >= self.get_parameter('standing_verticality_min').value

    def _reassociate_histories(self, tracks, now: float):
        """새 raw ID를 기존 logical ID의 alias로 연결한다.

        TrackHistory 자체를 ID 사이에서 이동하지 않으므로 이전 raw ID가 다시
        나타나도 역방향 승계가 발생하지 않는다.
        """
        incoming_ids = {
            self._resolve_track_id(track[0]) for track in tracks
        }
        max_age = self.get_parameter('track_reassociate_sec').value
        max_dist = self.get_parameter('track_reassociate_dist').value
        lying_max = self.get_parameter('verticality_lying_max').value
        used_sources = set()

        for raw_tid, cx, cy, _cz, _sx, _sy, _sz, verticality, _planarity in tracks:
            if verticality < 0.0 or verticality > lying_max:
                continue

            tid = self._resolve_track_id(raw_tid)
            if tid != raw_tid:
                continue

            target = self._histories.get(raw_tid)
            if target is not None and (
                    target.state != STATE_NORMAL
                    or self._has_standing_evidence(target)):
                continue

            matches = []
            for old_tid, hist in list(self._histories.items()):
                if (old_tid == raw_tid or old_tid in incoming_ids
                        or old_tid in used_sources
                        or not hist.samples
                        or (now - hist.last_seen) > max_age):
                    continue
                if (hist.state == STATE_NORMAL
                        and not self._has_standing_evidence(hist)):
                    continue

                _, old_x, old_y, *_rest = hist.samples[-1]
                distance = float(np.hypot(cx - old_x, cy - old_y))
                if distance <= max_dist:
                    state_priority = 0 if hist.state != STATE_NORMAL else 1
                    matches.append((state_priority, distance, old_tid, hist))

            if not matches:
                continue

            _, distance, old_tid, source = min(matches, key=lambda m: m[:2])
            if target is not None:
                del self._histories[raw_tid]
                self._recovery_since.pop(raw_tid, None)
                self._trigger_log_times.pop(raw_tid, None)
                self._diagnostic_log_times.pop(raw_tid, None)
                self._track_aliases = {
                    alias: logical
                    for alias, logical in self._track_aliases.items()
                    if logical != raw_tid
                }

            self._track_aliases[raw_tid] = old_tid
            used_sources.add(old_tid)
            self.get_logger().warn(
                f'[FALL TRACK REASSOCIATE] raw track {raw_tid} -> '
                f'logical track {old_tid} '
                f'(distance={distance:.2f}m, state={source.state})')

    def _log_trigger_blocked(self, tid: int, now: float, detail: str):
        """누운 자세인데 후보가 되지 못한 이유를 트랙별 초당 한 번 출력."""
        last_log = self._trigger_log_times.get(tid)
        if last_log is not None and (now - last_log) < 1.0:
            return
        self._trigger_log_times[tid] = now
        self.get_logger().info(
            f'[FALL TRIGGER BLOCKED] track {tid} | {detail}')

    # ────────────────────────────────────────────────────
    def _windowed_samples(self, hist: TrackHistory, now: float, lookback_sec: float):
        return [s for s in hist.samples if (now - s[0]) <= lookback_sec]

    def _horizontal_speeds(self, samples):
        """샘플 리스트에서 연속 구간별 수평 속도(m/s) 리스트를 계산."""
        return [(t, horizontal) for t, horizontal, _, _
                in self._smoothed_motion_speeds(samples)]

    def _smoothed_motion_speeds(self, samples):
        """연속 구간 속도를 최근 N개 중앙값으로 완화한다.

        반환값은 (시각, 수평 속도, z축 절대 속도, 아래 방향 속도)이다.
        """
        raw = []
        for ((t0, x0, y0, z0, *_r0),
             (t1, x1, y1, z1, *_r1)) in zip(samples, samples[1:]):
            dt = t1 - t0
            if dt <= 1e-3:
                continue
            horizontal = np.hypot(x1 - x0, y1 - y0) / dt
            vertical = (z1 - z0) / dt
            raw.append((t1, horizontal, abs(vertical), max(-vertical, 0.0)))

        median_samples = max(1, int(
            self.get_parameter('speed_median_samples').value))
        if len(raw) < median_samples:
            return []

        smoothed = []
        for i in range(median_samples - 1, len(raw)):
            window = raw[i - median_samples + 1:i + 1]
            smoothed.append((
                raw[i][0],
                float(np.median([s[1] for s in window])),
                float(np.median([s[2] for s in window])),
                float(np.median([s[3] for s in window])),
            ))
        return smoothed

    def _current_motion(self, hist: TrackHistory):
        speeds = self._smoothed_motion_speeds(list(hist.samples))
        if not speeds:
            return 0.0, 0.0
        _, horizontal, vertical_abs, _ = speeds[-1]
        return horizontal, vertical_abs

    def _log_fall_signals(self, tid: int, hist: TrackHistory, now: float,
                          cur_speed: float, cur_vertical_speed: float):
        """후보 여부와 무관하게 낙상 판정 입력값을 트랙별로 제한 출력."""
        interval = self.get_parameter('diagnostic_log_interval_sec').value
        if interval <= 0.0:
            return
        last_log = self._diagnostic_log_times.get(tid)
        if last_log is not None and (now - last_log) < interval:
            return
        self._diagnostic_log_times[tid] = now

        lookback = self.get_parameter('speed_spike_lookback_sec').value
        samples = self._windowed_samples(hist, now, lookback)
        speeds = self._smoothed_motion_speeds(samples)
        peak_speed = max((speed[1] for speed in speeds), default=0.0)
        peak_downward_speed = max((speed[3] for speed in speeds), default=0.0)
        _, _cx, _cy, _cz, height, verticality = hist.samples[-1]
        state_name = {
            STATE_NORMAL: 'NORMAL',
            STATE_CANDIDATE: 'CANDIDATE',
            STATE_CONFIRMED: 'CONFIRMED',
        }.get(hist.state, str(hist.state))
        verticality_text = (
            f'{verticality:.2f}' if verticality is not None else 'N/A')

        self.get_logger().info(
            f'[FALL SIGNAL] track {tid} | state={state_name}, '
            f'V={verticality_text}, H={height:.2f}m, '
            f'xy_now={cur_speed:.2f}m/s, xy_peak={peak_speed:.2f}m/s, '
            f'z_now={cur_vertical_speed:.2f}m/s, '
            f'down_peak={peak_downward_speed:.2f}m/s')

    def _recently_moving(self, hist: TrackHistory, now: float) -> bool:
        """낙상 판정과 별개로, 최근 motion_lookback_sec 동안 실제로
        motion_speed_threshold 이상 이동한 적이 있는지만 본다.
        (배경 흡수 방지용 ACTIVE 마킹 목적 — 정지된 오분류 트랙을
        걸러내기 위한 순수 이동 여부 체크)
        """
        lookback = self.get_parameter('motion_lookback_sec').value
        recent = self._windowed_samples(hist, now, lookback)
        speeds = self._horizontal_speeds(recent)
        motion_thresh = self.get_parameter('motion_speed_threshold').value
        return any(v >= motion_thresh for _, v in speeds)

    # ────────────────────────────────────────────────────
    def _update_state(self, tid: int, hist: TrackHistory, now: float):
        """규칙 기반 상태머신 한 스텝. (confidence, state) 반환."""

        cur_speed, cur_vertical_speed = self._current_motion(hist)
        _, cx, cy, cz, sz, verticality = hist.samples[-1]
        self._log_fall_signals(
            tid, hist, now, cur_speed, cur_vertical_speed)

        if hist.state == STATE_NORMAL:
            triggered, confidence = self._check_fall_trigger(
                tid, hist, now, cur_speed, cur_vertical_speed)
            if triggered:
                hist.state = STATE_CANDIDATE
                hist.state_since = now
                hist.candidate_confidence = confidence
                hist.confirmed_confidence = 0.0
                self._trigger_log_times.pop(tid, None)
                self.get_logger().warn(
                    f'[FALL CANDIDATE] track {tid} 감지 '
                    f'(speed={cur_speed:.2f}m/s, V={verticality}, conf={confidence:.2f})')
                return confidence, STATE_CANDIDATE
            return 0.0, STATE_NORMAL

        if hist.state == STATE_CANDIDATE:
            # 프레임 간 bbox 중심 속도는 라이다 스캔 밀도에 따라 크게 튄다.
            # 최근 확인 구간의 앞/뒤 중심 중앙값을 비교해 장기 변위로
            # 정지를 판정하고, 같은 구간에서 누운 자세 비율을 확인한다.
            # 후보 진입 순간의 낙하 이동은 정지 판정에 포함하지 않는다.
            v_lying_max = self.get_parameter('verticality_lying_max').value
            lying_ratio_min = self.get_parameter('confirm_lying_ratio').value
            speed_max = self.get_parameter('confirm_speed_max').value
            vertical_speed_max = self.get_parameter(
                'confirm_vertical_speed_max').value
            confirm_dur = self.get_parameter('confirm_stillness_sec').value
            confirm_window_start = max(hist.state_since, now - confirm_dur)
            candidate_samples = [
                s for s in hist.samples if s[0] >= confirm_window_start
            ]
            valid_verticalities = [
                s[5] for s in candidate_samples if s[5] is not None
            ]
            lying_ratio = (
                sum(v <= v_lying_max for v in valid_verticalities)
                / len(valid_verticalities)
                if valid_verticalities else 0.0
            )

            is_stable = False
            horizontal_displacement = float('inf')
            vertical_displacement = float('inf')
            if len(candidate_samples) >= 3:
                segment_size = max(1, len(candidate_samples) // 3)
                start_center = np.median(
                    [s[1:4] for s in candidate_samples[:segment_size]], axis=0)
                end_center = np.median(
                    [s[1:4] for s in candidate_samples[-segment_size:]], axis=0)
                horizontal_displacement = float(np.hypot(
                    end_center[0] - start_center[0],
                    end_center[1] - start_center[1]))
                vertical_displacement = float(abs(end_center[2] - start_center[2]))
                is_stable = (
                    horizontal_displacement <= speed_max * confirm_dur
                    and vertical_displacement <= vertical_speed_max * confirm_dur
                )

            lidar_posture_confirmed = (
                (now - hist.state_since) >= confirm_dur
                and lying_ratio >= lying_ratio_min
                and is_stable
            )
            if lidar_posture_confirmed:
                imu_hit = self._find_imu_evidence(hist.state_since, now)
                lidar_threshold = self.get_parameter(
                    'lidar_standalone_confidence').value
                if (hist.candidate_confidence >= lidar_threshold
                        or imu_hit is not None):
                    fused_confidence = hist.candidate_confidence
                    source = 'LiDAR'
                    if imu_hit is not None:
                        imu_weight = self.get_parameter(
                            'imu_confidence_weight').value
                        fused_confidence += imu_weight * imu_hit['confidence']
                        source = (
                            f'LiDAR+IMU(sensor={imu_hit["sensor_id"] or "unknown"}, '
                            f'confidence={imu_hit["confidence"]:.2f}, '
                            f'peak_ratio={imu_hit["peak_ratio"]:.2f})')

                    hist.state = STATE_CONFIRMED
                    hist.state_since = now
                    hist.confirmed_confidence = min(fused_confidence, 1.0)
                    self.get_logger().error(
                        f'[FALL CONFIRMED] track {tid} 낙상 확정 '
                        f'위치=({cx:.2f},{cy:.2f},{cz:.2f}) | {source} | '
                        f'fused={hist.confirmed_confidence:.2f}')
                    return hist.confirmed_confidence, STATE_CONFIRMED

            timeout = self.get_parameter('candidate_timeout_sec').value
            if (now - hist.state_since) > timeout:
                # 일정 시간 안에 확정되지 않음 → 순간적인 휘청임 등 오탐으로 판단, 복귀
                hist.state = STATE_NORMAL
                hist.state_since = now
                hist.candidate_confidence = 0.0
                self.get_logger().info(
                    f'[FALL CANDIDATE 해제] track {tid} 타임아웃, 정상 복귀 '
                    f'(lying={lying_ratio:.0%}, '
                    f'd_xy={horizontal_displacement:.2f}m, '
                    f'd_z={vertical_displacement:.2f}m)')
                return 0.0, STATE_NORMAL

            return hist.candidate_confidence, STATE_CANDIDATE

        if hist.state == STATE_CONFIRMED:
            v_min = self.get_parameter('recovery_verticality_min').value
            recovering = verticality is not None and verticality >= v_min

            hold = self.get_parameter('recovery_hold_sec').value
            if recovering:
                since = self._recovery_since.get(tid)
                if since is None:
                    self._recovery_since[tid] = now
                elif (now - since) >= hold:
                    hist.state = STATE_NORMAL
                    hist.state_since = now
                    hist.candidate_confidence = 0.0
                    hist.confirmed_confidence = 0.0
                    self._recovery_since.pop(tid, None)
                    self.get_logger().info(f'[FALL 회복] track {tid} 다시 일어남 → 정상 복귀')
                    return 0.0, EVENT_CODE_RECOVERED
            else:
                self._recovery_since.pop(tid, None)

            return hist.confirmed_confidence, STATE_CONFIRMED

        return 0.0, STATE_NORMAL

    # ────────────────────────────────────────────────────
    def _check_fall_trigger(self, tid: int, hist: TrackHistory, now: float,
                            cur_speed: float, cur_vertical_speed: float):
        """NORMAL → CANDIDATE 진입 조건 판정.

        조건 = 사전에 서 있었음 AND
               (수평 속도 급증 OR bbox 중심 급하강) AND
               (verticality 급락 OR 높이 급감)
        confidence는 만족한 신호 개수에 비례해 0.5~1.0 사이로 산출.
        """
        median_samples = max(1, int(
            self.get_parameter('speed_median_samples').value))
        if len(hist.samples) < median_samples + 1:
            current_v = hist.samples[-1][5]
            lying_max = self.get_parameter('verticality_lying_max').value
            if current_v is not None and current_v <= lying_max:
                self._log_trigger_blocked(
                    tid, now,
                    f'history | samples={len(hist.samples)}/'
                    f'{median_samples + 1}, V={current_v:.2f}')
            return False, 0.0

        # 낙상 전에 실제로 서 있던 트랙인지 확인한다. 현재 샘플은 제외한다.
        standing_min = self.get_parameter('standing_verticality_min').value
        standing_min_samples = int(
            self.get_parameter('standing_min_samples').value)
        prior_verticalities = [
            s[5] for s in list(hist.samples)[:-1] if s[5] is not None
        ]
        if len(prior_verticalities) < standing_min_samples:
            current_v = hist.samples[-1][5]
            lying_max = self.get_parameter('verticality_lying_max').value
            if current_v is not None and current_v <= lying_max:
                self._log_trigger_blocked(
                    tid, now,
                    f'standing_history | valid_samples='
                    f'{len(prior_verticalities)}/{standing_min_samples}, '
                    f'V={current_v:.2f}')
            return False, 0.0
        sorted_verticalities = np.sort(prior_verticalities)
        standing_baseline = float(np.median(
            sorted_verticalities[len(sorted_verticalities) // 2:]))
        if standing_baseline < standing_min:
            current_v = hist.samples[-1][5]
            lying_max = self.get_parameter('verticality_lying_max').value
            if current_v is not None and current_v <= lying_max:
                self._log_trigger_blocked(
                    tid, now,
                    f'standing | baseline={standing_baseline:.2f} '
                    f'< {standing_min:.2f}, V={current_v:.2f}')
            return False, 0.0

        # 사전 수평 이동 여부. 제자리 낙상은 아래의 z축 하강 신호로 통과한다.
        lookback = self.get_parameter('motion_lookback_sec').value
        recent = self._windowed_samples(hist, now, lookback)
        speeds = self._horizontal_speeds(recent)
        motion_thresh = self.get_parameter('motion_speed_threshold').value
        was_moving = any(v >= motion_thresh for _, v in speeds)

        # 낙상 동작 중에는 아직 속도가 높으므로 여기서 정지를 함께 요구하지
        # 않는다. 정지는 CANDIDATE 진입 후 확인 구간에서 별도로 검증한다.
        spike_lookback = self.get_parameter('speed_spike_lookback_sec').value
        spike_samples = self._windowed_samples(hist, now, spike_lookback)
        motion_speeds = self._smoothed_motion_speeds(spike_samples)
        peak_speed = max((s[1] for s in motion_speeds), default=0.0)
        peak_downward_speed = max((s[3] for s in motion_speeds), default=0.0)
        spike_thresh = self.get_parameter('speed_spike_threshold').value
        horizontal_fall_motion = (
            was_moving
            and peak_speed >= spike_thresh
        )
        vertical_fall_thresh = self.get_parameter(
            'vertical_fall_speed_threshold').value
        vertical_fall_motion = peak_downward_speed >= vertical_fall_thresh
        if not (horizontal_fall_motion or vertical_fall_motion):
            current_v = hist.samples[-1][5]
            lying_max = self.get_parameter('verticality_lying_max').value
            if current_v is not None and current_v <= lying_max:
                self._log_trigger_blocked(
                    tid, now,
                    f'motion | was_moving={was_moving}, '
                    f'peak_xy={peak_speed:.2f}/{spike_thresh:.2f}m/s, '
                    f'cur_xy={cur_speed:.2f}m/s, '
                    f'peak_down={peak_downward_speed:.2f}/'
                    f'{vertical_fall_thresh:.2f}m/s, '
                    f'cur_z={cur_vertical_speed:.2f}m/s')
            return False, 0.0

        # PCA verticality 급락: 단일 최고값 대신 서 있던 구간의 강건한 기준을 쓴다.
        v_window = self.get_parameter('verticality_drop_window_sec').value
        v_drop_thresh = self.get_parameter('verticality_drop_threshold').value
        v_samples = self._windowed_samples(hist, now, v_window)
        current_verticality = hist.samples[-1][5]
        prior_v_values = [s[5] for s in v_samples[:-1] if s[5] is not None]
        verticality_dropped = False
        verticality_baseline = float('nan')
        verticality_drop = float('nan')
        if current_verticality is not None and prior_v_values:
            sorted_values = np.sort(prior_v_values)
            verticality_baseline = float(np.median(
                sorted_values[len(sorted_values) // 2:]))
            verticality_drop = verticality_baseline - current_verticality
            verticality_dropped = verticality_drop >= v_drop_thresh

        # bbox 높이 급감: 현재값을 제외한 상위 절반의 중앙값을 기준으로 쓴다.
        h_window = self.get_parameter('height_drop_window_sec').value
        h_drop_ratio = self.get_parameter('height_drop_ratio').value
        h_samples = self._windowed_samples(hist, now, h_window)
        prior_sz_values = [s[4] for s in h_samples[:-1] if s[4] > 1e-3]
        current_sz = hist.samples[-1][4]
        height_dropped = False
        height_baseline = float('nan')
        height_drop_fraction = 0.0
        if len(prior_sz_values) >= 3:
            sorted_heights = np.sort(prior_sz_values)
            height_baseline = float(np.median(
                sorted_heights[len(sorted_heights) // 2:]))
            height_drop_fraction = (
                height_baseline - current_sz) / height_baseline
            height_dropped = height_drop_fraction >= h_drop_ratio

        if not (verticality_dropped or height_dropped):
            self._log_trigger_blocked(
                tid, now,
                f'posture | V={current_verticality}, '
                f'V_base={verticality_baseline:.2f}, '
                f'V_drop={verticality_drop:.2f}/'
                f'{v_drop_thresh:.2f}, H={current_sz:.2f}m, '
                f'H_base={height_baseline:.2f}m, '
                f'H_drop={height_drop_fraction:.0%}/{h_drop_ratio:.0%}')
            return False, 0.0

        # confidence: 기본 0.5(속도 신호) + 자세 신호 각각 +0.25
        confidence = 0.5
        if verticality_dropped:
            confidence += 0.25
        if height_dropped:
            confidence += 0.25
        return True, min(confidence, 1.0)

    # ────────────────────────────────────────────────────
    def _publish_markers(self, markers_info):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        stamp = self.get_clock().now().to_msg()

        for tid, cx, cy, cz, sz, state in markers_info:
            if state == STATE_NORMAL:
                continue

            if state == STATE_CANDIDATE:
                color = ColorRGBA(r=1.0, g=0.6, b=0.0, a=0.6)
                label = f'FALL? T{tid}'
            else:  # CONFIRMED
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.75)
                label = f'FALL DETECTED\nT{tid}'

            ring = Marker()
            ring.header.frame_id = self.frame_id
            ring.header.stamp = stamp
            ring.ns = 'fall_marker'
            ring.id = tid
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = float(cx)
            ring.pose.position.y = float(cy)
            ring.pose.position.z = float(cz)
            ring.pose.orientation.w = 1.0
            ring.scale.x = 0.9
            ring.scale.y = 0.9
            ring.scale.z = max(float(sz), 0.1)
            ring.color = color
            ring.lifetime.sec = 1
            ma.markers.append(ring)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = stamp
            text.ns = 'fall_marker_text'
            text.id = tid + 100000
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(cx)
            text.pose.position.y = float(cy)
            text.pose.position.z = float(cz) + max(float(sz), 0.1) / 2 + 0.3
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = label
            text.lifetime.sec = 1
            ma.markers.append(text)

        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = FallDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

