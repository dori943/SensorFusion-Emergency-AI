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
    2) 수평 속도 급증 후 급감 또는 bbox 중심의 빠른 z축 하강 후 정지
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
  일정 시간 "누운 자세 + 정지 상태"가 유지되면 FALL_CONFIRMED로
  확정한다(단순 순간 휘청임/재빨리 앉기와 구분하기 위한 확인 지연).
  확정 뒤 사람이 다시 일어나 움직이면 자동으로 NORMAL로 복귀한다.

입력:
  /human_tracks (std_msgs/Float32MultiArray)
      human_bbox_node가 퍼블리시.
      포맷: 9개씩 반복 [track_id,cx,cy,cz,sx,sy,sz,verticality,planarity]
      verticality/planarity가 -1.0이면 "PCA 미계산(정보 없음)"을 의미.

출력:
  /fall_events  (std_msgs/Float32MultiArray)
      상태가 CANDIDATE 이상인 트랙 + (protect_active_normal=True일 때만)
      낙상은 아니지만 최근 실제로 이동 중인(ACTIVE) 트랙을 담아 퍼블리시.
      포맷: 6개씩 반복 [track_id,cx,cy,cz,confidence,state_code]
      state_code: 1=CANDIDATE(의심), 2=CONFIRMED(확정),
                  3=ACTIVE(낙상 아님, 최근 실제 이동 중 — 배경 흡수 방지용,
                  protect_active_normal=True일 때만 발행)
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

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray


STATE_NORMAL = 0
STATE_CANDIDATE = 1
STATE_CONFIRMED = 2
# 낙상 상태머신의 실제 state는 아니고, /fall_events 출력 전용 코드.
# NORMAL이지만 최근에 실제로 이동한 트랙(=배경 흡수 방지 대상)을 표시.
EVENT_CODE_ACTIVE_NORMAL = 3


class TrackHistory:
    """트랙 하나의 최근 시계열(시각, 위치, 크기, PCA 형태)을 보관."""

    __slots__ = ("samples", "state", "state_since", "still_since",
                 "last_seen")

    def __init__(self, now_sec: float):
        self.samples = deque()   # (t, cx, cy, cz, sz, verticality)
        self.state = STATE_NORMAL
        self.state_since = now_sec      # 현재 state로 바뀐 시각
        self.still_since = None         # CANDIDATE 진입 후 "정지+누움"이 끊기지 않고 유지된 시작 시각
        self.last_seen = now_sec        # /human_tracks에 마지막으로 잡힌 시각 (트랙 청소용)


class FallDetectionNode(Node):

    def __init__(self):
        super().__init__('fall_detection_node')

        # ── 입출력 토픽 ──
        self.declare_parameter('tracks_topic', '/human_tracks')
        self.declare_parameter('output_topic', '/fall_events')
        self.declare_parameter('marker_topic', '/fall_marker')
        self.declare_parameter('frame_id', 'unilidar_lidar')

        # ── 이력 버퍼 ──
        self.declare_parameter('history_window_sec', 3.0)     # 트랙별로 보관할 최대 이력 길이

        # ── 1) 사전 이동 게이트 ──
        self.declare_parameter('motion_speed_threshold', 0.15)   # m/s, 이 이상이면 "움직였다"
        self.declare_parameter('motion_lookback_sec', 2.0)       # 이 기간 내 이동 여부를 확인

        # ── 2) 속도 급증 후 급감 ──
        self.declare_parameter('speed_spike_threshold', 0.8)     # m/s, 낙상성 순간 속도
        self.declare_parameter('speed_spike_lookback_sec', 1.2)  # 스파이크를 찾는 최근 창
        self.declare_parameter('speed_drop_ratio', 0.25)         # 스파이크 대비 현재 속도가 이 비율 밑이면 "급감"
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
        self.declare_parameter('confirm_speed_max', 0.2)         # "정지"로 볼 속도 상한 (m/s)
        self.declare_parameter('recovery_speed_threshold', 0.3)  # CONFIRMED 상태에서 이 이상 속도 + 아래 조건이면 회복으로 간주
        self.declare_parameter('recovery_verticality_min', 0.6)  # 다시 이 이상 서면 회복 신호
        self.declare_parameter('recovery_hold_sec', 1.0)         # 회복 신호가 이만큼 유지돼야 NORMAL 복귀 (순간 오검출 방지)

        # ── 트랙 청소 ──
        self.declare_parameter('track_timeout_sec', 5.0)  # 이 시간 이상 /human_tracks에서 안 보이면 이력 삭제

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

        self._histories: dict[int, TrackHistory] = {}
        self._recovery_since: dict[int, float] = {}  # CONFIRMED 트랙별 "회복 신호 지속 시작 시각"

        self.create_subscription(
            Float32MultiArray, self.tracks_topic, self._on_tracks, 10)

        self.pub_events = self.create_publisher(
            Float32MultiArray, self.output_topic, 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, self.marker_topic, 10)

        self.get_logger().info(
            f'FallDetectionNode 시작 | tracks={self.tracks_topic} | '
            f'motion>={self.get_parameter("motion_speed_threshold").value}m/s '
            f'spike>={self.get_parameter("speed_spike_threshold").value}m/s '
            f'v_drop>={self.get_parameter("verticality_drop_threshold").value} '
            f'confirm={self.get_parameter("confirm_stillness_sec").value}s '
            f'protect_active_normal={self.get_parameter("protect_active_normal").value}'
        )

    # ────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

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

            if state != STATE_NORMAL:
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
            markers_info.append((tid, cx, cy, cz, sz, state))

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

        if hist.state == STATE_NORMAL:
            triggered, confidence = self._check_fall_trigger(
                hist, now, cur_speed, cur_vertical_speed)
            if triggered:
                hist.state = STATE_CANDIDATE
                hist.state_since = now
                hist.still_since = None
                self.get_logger().warn(
                    f'[FALL CANDIDATE] track {tid} 감지 '
                    f'(speed={cur_speed:.2f}m/s, V={verticality}, conf={confidence:.2f})')
                return confidence, STATE_CANDIDATE
            return 0.0, STATE_NORMAL

        if hist.state == STATE_CANDIDATE:
            # 계속 "누운 + 정지" 상태인지 확인 → 유지되면 CONFIRMED로 승격
            v_lying_max = self.get_parameter('verticality_lying_max').value
            speed_max = self.get_parameter('confirm_speed_max').value
            vertical_speed_max = self.get_parameter(
                'confirm_vertical_speed_max').value
            is_still_and_lying = (
                cur_speed <= speed_max
                and cur_vertical_speed <= vertical_speed_max
                and verticality is not None
                and verticality <= v_lying_max
            )

            if is_still_and_lying:
                if hist.still_since is None:
                    hist.still_since = now
            else:
                # 정지/누움이 한 번이라도 끊기면 다시 처음부터 카운트
                # (걸어서 지나가다 순간적으로 트리거 조건에 걸린 오탐 배제)
                hist.still_since = None

            confirm_dur = self.get_parameter('confirm_stillness_sec').value
            if hist.still_since is not None and (now - hist.still_since) >= confirm_dur:
                hist.state = STATE_CONFIRMED
                hist.state_since = now
                self.get_logger().error(
                    f'[FALL CONFIRMED] track {tid} 낙상 확정 위치=({cx:.2f},{cy:.2f},{cz:.2f})')
                return 0.95, STATE_CONFIRMED

            timeout = self.get_parameter('candidate_timeout_sec').value
            if (now - hist.state_since) > timeout:
                # 일정 시간 안에 확정되지 않음 → 순간적인 휘청임 등 오탐으로 판단, 복귀
                hist.state = STATE_NORMAL
                hist.state_since = now
                hist.still_since = None
                self.get_logger().info(f'[FALL CANDIDATE 해제] track {tid} 타임아웃, 정상 복귀')
                return 0.0, STATE_NORMAL

            return 0.6, STATE_CANDIDATE

        if hist.state == STATE_CONFIRMED:
            v_min = self.get_parameter('recovery_verticality_min').value
            speed_min = self.get_parameter('recovery_speed_threshold').value
            recovering = (cur_speed >= speed_min) and (
                verticality is not None and verticality >= v_min)

            hold = self.get_parameter('recovery_hold_sec').value
            if recovering:
                since = self._recovery_since.get(tid)
                if since is None:
                    self._recovery_since[tid] = now
                elif (now - since) >= hold:
                    hist.state = STATE_NORMAL
                    hist.state_since = now
                    hist.still_since = None
                    self._recovery_since.pop(tid, None)
                    self.get_logger().info(f'[FALL 회복] track {tid} 다시 일어남 → 정상 복귀')
                    return 0.0, STATE_NORMAL
            else:
                self._recovery_since.pop(tid, None)

            return 0.95, STATE_CONFIRMED

        return 0.0, STATE_NORMAL

    # ────────────────────────────────────────────────────
    def _check_fall_trigger(self, hist: TrackHistory, now: float,
                            cur_speed: float, cur_vertical_speed: float):
        """NORMAL → CANDIDATE 진입 조건 판정.

        조건 = 사전에 서 있었음 AND
               ((수평 이동 후 급정지) OR (bbox 중심 급하강 후 정지)) AND
               (verticality 급락 OR 높이 급감)
        confidence는 만족한 신호 개수에 비례해 0.5~1.0 사이로 산출.
        """
        median_samples = max(1, int(
            self.get_parameter('speed_median_samples').value))
        if len(hist.samples) < median_samples + 1:
            return False, 0.0

        # 낙상 전에 실제로 서 있던 트랙인지 확인한다. 현재 샘플은 제외한다.
        standing_min = self.get_parameter('standing_verticality_min').value
        standing_min_samples = int(
            self.get_parameter('standing_min_samples').value)
        prior_verticalities = [
            s[5] for s in list(hist.samples)[:-1] if s[5] is not None
        ]
        if len(prior_verticalities) < standing_min_samples:
            return False, 0.0
        sorted_verticalities = np.sort(prior_verticalities)
        standing_baseline = float(np.median(
            sorted_verticalities[len(sorted_verticalities) // 2:]))
        if standing_baseline < standing_min:
            return False, 0.0

        # 사전 수평 이동 여부. 제자리 낙상은 아래의 z축 하강 신호로 통과한다.
        lookback = self.get_parameter('motion_lookback_sec').value
        recent = self._windowed_samples(hist, now, lookback)
        speeds = self._horizontal_speeds(recent)
        motion_thresh = self.get_parameter('motion_speed_threshold').value
        was_moving = any(v >= motion_thresh for _, v in speeds)

        # 수평 속도 급증 후 급감 또는 bbox 중심의 빠른 하강 후 정지를 찾는다.
        spike_lookback = self.get_parameter('speed_spike_lookback_sec').value
        spike_samples = self._windowed_samples(hist, now, spike_lookback)
        motion_speeds = self._smoothed_motion_speeds(spike_samples)
        peak_speed = max((s[1] for s in motion_speeds), default=0.0)
        peak_downward_speed = max((s[3] for s in motion_speeds), default=0.0)
        spike_thresh = self.get_parameter('speed_spike_threshold').value
        drop_ratio = self.get_parameter('speed_drop_ratio').value
        horizontal_spike_then_drop = (
            was_moving
            and peak_speed >= spike_thresh
            and cur_speed <= peak_speed * drop_ratio
        )
        vertical_fall_thresh = self.get_parameter(
            'vertical_fall_speed_threshold').value
        vertical_stop_thresh = self.get_parameter(
            'confirm_vertical_speed_max').value
        vertical_fall_then_stop = (
            peak_downward_speed >= vertical_fall_thresh
            and cur_vertical_speed <= vertical_stop_thresh
        )
        if not (horizontal_spike_then_drop or vertical_fall_then_stop):
            return False, 0.0

        # PCA verticality 급락: 단일 최고값 대신 서 있던 구간의 강건한 기준을 쓴다.
        v_window = self.get_parameter('verticality_drop_window_sec').value
        v_drop_thresh = self.get_parameter('verticality_drop_threshold').value
        v_samples = self._windowed_samples(hist, now, v_window)
        current_verticality = hist.samples[-1][5]
        prior_v_values = [s[5] for s in v_samples[:-1] if s[5] is not None]
        verticality_dropped = False
        if current_verticality is not None and prior_v_values:
            sorted_values = np.sort(prior_v_values)
            baseline = float(np.median(
                sorted_values[len(sorted_values) // 2:]))
            verticality_dropped = (
                baseline - current_verticality) >= v_drop_thresh

        # bbox 높이 급감: 현재값을 제외한 상위 절반의 중앙값을 기준으로 쓴다.
        h_window = self.get_parameter('height_drop_window_sec').value
        h_drop_ratio = self.get_parameter('height_drop_ratio').value
        h_samples = self._windowed_samples(hist, now, h_window)
        prior_sz_values = [s[4] for s in h_samples[:-1] if s[4] > 1e-3]
        current_sz = hist.samples[-1][4]
        height_dropped = False
        if len(prior_sz_values) >= 3:
            sorted_heights = np.sort(prior_sz_values)
            height_baseline = float(np.median(
                sorted_heights[len(sorted_heights) // 2:]))
            height_dropped = (
                (height_baseline - current_sz) / height_baseline
                >= h_drop_ratio
            )

        if not (verticality_dropped or height_dropped):
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

