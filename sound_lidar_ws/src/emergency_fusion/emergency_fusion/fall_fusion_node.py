"""
emergency_fusion/fall_fusion_node.py
────────────────────────────────────────────────────────────────────────
목적:
  낙상 태스크의 "최종 판단자". 서로 다른 3개 모달리티(LiDAR / Mic Array /
  IMU)가 각자 독립적으로 뽑아낸 증거를 시간·공간으로 정합해 하나의
  낙상 사건(FallCase)으로 묶고, 가중 융합 점수로 최종 판정을 내린다.

  각 노드는 "증거(evidence)"만 발행하고 판단은 하지 않는다는 원칙:
    - fall_detection_node   → 자세/속도 기반 낙상 상태 (가장 강한 앵커)
    - sound_source_marker   → 소리 발생 방향이 그 사람 위치와 일치하는가
    - imu_fall_sos_node     → 충격 피크가 동적 임계값을 넘었는가

  ── 융합 점수 ──────────────────────────────────────────────
      score = w_lidar·s_lidar + w_sound·s_sound + w_imu·s_imu
              (기본 0.70)      (기본 0.10)      (기본 0.20)

    s_lidar : CANDIDATE=0.6, CONFIRMED=1.0
    s_imu   : /fall/evidence JSON의 confidence (severity 게이트 + floor 0.6)
    s_sound : 낙상 시각·위치와 정합된 소리가 있으면 SoundEvent confidence

    confirm_threshold = 0.85 이므로 판정은 사실상 아래와 같다.
        LiDAR 확정 + IMU 피크 = 0.70 + 0.20 = 0.90 ≥ 0.85  → 확정
        LiDAR 확정 + 소리     = 0.70 + 0.10 = 0.80 <  0.85  → 보류(PENDING)
    즉 "IMU가 없으면 즉시 확정하지 않는다"가 불변식이고, 이 구간이 바로
    사용자가 지적한 미탐(false negative) 구간이다.

  ── 소리를 왜 점수에서 사실상 뺐는가 ────────────────────────
    위 산술에서 알 수 있듯, IMU가 있으면 소리 없이도 0.90으로 확정되고
    IMU가 없으면 소리를 더해도 0.80으로 미확정이다. 즉 w_sound가
    0.1이든 0이든 최종 판정은 한 번도 뒤집히지 않는다. 이건 파라미터를
    잘못 잡아서가 아니라, 지금의 소리 파이프라인이 낙상 태스크에
    정보를 거의 주지 못하기 때문이다.

      · sound_localizer는 threshold_db(기본 50dB)만 넘으면 10Hz로 계속
        발행한다. 즉 "소리가 났다"는 거의 상시 참이다.
      · sound_source_marker는 그 방향에서 가장 가까운 트랙에 태깅할 뿐,
        그게 비명인지 TV 소리인지 그냥 통화 중인지 구분하지 못한다.
      · 방 안에 사람이 한 명이면 그 사람이 말만 해도 매칭된다.

    상시 켜져 있는 신호를 점수에 더하는 것은 정보를 더하는 게 아니라
    임계값을 그만큼 낮추는 것과 같다. 그래서 w_sound는 "표시/로깅용
    보조 항"으로만 남기고(0으로 두면 lidar 0.75 + imu 0.25 구성과
    결정 함수가 완전히 동일해진다), 소리의 실제 기여는 아래
    "소리 가속" 경로로 옮겼다.

    ※ 향후 업그레이드: 소리를 진짜 가중 모달리티로 쓰려면 DoA가 아니라
      "구조 요청인가"를 판별하는 분류기 출력(emergency / normal_speech /
      background)이 들어와야 한다. 그때는 w_sound를 올리고 s_sound를
      emergency 클래스 확률로 바꾸면 이 노드는 그대로 쓸 수 있다.

  ── 미탐 구제 로직 (핵심) ──────────────────────────────────
    실제 낙상인데 IMU가 못 잡는 경우는 흔하다.
      · 웨어러블/바닥 IMU를 안 차고 있었거나 BLE가 끊겼다
      · 카펫·매트 위로 천천히 주저앉아 충격 피크가 안 나온다
      · 임계값이 보수적으로 잡혀 있다
    이때 점수가 0.75에서 멈춘다고 그냥 버리면 진짜 낙상을 놓친다.
    그래서 0.75 구간을 폐기하지 않고 PENDING(보류)으로 유지한 뒤,
    "낙상 이후의 행동"을 계속 관찰해서 두 경로로 위급(CRITICAL) 승격한다.

      경로 A — 지속 누움 (long lie)
        낙상 위치에서 계속 누운 자세(verticality 낮음) + 정지 상태가
        critical_lying_sec 이상 유지 → 스스로 일어나지 못하는 상태.
        의학적으로 long lie는 그 자체가 예후를 악화시키는 위험 신호다.

      경로 B — bbox 소실 (track lost)
        낙상 후 그 위치 주변에서 사람 트랙이 아예 사라지고
        critical_missing_sec 동안 돌아오지 않음.
        쓰러지면서 높이가 person_height_min 아래로 내려가거나, 가구에
        가려지거나, 정지한 채로 배경에 흡수되면 실제로 이런 일이 난다.
        "사람이 사라졌다"는 곧 "관측이 끊긴 낙상자"이므로 안전 측으로
        판단한다.

      소리 가속 — 위 두 경로의 대기 시간을 줄이는 용도
        누워 있는 동안 그 위치에서 소리가 반복해서 나면
        critical_lying_sec를 sound_lying_speedup 배로 단축한다.
        "말소리인지 신음인지 구분 못 한다"는 한계는 그대로지만,
        이미 LiDAR가 '넘어져서 계속 누워 있다'고 판단한 뒤의 조건부라
        사전 확률이 완전히 다르다. 10초 넘게 누운 채 소리를 내는
        상황은 그 자체로 정상 대화와 구분된다. 소리를 점수에 더할
        때와 달리, 여기서는 오탐이 "정상인을 낙상으로 만드는" 게
        아니라 "이미 누운 사람의 알림을 몇 초 앞당기는" 것뿐이다.

        ※ 단, 스스로 일어나 걸어 나간 경우와 반드시 구분해야 한다.
          소실 직전 마지막 관측이 (기립 자세 + 이동 중)이었거나
          /fall_events의 RECOVERED(4) 이벤트를 받았으면 이탈로 보고
          승격하지 않고 사건을 종료한다.
          또 /human_tracks 자체가 stale(상위 노드 다운)이면 소실
          타이머를 아예 진행시키지 않는다 — 노드 죽음을 낙상으로
          오인하지 않기 위함.

    CRITICAL로 올라가면 해제(회복 또는 ack)될 때까지
    critical_repeat_sec 주기로 알림을 계속 재발행한다.

  ── 상태 ───────────────────────────────────────────────────
    0 OBSERVING : 증거 부족, 관찰만
    1 PENDING   : 근거 있음(≈0.75), 확정 보류 + 사후 관찰 중
    2 CONFIRMED : 융합 점수 임계 초과 → 낙상 확정 알림
    3 CRITICAL  : 위급 낙상 (long lie 또는 track lost) → 반복 알림
    4 RESOLVED  : 회복/이탈/ack로 종료

입력:
  /fall_events         (std_msgs/Float32MultiArray)
        6개씩 [track_id, cx, cy, cz, confidence, state_code]
        state_code 1=CANDIDATE 2=CONFIRMED 3=ACTIVE 4=RECOVERED
  /sound_source_track  (std_msgs/Float32MultiArray)
        5개 [track_id, cx, cy, cz, confidence]  (track_id=-1 → 방향만 유효)
  /fall/evidence       (std_msgs/String, JSON)
        imu_fall_sos_node의 fall-evidence 스키마
  /human_tracks        (std_msgs/Float32MultiArray)
        9개씩 [track_id, cx, cy, cz, sx, sy, sz, verticality, planarity]
        (사후 관찰 — 누움 지속 / bbox 소실 판정에 사용. verticality=-1은 미계산)
  /emergency/fall_ack  (std_msgs/String)
        "all" 또는 case_id / {"case_id": n} → 해당 알림 반복 중지

출력:
  /emergency/fall_alert  (std_msgs/String, JSON)  최종 낙상 알림 (반복 발행)
  /emergency/fall_state  (std_msgs/Float32MultiArray)
        6개씩 [case_id, cx, cy, cz, fused_score, level_code]
  /emergency/fall_marker (visualization_msgs/MarkerArray)  rviz2 확인용
"""

from __future__ import annotations

import json
import math
from collections import deque

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point


# ── 융합 상태 코드 ───────────────────────────────────────────────
LEVEL_OBSERVING = 0
LEVEL_PENDING = 1
LEVEL_CONFIRMED = 2
LEVEL_CRITICAL = 3
LEVEL_RESOLVED = 4

LEVEL_NAME = {
    LEVEL_OBSERVING: 'observing',
    LEVEL_PENDING: 'pending',
    LEVEL_CONFIRMED: 'confirmed',
    LEVEL_CRITICAL: 'critical',
    LEVEL_RESOLVED: 'resolved',
}

# /fall_events state_code
SC_CANDIDATE = 1
SC_CONFIRMED = 2
SC_ACTIVE = 3
SC_RECOVERED = 4

SEVERITY_RANK = {'weak': 0, 'medium': 1, 'strong': 2, 'critical': 3}


def wrap180(deg: float) -> float:
    return (deg + 180.0) % 360.0 - 180.0


def angle_of(x: float, y: float) -> float:
    return math.degrees(math.atan2(y, x))


def xy_dist(a, b) -> float:
    return float(math.hypot(a[0] - b[0], a[1] - b[1]))


class SoundSample:
    __slots__ = ('t', 'track_id', 'pos', 'confidence')

    def __init__(self, t, track_id, pos, confidence):
        self.t = t
        self.track_id = int(track_id)
        self.pos = pos
        self.confidence = float(confidence)


class ImuSample:
    __slots__ = ('t', 'confidence', 'severity', 'peak_ratio', 'raw')

    def __init__(self, t, confidence, severity, peak_ratio, raw):
        self.t = t
        self.confidence = float(confidence)
        self.severity = str(severity)
        self.peak_ratio = float(peak_ratio)
        self.raw = raw


class TrackSample:
    """/human_tracks 한 트랙."""
    __slots__ = ('track_id', 'center', 'size', 'verticality', 'planarity')

    def __init__(self, track_id, center, size, verticality, planarity):
        self.track_id = int(track_id)
        self.center = center
        self.size = size
        # -1.0 은 "PCA 미계산" → None으로 정규화
        self.verticality = None if verticality < 0.0 else float(verticality)
        self.planarity = None if planarity < 0.0 else float(planarity)


class FallCase:
    """하나의 낙상 사건. track_id가 아니라 '위치'를 기준으로 유지한다.

    human_bbox/fall_detection이 낙상 중 트랙 ID를 갈아끼우는 일이 잦고,
    쓰러진 뒤 새 ID로 다시 잡히는 경우도 있어서, ID 일치는 힌트로만 쓰고
    실제 동일성은 xy 거리로 판단한다.
    """

    _next_id = 1

    def __init__(self, track_id: int, pos: np.ndarray, now: float):
        self.case_id = FallCase._next_id
        FallCase._next_id += 1

        self.track_id = int(track_id)
        self.pos = pos.copy()          # 낙상 지점(사후 관찰의 공간 앵커)
        self.t_onset = now             # 낙상 시작 시각(증거 정합 기준시각)

        # ── 모달리티별 증거 ──
        self.lidar_state = SC_CANDIDATE
        self.lidar_score = 0.0
        self.t_lidar = now

        self.sound_score = 0.0
        self.t_sound = None
        self.sound_angle_err = None
        self.sound_hits_recent = 0     # 누운 뒤 최근 창에서 매칭된 소리 개수
        self.lying_limit_used = None   # 실제 적용된 long lie 임계(가속 반영)

        self.imu_score = 0.0
        self.t_imu = None
        self.imu_severity = None
        self.imu_peak_ratio = None

        self.fused = 0.0

        # ── 사후 관찰 ──
        self.level = LEVEL_OBSERVING
        self.t_level = now
        self.escalation_reason = None

        self.lying_since = None        # 누움+정지 연속 시작 시각
        self.missing_since = None      # 트랙 소실 시작 시각
        self.upright_since = None      # 기립 회복 연속 시작 시각

        self.last_seen_t = now
        self.last_verticality = None
        self.last_speed = 0.0
        self.seen_once = False
        self.recovered_event = False   # /fall_events RECOVERED 수신

        self.motion = deque(maxlen=20)  # (t, center) — 속도 추정용

        # ── 알림 ──
        self.alert_count = 0
        self.t_last_alert = None
        self.acked = False

    # ── 헬퍼 ────────────────────────────────────────────────
    def lying_duration(self, now: float) -> float:
        return 0.0 if self.lying_since is None else now - self.lying_since

    def missing_duration(self, now: float) -> float:
        return 0.0 if self.missing_since is None else now - self.missing_since

    def looks_like_walkaway(self, standing_min: float, exit_speed: float) -> bool:
        """소실 직전 마지막 관측이 '일어나서 걸어 나감'으로 보이는가."""
        if self.recovered_event:
            return True
        if self.last_verticality is None:
            return False
        return (self.last_verticality >= standing_min
                and self.last_speed >= exit_speed)


class FallFusionNode(Node):

    def __init__(self):
        super().__init__('fall_fusion_node')

        # ── 토픽 ──
        self.declare_parameter('fall_events_topic', '/fall_events')
        self.declare_parameter('sound_track_topic', '/sound_source_track')
        self.declare_parameter('imu_evidence_topic', '/fall/evidence')
        self.declare_parameter('human_tracks_topic', '/human_tracks')
        self.declare_parameter('ack_topic', '/emergency/fall_ack')
        self.declare_parameter('alert_topic', '/emergency/fall_alert')
        self.declare_parameter('state_topic', '/emergency/fall_state')
        self.declare_parameter('marker_topic', '/emergency/fall_marker')
        self.declare_parameter('frame_id', 'unilidar_lidar')
        self.declare_parameter('eval_rate', 10.0)

        # ── 융합 가중치 / 임계값 ──
        # 합 1.0. LiDAR가 주 앵커, IMU가 확정 증거, 소리는 보조 표시 항.
        # w_sound=0.0으로 두면 "lidar 0.7+0.1 / imu 0.2"가 아니라
        # 사실상 lidar 0.75 / imu 0.25 구성이 되며, confirm 판정 결과는
        # 두 구성이 완전히 동일하다(아래 임계값 주석 참고).
        self.declare_parameter('w_lidar', 0.70)
        self.declare_parameter('w_sound', 0.10)
        self.declare_parameter('w_imu', 0.20)
        # 0.85: LiDAR+IMU(0.90)는 통과, LiDAR+소리(0.80)는 미통과.
        # 이 값을 0.80 이하로 내리면 소리만으로 확정이 되어버려
        # "상시 켜져 있는 신호"가 그대로 오탐이 된다. 내리지 말 것.
        self.declare_parameter('confirm_threshold', 0.85)   # 즉시 확정 임계
        # 0.55: LiDAR 확정 단독(0.70)과 LiDAR 후보+IMU(0.60)는 모두 사후
        # 관찰에 넣고, LiDAR 후보 단독(0.42)은 넣지 않는 지점.
        # 후보 단계에서 트랙이 곧바로 소실되는 낙상(쓰러지면서 bbox가
        # 깨지는 경우)을 놓치지 않으려면 이 값을 0.6 이상으로 올리면 안 된다.
        self.declare_parameter('pending_threshold', 0.55)   # 보류(사후 관찰) 진입 임계

        # 모달리티별 정규화 점수
        self.declare_parameter('lidar_candidate_score', 0.6)
        self.declare_parameter('lidar_confirmed_score', 1.0)
        self.declare_parameter('imu_score_floor', 0.6)
        self.declare_parameter('imu_min_severity', 'medium')  # weak 피크는 무시

        # 소리는 floor를 주지 않는다. 방향 일치는 그 자체로 증거가 아니라
        # "그 방향에 사람이 있다"는 사실의 재확인에 가깝기 때문.
        # 대신 최소 음량 게이트로 생활 소음 수준은 걸러낸다.
        self.declare_parameter('sound_min_confidence', 0.5)

        # ── 시간 정합 창 (낙상 시작 시각 t_onset 기준) ──
        # 소리는 넘어지는 순간(비명/충돌음)에 나므로 대체로 동시~직후,
        # IMU는 BLE 큐잉 때문에 수백 ms 늦게 도착하는 일이 있어 뒤를 넓게 잡는다.
        self.declare_parameter('sound_window_pre_sec', 2.0)
        self.declare_parameter('sound_window_post_sec', 3.0)
        self.declare_parameter('imu_window_pre_sec', 2.5)
        self.declare_parameter('imu_window_post_sec', 4.0)
        self.declare_parameter('evidence_buffer_sec', 30.0)

        # ── 공간 정합 ──
        self.declare_parameter('assoc_radius', 1.2)          # 사건↔트랙 동일성 반경 (m)
        self.declare_parameter('sound_match_radius', 1.5)    # 소리 위치↔낙상 위치 (m)
        self.declare_parameter('sound_angle_tolerance_deg', 35.0)  # track_id=-1(방향만)일 때

        # ── 사후 관찰(미탐 구제) ──
        self.declare_parameter('lying_verticality_max', 0.45)
        self.declare_parameter('lying_height_max', 0.60)     # verticality 미계산 시 sz 대체 기준
        self.declare_parameter('still_speed_max', 0.15)      # m/s, 이 아래면 정지
        self.declare_parameter('critical_lying_sec', 10.0)   # 계속 누워있음 → 위급
        self.declare_parameter('critical_missing_sec', 6.0)  # bbox 소실 지속 → 위급

        # ── 소리 가속 (점수가 아닌 '대기 시간 단축'으로만 기여) ──
        self.declare_parameter('sound_escalation_enabled', True)
        self.declare_parameter('sound_escalation_window_sec', 8.0)  # 최근 이 구간의 소리를 봄
        self.declare_parameter('sound_escalation_min_hits', 5)      # 이만큼 매칭돼야 유효
        self.declare_parameter('sound_lying_speedup', 0.5)          # critical_lying_sec × 이 비율
        self.declare_parameter('standing_verticality_min', 0.65)
        self.declare_parameter('exit_speed_min', 0.30)       # 걸어 나간 것으로 볼 최소 속도
        self.declare_parameter('recovery_hold_sec', 1.5)     # 기립이 이만큼 유지되면 회복
        self.declare_parameter('tracks_stale_timeout_sec', 2.0)

        # ── 알림 ──
        self.declare_parameter('confirmed_repeat_sec', 5.0)
        self.declare_parameter('critical_repeat_sec', 3.0)
        self.declare_parameter('max_alert_repeat', 0)        # 0 = 무한 반복
        self.declare_parameter('case_expire_sec', 300.0)     # 종료된 사건 정리
        self.declare_parameter('case_idle_expire_sec', 60.0) # 증거 없이 방치된 OBSERVING 정리

        self.frame_id = self.get_parameter('frame_id').value

        # ── 상태 ──
        self._cases: list[FallCase] = []
        self._sound_buf: deque[SoundSample] = deque()
        self._imu_buf: deque[ImuSample] = deque()
        self._tracks: list[TrackSample] = []
        self._tracks_t = None

        # ── 구독 ──
        self.create_subscription(
            Float32MultiArray, self.get_parameter('fall_events_topic').value,
            self._on_fall_events, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('sound_track_topic').value,
            self._on_sound_track, 10)
        self.create_subscription(
            String, self.get_parameter('imu_evidence_topic').value,
            self._on_imu_evidence, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('human_tracks_topic').value,
            self._on_human_tracks, 10)
        self.create_subscription(
            String, self.get_parameter('ack_topic').value, self._on_ack, 10)

        # ── 발행 ──
        self.pub_alert = self.create_publisher(
            String, self.get_parameter('alert_topic').value, 10)
        self.pub_state = self.create_publisher(
            Float32MultiArray, self.get_parameter('state_topic').value, 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, self.get_parameter('marker_topic').value, 10)

        rate = float(self.get_parameter('eval_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'FallFusionNode 시작 | '
            f'w=({self.get_parameter("w_lidar").value},'
            f'{self.get_parameter("w_sound").value},'
            f'{self.get_parameter("w_imu").value}) '
            f'confirm={self.get_parameter("confirm_threshold").value} '
            f'pending={self.get_parameter("pending_threshold").value} | '
            f'위급승격: 누움{self.get_parameter("critical_lying_sec").value}s / '
            f'소실{self.get_parameter("critical_missing_sec").value}s | '
            f'소리가속={"on" if self.get_parameter("sound_escalation_enabled").value else "off"}'
        )
        if (self.get_parameter('w_lidar').value
                + self.get_parameter('w_sound').value
                < self.get_parameter('confirm_threshold').value):
            self.get_logger().info(
                '판정 구조: IMU 증거 없이는 confirm_threshold를 넘을 수 없음 '
                '(소리는 점수 표시 + 위급승격 가속에만 기여).')
        else:
            self.get_logger().warn(
                'w_lidar+w_sound가 confirm_threshold 이상입니다 — IMU 없이 '
                '소리만으로 낙상이 확정될 수 있습니다. 소리는 비명/구조요청을 '
                '구분하지 못하므로 오탐이 크게 늘 수 있습니다.')

    # ════════════════════════════════════════════════════════
    # 시간
    # ════════════════════════════════════════════════════════
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ════════════════════════════════════════════════════════
    # 콜백
    # ════════════════════════════════════════════════════════
    def _on_fall_events(self, msg: Float32MultiArray):
        """LiDAR 낙상 이벤트 = 사건 생성/갱신의 앵커."""
        now = self._now()
        data = np.asarray(msg.data, dtype=np.float64)

        for i in range(0, len(data) - 5, 6):
            tid = int(data[i])
            pos = data[i + 1:i + 4].copy()
            conf = float(data[i + 4])
            state = int(data[i + 5])

            if state == SC_ACTIVE:
                # 정상 이동 중 → 낙상 아님. 근처 사건이 있으면 회복 힌트로만 사용.
                continue

            if state == SC_RECOVERED:
                case = self._find_case(tid, pos)
                if case is not None and case.level != LEVEL_RESOLVED:
                    case.recovered_event = True
                    self._resolve(case, '회복 이벤트 수신', now)
                continue

            if state not in (SC_CANDIDATE, SC_CONFIRMED):
                continue

            case = self._find_case(tid, pos)
            if case is None:
                case = FallCase(tid, pos, now)
                self._cases.append(case)
                self.get_logger().warn(
                    f'[CASE {case.case_id}] 신규 낙상 사건 개시 '
                    f'(track {tid}, pos=({pos[0]:.2f},{pos[1]:.2f}))')
            elif case.level == LEVEL_RESOLVED:
                # 종료된 사건 위치에서 다시 낙상 → 새 사건으로 취급
                case = FallCase(tid, pos, now)
                self._cases.append(case)
                self.get_logger().warn(
                    f'[CASE {case.case_id}] 동일 위치 재낙상 → 새 사건 개시')

            # 앵커 갱신
            case.track_id = tid
            case.pos = pos
            case.t_lidar = now
            case.lidar_state = max(case.lidar_state, state)
            case.lidar_score = max(
                case.lidar_score,
                self.get_parameter('lidar_confirmed_score').value
                if state == SC_CONFIRMED
                else self.get_parameter('lidar_candidate_score').value)
            # CANDIDATE→CONFIRMED로 올라올 때 confidence가 더 크면 반영
            case.lidar_score = max(case.lidar_score, min(conf, 1.0))

    def _on_sound_track(self, msg: Float32MultiArray):
        """소리 발생원 트랙. 빈 배열이면 '지금 유효한 소리 없음'."""
        data = np.asarray(msg.data, dtype=np.float64)
        if len(data) < 5:
            return
        self._sound_buf.append(SoundSample(
            self._now(), int(data[0]), data[1:4].copy(), float(data[4])))

    def _on_imu_evidence(self, msg: String):
        """imu_fall_sos_node의 fall-evidence JSON."""
        now = self._now()
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'IMU evidence JSON 파싱 실패: {e}')
            return

        if payload.get('event_type') != 'fall_evidence':
            return

        md = payload.get('modality_data', {}) or {}
        self._imu_buf.append(ImuSample(
            t=now,
            confidence=float(payload.get('confidence', 0.0)),
            severity=payload.get('severity', 'weak'),
            peak_ratio=float(md.get('peak_ratio', 0.0)),
            raw=payload,
        ))
        self.get_logger().info(
            f'[IMU 증거] conf={payload.get("confidence")} '
            f'severity={payload.get("severity")} '
            f'ratio={md.get("peak_ratio")}')

    def _on_human_tracks(self, msg: Float32MultiArray):
        """사후 관찰용 원본 트랙. fall_detection은 CANDIDATE 이상만 발행하므로
        '누워있는지 / 사라졌는지'는 여기서 직접 봐야 한다."""
        data = np.asarray(msg.data, dtype=np.float64)
        tracks = []
        for i in range(0, len(data) - 8, 9):
            tracks.append(TrackSample(
                track_id=data[i],
                center=data[i + 1:i + 4].copy(),
                size=data[i + 4:i + 7].copy(),
                verticality=data[i + 7],
                planarity=data[i + 8],
            ))
        self._tracks = tracks
        self._tracks_t = self._now()

    def _on_ack(self, msg: String):
        """알림 확인 → 반복 중지. 'all' 또는 case_id 숫자/JSON."""
        raw = (msg.data or '').strip()
        target = None
        if raw.lower() in ('all', '*'):
            target = 'all'
        else:
            try:
                target = int(json.loads(raw)['case_id'])
            except Exception:
                try:
                    target = int(float(raw))
                except ValueError:
                    self.get_logger().warn(f'ack 파싱 실패: "{raw}"')
                    return

        for case in self._cases:
            if target == 'all' or case.case_id == target:
                case.acked = True
                self.get_logger().info(f'[CASE {case.case_id}] 알림 확인(ack) — 반복 중지')

    # ════════════════════════════════════════════════════════
    # 사건 ↔ 관측 연결
    # ════════════════════════════════════════════════════════
    def _find_case(self, track_id: int, pos: np.ndarray) -> FallCase | None:
        """ID 일치를 우선하되, 없으면 위치로 찾는다."""
        radius = self.get_parameter('assoc_radius').value

        for case in self._cases:
            if case.level != LEVEL_RESOLVED and case.track_id == track_id:
                return case

        best, best_d = None, radius
        for case in self._cases:
            if case.level == LEVEL_RESOLVED:
                continue
            d = xy_dist(case.pos, pos)
            if d < best_d:
                best, best_d = case, d
        return best

    def _nearest_track(self, case: FallCase) -> TrackSample | None:
        radius = self.get_parameter('assoc_radius').value
        best, best_d = None, radius
        for tr in self._tracks:
            if tr.track_id == case.track_id:
                return tr
            d = xy_dist(case.pos, tr.center)
            if d < best_d:
                best, best_d = tr, d
        return best

    # ════════════════════════════════════════════════════════
    # 증거 정합
    # ════════════════════════════════════════════════════════
    def _sound_matches_case(self, s: SoundSample, case: FallCase):
        """이 소리 샘플이 사건 위치에서 난 것으로 볼 수 있는가.
        (매칭 여부, 각도 오차) 반환. 매칭 실패 시 (False, None).

        음량 게이트를 여기서 함께 건다 — 생활 소음 수준의 상시 발행을
        증거로 취급하지 않기 위함.
        """
        if s.confidence < self.get_parameter('sound_min_confidence').value:
            return False, None

        radius = self.get_parameter('sound_match_radius').value
        tol = self.get_parameter('sound_angle_tolerance_deg').value
        case_angle = angle_of(case.pos[0], case.pos[1])

        if s.track_id == case.track_id and s.track_id >= 0:
            # 소리 노드가 같은 트랙에 매칭시킴 = 가장 확실한 일치
            return True, 0.0

        if s.track_id >= 0 and xy_dist(s.pos, case.pos) <= radius:
            # 트랙 ID는 다르지만(재할당 등) 위치가 사실상 같은 지점
            return True, abs(wrap180(angle_of(s.pos[0], s.pos[1]) - case_angle))

        # track_id=-1(방향만 유효) → 각도만 비교.
        # 이 fallback은 낙상 순간 bbox가 깨져 트랙 매칭이 안 되는
        # 상황에서 오히려 자주 나오므로 반드시 받아줘야 한다.
        err = abs(wrap180(angle_of(s.pos[0], s.pos[1]) - case_angle))
        if err <= tol:
            return True, err
        return False, None

    def _match_sound(self, case: FallCase, now: float):
        """낙상 시각 창 안에서, 낙상 위치와 방향이 일치하는 소리가 있었는가.

        주의: 이 점수는 w_sound가 작아서(기본 0.10) 최종 확정 판정을
        뒤집지 못한다. 점수 표시와 로깅, 그리고 나중에 소리 분류기가
        붙었을 때를 위한 자리로만 유지한다.
        """
        pre = self.get_parameter('sound_window_pre_sec').value
        post = self.get_parameter('sound_window_post_sec').value
        lo, hi = case.t_onset - pre, case.t_onset + post

        for s in self._sound_buf:
            if not (lo <= s.t <= hi):
                continue
            matched, angle_err = self._sound_matches_case(s, case)
            if not matched:
                continue
            score = min(s.confidence, 1.0)
            if score > case.sound_score:
                case.sound_score = score
                case.t_sound = s.t
                case.sound_angle_err = angle_err

    def _recent_sound_hits(self, case: FallCase, now: float) -> int:
        """최근 sound_escalation_window_sec 동안 사건 위치에서 난 소리 개수.

        낙상 시각 창과 달리 '지금' 기준이다 — 누운 뒤에도 계속 소리가
        나는지를 보기 위한 것이므로 시간 앵커가 다르다.
        """
        window = self.get_parameter('sound_escalation_window_sec').value
        lo = now - window
        hits = 0
        for s in self._sound_buf:
            if s.t < lo:
                continue
            matched, _ = self._sound_matches_case(s, case)
            if matched:
                hits += 1
        return hits

    def _match_imu(self, case: FallCase, now: float):
        """낙상 시각 창 안에서 충격 피크가 임계를 넘었는가.

        IMU는 위치 정보가 없으므로(웨어러블/바닥 단일 센서) 시간 정합만 한다.
        대신 severity 게이트로 약한 피크(발소리·문 닫힘)를 걸러낸다.
        """
        pre = self.get_parameter('imu_window_pre_sec').value
        post = self.get_parameter('imu_window_post_sec').value
        floor = self.get_parameter('imu_score_floor').value
        min_rank = SEVERITY_RANK.get(
            str(self.get_parameter('imu_min_severity').value).lower(), 1)

        lo, hi = case.t_onset - pre, case.t_onset + post

        for s in self._imu_buf:
            if not (lo <= s.t <= hi):
                continue
            if SEVERITY_RANK.get(s.severity.lower(), 0) < min_rank:
                continue
            score = max(floor, min(s.confidence, 1.0))
            if score > case.imu_score:
                case.imu_score = score
                case.t_imu = s.t
                case.imu_severity = s.severity
                case.imu_peak_ratio = s.peak_ratio

    def _fuse(self, case: FallCase) -> float:
        wl = self.get_parameter('w_lidar').value
        ws = self.get_parameter('w_sound').value
        wi = self.get_parameter('w_imu').value
        total = wl + ws + wi
        if total <= 0.0:
            return 0.0
        score = (wl * case.lidar_score
                 + ws * case.sound_score
                 + wi * case.imu_score) / total
        return float(np.clip(score, 0.0, 1.0))

    # ════════════════════════════════════════════════════════
    # 사후 관찰 (미탐 구제)
    # ════════════════════════════════════════════════════════
    def _observe(self, case: FallCase, now: float):
        """낙상 지점의 사람 트랙을 계속 보면서
        (a) 계속 누워있는지 (b) 아예 사라졌는지 (c) 다시 일어났는지 판단."""
        stale_to = self.get_parameter('tracks_stale_timeout_sec').value
        tracks_fresh = (self._tracks_t is not None
                        and (now - self._tracks_t) <= stale_to)

        if not tracks_fresh:
            # 상위 노드(human_bbox)가 죽었거나 발행이 끊긴 상태.
            # 이걸 "사람이 사라졌다"로 해석하면 노드 장애가 곧 위급 알림이 된다.
            # 타이머를 진행시키지 않고 그대로 얼려둔다.
            return

        tr = self._nearest_track(case)

        if tr is None:
            # ── (b) bbox 소실 ──
            if case.missing_since is None:
                case.missing_since = now
                self.get_logger().warn(
                    f'[CASE {case.case_id}] 낙상 위치에서 사람 트랙 소실 — 관찰 시작')
            case.lying_since = None
            case.upright_since = None
            return

        # 트랙이 보임 → 소실 타이머 해제
        if case.missing_since is not None:
            self.get_logger().info(
                f'[CASE {case.case_id}] 트랙 재획득 (소실 '
                f'{case.missing_duration(now):.1f}s)')
        case.missing_since = None
        case.seen_once = True
        case.last_seen_t = now
        case.pos = tr.center.copy()      # 앵커를 현재 위치로 따라가게
        case.track_id = tr.track_id

        # 속도 추정 (프레임 간 튐을 줄이려 0.5s 이상 벌어진 두 샘플로 계산)
        case.motion.append((now, tr.center.copy()))
        speed = 0.0
        for t_old, c_old in case.motion:
            dt = now - t_old
            if dt >= 0.5:
                speed = xy_dist(tr.center, c_old) / dt
                break
        case.last_speed = speed

        # 누움 여부: verticality 우선, 미계산이면 bbox 높이로 대체
        v_max = self.get_parameter('lying_verticality_max').value
        h_max = self.get_parameter('lying_height_max').value
        if tr.verticality is not None:
            case.last_verticality = tr.verticality
            is_lying = tr.verticality <= v_max
        else:
            is_lying = float(tr.size[2]) <= h_max

        still = speed <= self.get_parameter('still_speed_max').value

        # ── (a) 누움 + 정지 지속 ──
        if is_lying and still:
            if case.lying_since is None:
                case.lying_since = now
        else:
            case.lying_since = None

        # ── (c) 기립 회복 ──
        stand_min = self.get_parameter('standing_verticality_min').value
        upright = (tr.verticality is not None and tr.verticality >= stand_min)
        if upright:
            if case.upright_since is None:
                case.upright_since = now
        else:
            case.upright_since = None

    # ════════════════════════════════════════════════════════
    # 판정
    # ════════════════════════════════════════════════════════
    def _decide(self, case: FallCase, now: float):
        confirm_th = self.get_parameter('confirm_threshold').value
        pending_th = self.get_parameter('pending_threshold').value
        hold = self.get_parameter('recovery_hold_sec').value

        # ── 회복이 최우선. 일어나서 유지되면 어떤 레벨이든 종료 ──
        if (case.upright_since is not None
                and (now - case.upright_since) >= hold):
            if case.level != LEVEL_RESOLVED:
                self._resolve(case, '기립 자세 회복 확인', now)
            return

        # ── 1차: 융합 점수 ──
        if case.fused >= confirm_th and case.level < LEVEL_CONFIRMED:
            self._escalate(
                case, LEVEL_CONFIRMED,
                f'융합 점수 {case.fused:.2f} ≥ {confirm_th:.2f} '
                f'(lidar/sound/imu = {case.lidar_score:.2f}/'
                f'{case.sound_score:.2f}/{case.imu_score:.2f})', now)
        elif case.fused >= pending_th and case.level == LEVEL_OBSERVING:
            self._escalate(
                case, LEVEL_PENDING,
                f'융합 점수 {case.fused:.2f} — 확정 임계 미달, 사후 관찰 진입 '
                f'(IMU 증거 {"있음" if case.imu_score > 0 else "없음"})', now)

        # ── 2차: 미탐 구제 — 사후 행동으로 위급 승격 ──
        # PENDING(=0.75 구간)뿐 아니라 CONFIRMED에서도 적용된다.
        # 확정 낙상이어도 계속 못 일어나면 그건 더 위급한 상황이다.
        if case.level in (LEVEL_PENDING, LEVEL_CONFIRMED):
            lying_lim = self.get_parameter('critical_lying_sec').value
            miss_lim = self.get_parameter('critical_missing_sec').value

            # ── 소리 가속 ──
            # 소리를 점수에 더하지 않는 대신 여기서만 쓴다.
            # 이미 '누워 있음'이 성립한 뒤의 조건부라서, 상시 발행되는
            # 소리라도 오탐 비용이 "정상인을 낙상으로 만드는 것"이 아니라
            # "이미 누운 사람의 알림을 몇 초 앞당기는 것"에 그친다.
            sound_hits = 0
            if (self.get_parameter('sound_escalation_enabled').value
                    and case.lying_since is not None):
                sound_hits = self._recent_sound_hits(case, now)
                if sound_hits >= self.get_parameter(
                        'sound_escalation_min_hits').value:
                    lying_lim *= self.get_parameter('sound_lying_speedup').value
            case.sound_hits_recent = sound_hits
            case.lying_limit_used = lying_lim

            if case.lying_duration(now) >= lying_lim:
                accel = ' + 해당 위치에서 소리 지속'if sound_hits else ''
                self._escalate(
                    case, LEVEL_CRITICAL,
                    f'낙상 후 {case.lying_duration(now):.0f}초간 누운 채 '
                    f'움직이지 못함 (long lie{accel})', now)
                return

            if case.missing_duration(now) >= miss_lim:
                stand_min = self.get_parameter('standing_verticality_min').value
                exit_spd = self.get_parameter('exit_speed_min').value
                if case.looks_like_walkaway(stand_min, exit_spd):
                    # 일어나서 걸어 나간 것 → 위급 아님
                    self._resolve(
                        case,
                        '기립 후 이동하여 관측 범위 이탈 (회복으로 판단)', now)
                else:
                    # 쓰러진 자세 그대로 관측이 끊김 → 가장 위험한 케이스
                    self._escalate(
                        case, LEVEL_CRITICAL,
                        f'낙상 후 {case.missing_duration(now):.0f}초간 사람 트랙이 '
                        f'재검출되지 않음 (bbox 소실, '
                        f'마지막 verticality='
                        f'{case.last_verticality if case.last_verticality is not None else "N/A"})',
                        now)

    def _escalate(self, case: FallCase, level: int, reason: str, now: float):
        if level <= case.level:
            return
        case.level = level
        case.t_level = now
        case.escalation_reason = reason
        # 레벨이 올라가면 이전 ack는 무효 — 더 위급해졌으므로 다시 알린다.
        case.acked = False
        case.t_last_alert = None

        msg = f'[CASE {case.case_id}] → {LEVEL_NAME[level].upper()} : {reason}'
        if level >= LEVEL_CRITICAL:
            self.get_logger().error(msg)
        else:
            self.get_logger().warn(msg)

    def _resolve(self, case: FallCase, reason: str, now: float):
        case.level = LEVEL_RESOLVED
        case.t_level = now
        case.escalation_reason = reason
        self.get_logger().info(f'[CASE {case.case_id}] 종료 : {reason}')
        self._publish_alert(case, now, final=True)

    # ════════════════════════════════════════════════════════
    # 메인 루프
    # ════════════════════════════════════════════════════════
    def _tick(self):
        now = self._now()
        self._purge_buffers(now)

        for case in self._cases:
            if case.level == LEVEL_RESOLVED:
                continue

            # 증거 정합 → 점수
            self._match_sound(case, now)
            self._match_imu(case, now)
            case.fused = self._fuse(case)

            # 사후 관찰 → 판정
            self._observe(case, now)
            self._decide(case, now)

            # 알림
            self._maybe_alert(case, now)

        self._expire_cases(now)
        self._publish_state()
        self._publish_markers()

    def _purge_buffers(self, now: float):
        keep = self.get_parameter('evidence_buffer_sec').value
        while self._sound_buf and (now - self._sound_buf[0].t) > keep:
            self._sound_buf.popleft()
        while self._imu_buf and (now - self._imu_buf[0].t) > keep:
            self._imu_buf.popleft()

    def _expire_cases(self, now: float):
        exp = self.get_parameter('case_expire_sec').value
        idle = self.get_parameter('case_idle_expire_sec').value
        kept = []
        for case in self._cases:
            if case.level == LEVEL_RESOLVED and (now - case.t_level) > exp:
                continue
            # 증거가 더 붙지 않고 LiDAR 갱신도 끊긴 OBSERVING 사건 정리
            if (case.level == LEVEL_OBSERVING
                    and (now - case.t_lidar) > idle):
                self.get_logger().info(
                    f'[CASE {case.case_id}] 증거 부족으로 자동 폐기 '
                    f'(score={case.fused:.2f})')
                continue
            kept.append(case)
        self._cases = kept

    # ════════════════════════════════════════════════════════
    # 알림
    # ════════════════════════════════════════════════════════
    def _maybe_alert(self, case: FallCase, now: float):
        if case.level < LEVEL_CONFIRMED:
            return
        if case.acked:
            return

        period = (self.get_parameter('critical_repeat_sec').value
                  if case.level == LEVEL_CRITICAL
                  else self.get_parameter('confirmed_repeat_sec').value)

        max_rep = int(self.get_parameter('max_alert_repeat').value)
        if max_rep > 0 and case.alert_count >= max_rep:
            return

        if case.t_last_alert is not None and (now - case.t_last_alert) < period:
            return

        self._publish_alert(case, now)

    def _publish_alert(self, case: FallCase, now: float, final: bool = False):
        stamp = self.get_clock().now().to_msg()

        if final:
            severity = 'info'
        elif case.level == LEVEL_CRITICAL:
            severity = 'critical'
        else:
            severity = 'high'

        payload = {
            'schema_version': 1,
            'event_type': 'fall_alert',
            'modality': 'fusion',
            'case_id': case.case_id,
            'track_id': case.track_id,

            'ros_stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},

            'level': LEVEL_NAME[case.level],
            'severity': severity,
            'reason': case.escalation_reason,
            'confidence': round(case.fused, 3),
            'fused_score': round(case.fused, 3),
            'repeat_count': case.alert_count,
            'elapsed_since_fall_sec': round(now - case.t_onset, 2),

            'frame_id': self.frame_id,
            'position': {
                'x': float(case.pos[0]),
                'y': float(case.pos[1]),
                'z': float(case.pos[2]),
            },

            'evidence': {
                'lidar': {
                    'present': case.lidar_score > 0.0,
                    'state': ('confirmed' if case.lidar_state == SC_CONFIRMED
                              else 'candidate'),
                    'score': round(case.lidar_score, 3),
                    'weighted': round(
                        case.lidar_score * self.get_parameter('w_lidar').value, 3),
                },
                'sound': {
                    'present': case.sound_score > 0.0,
                    'score': round(case.sound_score, 3),
                    'weighted': round(
                        case.sound_score * self.get_parameter('w_sound').value, 3),
                    'angle_error_deg': (round(case.sound_angle_err, 1)
                                        if case.sound_angle_err is not None else None),
                    'offset_sec': (round(case.t_sound - case.t_onset, 2)
                                   if case.t_sound is not None else None),
                },
                'imu': {
                    'present': case.imu_score > 0.0,
                    'score': round(case.imu_score, 3),
                    'weighted': round(
                        case.imu_score * self.get_parameter('w_imu').value, 3),
                    'severity': case.imu_severity,
                    'peak_ratio': case.imu_peak_ratio,
                    'offset_sec': (round(case.t_imu - case.t_onset, 2)
                                   if case.t_imu is not None else None),
                },
            },

            # 미탐 구제 경로가 왜 발동했는지 추적 가능하게 남긴다
            'post_fall': {
                'lying_duration_sec': round(case.lying_duration(now), 1),
                'track_lost_duration_sec': round(case.missing_duration(now), 1),
                'last_verticality': case.last_verticality,
                'last_speed_mps': round(case.last_speed, 3),
                'imu_missing': case.imu_score <= 0.0,
                'sound_hits_recent': case.sound_hits_recent,
                'lying_limit_sec': (round(case.lying_limit_used, 1)
                                    if case.lying_limit_used is not None else None),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.pub_alert.publish(msg)

        if not final:
            case.alert_count += 1
            case.t_last_alert = now
            alert_msg = (
                f'[FALL ALERT #{case.alert_count}] case {case.case_id} '
                f'{LEVEL_NAME[case.level].upper()} score={case.fused:.2f} '
                f'pos=({case.pos[0]:.2f},{case.pos[1]:.2f}) — '
                f'{case.escalation_reason}'
            )
            if case.level == LEVEL_CRITICAL:
                self.get_logger().error(alert_msg)
            else:
                self.get_logger().warn(alert_msg)

    # ════════════════════════════════════════════════════════
    # 발행 (상태 / 마커)
    # ════════════════════════════════════════════════════════
    def _publish_state(self):
        out = Float32MultiArray()
        data = []
        for case in self._cases:
            if case.level in (LEVEL_OBSERVING, LEVEL_RESOLVED):
                continue
            data.extend([
                float(case.case_id),
                float(case.pos[0]), float(case.pos[1]), float(case.pos[2]),
                float(case.fused), float(case.level),
            ])
        out.data = data
        self.pub_state.publish(out)

    def _publish_markers(self):
        ma = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        idx = 0
        for case in self._cases:
            if case.level in (LEVEL_OBSERVING, LEVEL_RESOLVED):
                continue

            if case.level == LEVEL_CRITICAL:
                color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=0.9)
                label = '위급 낙상'
            elif case.level == LEVEL_CONFIRMED:
                color = ColorRGBA(r=1.0, g=0.35, b=0.0, a=0.8)
                label = '낙상 확정'
            else:
                color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.6)
                label = '낙상 보류'

            cyl = Marker()
            cyl.header.frame_id = self.frame_id
            cyl.header.stamp = stamp
            cyl.ns = 'fall_fusion'
            cyl.id = idx
            idx += 1
            cyl.type = Marker.CYLINDER
            cyl.action = Marker.ADD
            cyl.pose.position = Point(
                x=float(case.pos[0]), y=float(case.pos[1]), z=float(case.pos[2]))
            cyl.pose.orientation.w = 1.0
            r = 0.9 if case.level == LEVEL_CRITICAL else 0.6
            cyl.scale.x = r
            cyl.scale.y = r
            cyl.scale.z = 0.15
            cyl.color = color
            ma.markers.append(cyl)

            txt = Marker()
            txt.header.frame_id = self.frame_id
            txt.header.stamp = stamp
            txt.ns = 'fall_fusion_text'
            txt.id = idx
            idx += 1
            txt.type = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position = Point(
                x=float(case.pos[0]), y=float(case.pos[1]),
                z=float(case.pos[2]) + 0.9)
            txt.pose.orientation.w = 1.0
            txt.scale.z = 0.28
            txt.color = color
            mods = ''.join([
                'L' if case.lidar_score > 0 else '-',
                'S' if case.sound_score > 0 else '-',
                'I' if case.imu_score > 0 else '-',
            ])
            txt.text = (f'{label} #{case.case_id}\n'
                        f'{case.fused:.2f} [{mods}]')
            ma.markers.append(txt)

        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FallFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
