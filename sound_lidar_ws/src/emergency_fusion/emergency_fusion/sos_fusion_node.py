"""
emergency_fusion/sos_fusion_node.py
────────────────────────────────────────────────────────────────────────
목적:
  구조신호(SOS) 태스크의 최종 판단자. 서로 독립적인 두 모달리티가
  같은 시간대에 동시에 SOS를 주장할 때만 최종 확정한다.

    소리 : 오디오 분류기가 'emergency' 라벨을 낸다 (팀원 구현)
    IMU  : imu_fall_sos_node가 바닥 두드림 패턴을 잡아 /sos/detected 발행

  ── 판정 규칙 : AND ────────────────────────────────────────
    두 증거가 match_window_sec 안에 모두 들어와야 CONFIRMED.
    도착 순서는 상관없다 (소리 먼저든 IMU 먼저든 동일).

    낙상 태스크와 달리 가중 점수를 쓰지 않는다. 이유:
      · 모달리티가 2개뿐이라 가중치를 어떻게 잡아도 결정 함수는
        결국 AND / OR / 단독확정 셋 중 하나로 축약된다.
        중간값을 두면 튜닝 여지가 있는 것처럼 보이지만 실제로는 없다.
      · 두 신호 모두 단독 오탐률이 높다.
        소리: TV 비명, 게임 소리, 아이 장난
        IMU : 문 쾅 닫힘, 물건 떨어뜨림, 청소기
        하나만으로 확정하면 오탐이 실용 불가 수준이 된다.

    다만 한쪽만 온 상태를 버리지 않고 PENDING으로 남겨 로그와
    /emergency/sos_state에 노출한다. 확정은 안 되지만, 어느 쪽이
    자주 헛도는지 튜닝할 때 이 기록이 필요하다.

  ── 소리 쪽 입력 ───────────────────────────────────────────
    팀원의 emergency_detector_node가 두 토픽을 발행한다.

      /emergency_detector/alarm  (std_msgs/Bool)
          최근 5개 3초 윈도우 중 60% 이상이 emergency일 때 True.
          단발 오분류 필터링을 그 노드가 이미 하므로, 여기서 다시
          횟수를 세지 않고 True를 그대로 증거로 받는다.

      /emergency_detector/probs  (std_msgs/String, JSON)
          {"emergency": 0.82, "normal_speech": 0.1, "background": 0.08}
          매 윈도우마다 발행. 알람의 확신도로만 쓰고 판정에는 쓰지 않는다.

    분류기 구현이 바뀌어 라벨 문자열을 던지는 형태가 되어도 받을 수
    있도록, String 파서는 아래 형태를 모두 처리한다.

      "emergency"
      {"label": "emergency", "confidence": 0.93}
      {"class": "emergency", "score": 0.93}
      {"emergency": 0.82, "normal_speech": 0.1}     ← 최상위 클래스-확률
      {"probs": {"emergency": 0.82, ...}}

입력:
  /emergency_detector/alarm  (std_msgs/Bool)    소리 SOS 확정 신호
  /emergency_detector/probs  (std_msgs/String)  클래스별 확률 (확신도용)
  /sos/detected              (std_msgs/String)  imu_fall_sos_node의 SOS JSON
  /emergency/sos_ack         (std_msgs/String)  "all" 또는 case_id → 알림 중지

출력:
  /emergency/sos_alert (std_msgs/String, JSON)  최종 SOS 알림 (반복 발행)
  /emergency/sos_state (std_msgs/Float32MultiArray)
        4개씩 [case_id, level_code, sound_ok, imu_ok]
"""

from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String


LEVEL_IDLE = 0
LEVEL_PENDING = 1      # 한쪽만 들어옴
LEVEL_CONFIRMED = 2    # 양쪽 다 들어옴
LEVEL_RESOLVED = 3     # ack 또는 만료

LEVEL_NAME = {
    LEVEL_IDLE: 'idle',
    LEVEL_PENDING: 'pending',
    LEVEL_CONFIRMED: 'confirmed',
    LEVEL_RESOLVED: 'resolved',
}

DEFAULT_LABEL_KEYS = ['label', 'class', 'prediction', 'event', 'result']
DEFAULT_SCORE_KEYS = ['confidence', 'score', 'prob', 'probability']


class Evidence:
    """한 모달리티의 최신 증거."""

    __slots__ = ('t', 'confidence', 'raw')

    def __init__(self, t, confidence, raw):
        self.t = t
        self.confidence = float(confidence)
        self.raw = raw


class SosFusionNode(Node):

    def __init__(self):
        super().__init__('sos_fusion_node')

        # ── 토픽 ──
        self.declare_parameter('sound_alarm_topic', '/emergency_detector/alarm')
        self.declare_parameter('sound_probs_topic', '/emergency_detector/probs')
        # 분류기가 라벨 문자열을 던지는 형태로 바뀌면 이 토픽으로 받는다.
        # 비워두면('') 구독하지 않는다.
        self.declare_parameter('sound_label_topic', '')
        self.declare_parameter('imu_topic', '/sos/detected')
        self.declare_parameter('ack_topic', '/emergency/sos_ack')
        self.declare_parameter('alert_topic', '/emergency/sos_alert')
        self.declare_parameter('state_topic', '/emergency/sos_state')
        self.declare_parameter('eval_rate', 5.0)

        # ── 소리 입력 파싱 ──
        self.declare_parameter('sound_emergency_label', 'emergency')
        self.declare_parameter('sound_label_keys', DEFAULT_LABEL_KEYS)
        self.declare_parameter('sound_score_keys', DEFAULT_SCORE_KEYS)
        # 라벨 토픽으로 받을 때만 쓰는 게이트. Bool 알람 경로에는 적용되지
        # 않는다 — emergency_detector_node가 이미 자체 임계값
        # (emergency_threshold)과 5-윈도우 과반 투표를 통과시킨 결과라
        # 여기서 다시 거르면 이중 필터가 된다.
        self.declare_parameter('sound_min_confidence', 0.6)

        # ── IMU 입력 ──
        # imu_fall_sos_node는 sos_detected를 이미 자체 게이트(min_sos_hits,
        # ioi 규칙성 등)를 통과한 뒤에만 발행하므로 여기서 재검증하지 않는다.
        self.declare_parameter('imu_min_confidence', 0.0)

        # ── 융합 ──
        # 두 증거가 이 시간 안에 모두 들어와야 확정.
        # 소리는 사건 순간에 나고 IMU 두드림은 몇 초에 걸쳐 이어지므로
        # 넉넉히 잡는다. 좁히면 "살려주세요" 외친 뒤 두드리기 시작하는
        # 자연스러운 순서를 놓친다.
        self.declare_parameter('match_window_sec', 20.0)
        # 확정 후 양쪽 모두 이 시간 동안 갱신이 없으면 사건 종료.
        self.declare_parameter('case_timeout_sec', 60.0)

        # ── 알림 ──
        self.declare_parameter('repeat_sec', 3.0)
        self.declare_parameter('max_alert_repeat', 0)   # 0 = 무한 반복

        # ── 상태 ──
        self._sound_ev: Evidence | None = None
        self._imu_ev: Evidence | None = None
        self._last_probs = None       # 최신 클래스별 확률 (확신도 표시용)
        self._t_last_probs = None

        self._level = LEVEL_IDLE
        self._case_id = 0
        self._t_level = 0.0
        self._alert_count = 0
        self._t_last_alert = None
        self._acked = False
        self._logged_pending = None

        # ── ROS I/O ──
        self.create_subscription(
            Bool, self.get_parameter('sound_alarm_topic').value,
            self._on_sound_alarm, 10)
        self.create_subscription(
            String, self.get_parameter('sound_probs_topic').value,
            self._on_sound_probs, 10)

        label_topic = self.get_parameter('sound_label_topic').value
        if label_topic:
            self.create_subscription(
                String, label_topic, self._on_sound_label, 10)

        self.create_subscription(
            String, self.get_parameter('imu_topic').value,
            self._on_imu, 10)
        self.create_subscription(
            String, self.get_parameter('ack_topic').value,
            self._on_ack, 10)

        self.pub_alert = self.create_publisher(
            String, self.get_parameter('alert_topic').value, 10)
        self.pub_state = self.create_publisher(
            Float32MultiArray, self.get_parameter('state_topic').value, 10)

        rate = float(self.get_parameter('eval_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            'SosFusionNode 시작 | AND 판정 '
            f'(소리 {self.get_parameter("sound_alarm_topic").value} '
            f'+ IMU {self.get_parameter("imu_topic").value}) | '
            f'정합창 {self.get_parameter("match_window_sec").value}초'
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    # ════════════════════════════════════════════════════════
    # 소리 입력
    # ════════════════════════════════════════════════════════
    def _parse_sound(self, raw: str):
        """분류기 출력에서 (라벨, 확률)을 뽑는다.

        emergency_detector_node의 probs 토픽은 최상위가 곧 클래스-확률
        딕셔너리라({"emergency":0.82,"normal_speech":0.1,...}) label/score
        키가 아예 없다. 이 형태를 놓치면 확신도가 항상 None이 되므로
        최상위 딕셔너리도 클래스 맵으로 인식한다.
        """
        raw = (raw or '').strip()
        if not raw:
            return None, None

        try:
            obj = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return raw.lower(), None

        if not isinstance(obj, dict):
            return str(obj).lower(), None

        label = None
        for k in self.get_parameter('sound_label_keys').value:
            v = obj.get(k)
            if isinstance(v, str):
                label = v.lower()
                break

        score = None
        for k in self.get_parameter('sound_score_keys').value:
            v = obj.get(k)
            if isinstance(v, (int, float)):
                score = float(v)
                break

        if label is None:
            # 중첩된 확률 맵: {"probs": {...}}
            for k in ('probs', 'probabilities', 'scores'):
                v = obj.get(k)
                if isinstance(v, dict) and v:
                    label = max(v, key=v.get).lower()
                    score = float(v[label])
                    return label, score
            # 최상위가 곧 확률 맵인 경우 (emergency_detector_node의 probs)
            if obj and all(isinstance(v, (int, float)) for v in obj.values()):
                label = max(obj, key=obj.get).lower()
                score = float(obj[label])

        return label, score

    def _on_sound_probs(self, msg: String):
        """클래스별 확률 — 판정에는 쓰지 않고 확신도 표시용으로만 보관."""
        label, score = self._parse_sound(msg.data)
        try:
            obj = json.loads(msg.data)
            if isinstance(obj, dict):
                self._last_probs = obj
                self._t_last_probs = self._now()
        except (json.JSONDecodeError, TypeError):
            pass

    def _on_sound_alarm(self, msg: Bool):
        """emergency_detector_node의 알람 신호.

        그 노드가 이미 emergency_threshold + 5윈도우 과반 투표를 통과시킨
        결과이므로, True면 그대로 소리 측 증거로 채택한다. 여기서 다시
        횟수를 세면 이중 필터가 되어 반응이 크게 늦어진다.
        False는 무시한다 — 알람이 꺼졌다고 방금 있었던 외침이 없던 일이
        되는 건 아니고, 만료는 match_window_sec가 담당한다.
        """
        if not msg.data:
            return

        now = self._now()
        target = str(self.get_parameter('sound_emergency_label').value).lower()

        conf = 1.0
        if isinstance(self._last_probs, dict):
            v = self._last_probs.get(target)
            if isinstance(v, (int, float)):
                conf = float(v)

        first = self._sound_ev is None
        self._sound_ev = Evidence(now, conf, {
            'source': 'emergency_detector/alarm',
            'label': target,
            'probs': self._last_probs,
        })
        if first:
            self.get_logger().warn(
                f'[소리 SOS] 분류기 알람 수신 ({target} p={conf:.2f})')

    def _on_sound_label(self, msg: String):
        """라벨 문자열 경로 (분류기 구현이 바뀌었을 때의 대안 입력)."""
        label, score = self._parse_sound(msg.data)
        if label is None:
            return
        target = str(self.get_parameter('sound_emergency_label').value).lower()
        if label != target:
            return
        if score is not None and score < self.get_parameter(
                'sound_min_confidence').value:
            return

        now = self._now()
        first = self._sound_ev is None
        self._sound_ev = Evidence(now, score if score is not None else 1.0, {
            'source': 'label_topic',
            'label': label,
        })
        if first:
            self.get_logger().warn(
                f'[소리 SOS] 라벨 "{label}" 수신 '
                f'(conf={score if score is not None else "n/a"})')

    # ════════════════════════════════════════════════════════
    # IMU 입력
    # ════════════════════════════════════════════════════════
    def _on_imu(self, msg: String):
        now = self._now()
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warn(f'IMU SOS JSON 파싱 실패: {e}')
            return

        if payload.get('event_type') != 'sos_detected':
            return

        conf = float(payload.get('confidence', 0.0))
        if conf < self.get_parameter('imu_min_confidence').value:
            return

        md = payload.get('modality_data', {}) or {}
        seq = md.get('sequence', {}) or {}
        self._imu_ev = Evidence(now, conf, {
            'sos_type': md.get('sos_type'),
            'hit_count': seq.get('hit_count'),
            'duration_sec': seq.get('duration_sec'),
            'sensor_id': payload.get('sensor_id'),
            'severity': payload.get('severity'),
        })
        self.get_logger().warn(
            f'[IMU SOS] 두드림 패턴 감지 (conf={conf:.2f}, '
            f'hits={seq.get("hit_count")})')

    # ════════════════════════════════════════════════════════
    # ack
    # ════════════════════════════════════════════════════════
    def _on_ack(self, msg: String):
        raw = (msg.data or '').strip()
        if not raw:
            return
        if raw.lower() in ('all', '*'):
            ok = True
        else:
            try:
                ok = int(json.loads(raw)['case_id']) == self._case_id
            except Exception:
                try:
                    ok = int(float(raw)) == self._case_id
                except ValueError:
                    self.get_logger().warn(f'ack 파싱 실패: "{raw}"')
                    return
        if ok and self._level == LEVEL_CONFIRMED:
            self._acked = True
            self.get_logger().info(
                f'[CASE {self._case_id}] SOS 알림 확인(ack) — 반복 중지')

    # ════════════════════════════════════════════════════════
    # 메인 루프
    # ════════════════════════════════════════════════════════
    def _tick(self):
        now = self._now()
        self._expire(now)

        sound_ok = self._fresh(self._sound_ev, now)
        imu_ok = self._fresh(self._imu_ev, now)

        if sound_ok and imu_ok:
            if self._level != LEVEL_CONFIRMED:
                self._case_id += 1
                self._level = LEVEL_CONFIRMED
                self._t_level = now
                self._alert_count = 0
                self._t_last_alert = None
                self._acked = False
                gap = abs(self._sound_ev.t - self._imu_ev.t)
                first = '소리' if self._sound_ev.t <= self._imu_ev.t else 'IMU'
                self.get_logger().error(
                    f'[CASE {self._case_id}] SOS 확정 — 소리 + IMU 양쪽 감지 '
                    f'({first}가 먼저, 간격 {gap:.1f}초)')
        elif sound_ok or imu_ok:
            if self._level != LEVEL_PENDING:
                self._level = LEVEL_PENDING
                self._t_level = now
            side = '소리' if sound_ok else 'IMU'
            if self._logged_pending != side:
                self.get_logger().info(
                    f'[보류] {side}만 SOS 주장 중 — 다른 모달리티 대기 '
                    f'(정합창 {self.get_parameter("match_window_sec").value}초). '
                    f'단독으로는 확정하지 않습니다.')
                self._logged_pending = side
        else:
            if self._level != LEVEL_IDLE:
                self._level = LEVEL_IDLE
                self._t_level = now
            self._logged_pending = None

        self._maybe_alert(now, sound_ok, imu_ok)
        self._publish_state(sound_ok, imu_ok)

    def _fresh(self, ev: Evidence | None, now: float) -> bool:
        """증거가 정합창 안에 있는가."""
        if ev is None:
            return False
        return (now - ev.t) <= self.get_parameter('match_window_sec').value

    def _expire(self, now: float):
        """확정 후 양쪽 다 오래 갱신이 없으면 사건 종료."""
        if self._level != LEVEL_CONFIRMED:
            return
        timeout = self.get_parameter('case_timeout_sec').value
        last = max(
            self._sound_ev.t if self._sound_ev else 0.0,
            self._imu_ev.t if self._imu_ev else 0.0)
        if (now - last) > timeout:
            self.get_logger().info(
                f'[CASE {self._case_id}] SOS 종료 — {timeout:.0f}초간 '
                f'양쪽 모두 추가 신호 없음')
            self._level = LEVEL_RESOLVED
            self._t_level = now
            self._sound_ev = None
            self._imu_ev = None

    # ════════════════════════════════════════════════════════
    # 알림
    # ════════════════════════════════════════════════════════
    def _maybe_alert(self, now: float, sound_ok: bool, imu_ok: bool):
        if self._level != LEVEL_CONFIRMED or self._acked:
            return

        max_rep = int(self.get_parameter('max_alert_repeat').value)
        if max_rep > 0 and self._alert_count >= max_rep:
            return

        period = self.get_parameter('repeat_sec').value
        if self._t_last_alert is not None and (now - self._t_last_alert) < period:
            return

        self._publish_alert(now, sound_ok, imu_ok)

    def _publish_alert(self, now: float, sound_ok: bool, imu_ok: bool):
        stamp = self.get_clock().now().to_msg()

        s, i = self._sound_ev, self._imu_ev
        confidence = min(s.confidence, i.confidence) if (s and i) else 0.0

        payload = {
            'schema_version': 1,
            'event_type': 'sos_alert',
            'modality': 'fusion',
            'case_id': self._case_id,

            'ros_stamp': {'sec': int(stamp.sec), 'nanosec': int(stamp.nanosec)},

            'level': LEVEL_NAME[self._level],
            'severity': 'critical',
            'rule': 'sound_emergency AND imu_sos',
            # 두 증거 중 약한 쪽을 최종 확신도로 쓴다. AND 판정이므로
            # 평균이나 최대값을 쓰면 한쪽이 약해도 높게 나와 오해를 준다.
            'confidence': round(confidence, 3),
            'repeat_count': self._alert_count,
            'elapsed_sec': round(now - self._t_level, 2),

            'evidence': {
                'sound': {
                    'present': sound_ok,
                    'confidence': round(s.confidence, 3) if s else None,
                    'age_sec': round(now - s.t, 2) if s else None,
                    'detail': s.raw if s else None,
                },
                'imu': {
                    'present': imu_ok,
                    'confidence': round(i.confidence, 3) if i else None,
                    'age_sec': round(now - i.t, 2) if i else None,
                    'detail': i.raw if i else None,
                },
                'gap_sec': (round(abs(s.t - i.t), 2) if (s and i) else None),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        self.pub_alert.publish(msg)

        self._alert_count += 1
        self._t_last_alert = now
        self.get_logger().error(
            f'[SOS ALERT #{self._alert_count}] case {self._case_id} '
            f'conf={confidence:.2f} — 소리+IMU 구조신호 확정')

    def _publish_state(self, sound_ok: bool, imu_ok: bool):
        out = Float32MultiArray()
        out.data = [
            float(self._case_id),
            float(self._level),
            1.0 if sound_ok else 0.0,
            1.0 if imu_ok else 0.0,
        ]
        self.pub_state.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SosFusionNode()
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
