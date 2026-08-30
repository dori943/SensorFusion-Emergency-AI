"""
sound_source_marker_node.py
─────────────────────────────
목적:
  LiDAR는 항상 360도 전체 스캔으로 human_bbox_node가 사람 트랙을 계속
  추적하고 있는 상태에서, ReSpeaker Mic Array v3.0(sound_localizer_node)이
  보내는 소리 방향(angle)을 받아 "지금 소리가 난 방향에 있는 사람 트랙"을
  찾아 표시만 하는 노드.

  포인트클라우드 자체를 소리 방향으로 잘라내는 방식(lidar_focus_node,
  sound_motion_roi_node)과 달리, 이 노드는 검출/추적 결과에 사후로
  태깅만 하기 때문에 소리가 안 나는 낙상(예: 조용히 주저앉음)도
  놓치지 않는다.

입력:
  /protected_regions  (std_msgs/Float32MultiArray)
      human_bbox_node가 퍼블리시. is_moving과 무관하게 confirmed된
      모든 트랙(정지/낙상 포함)을 담고 있어서 소스로 사용.
      포맷: 7개씩 반복 [cx, cy, cz, sx, sy, sz, track_id]

  /sound_events        (sound_interfaces/SoundEvent)
      sound_localizer_node(ReSpeaker)가 퍼블리시.
      angle(-180~180), confidence, amplitude, is_active

출력:
  /sound_source_track  (std_msgs/Float32MultiArray)
      매칭된 트랙 정보: [track_id, cx, cy, cz, confidence]
      매칭된 트랙이 없으면 빈 배열([]) 퍼블리시.

  /sound_source_marker (visualization_msgs/MarkerArray)
      매칭된 트랙 위치에 강조 표시(링 + 화살표 + 텍스트) — rviz2 확인용.

구현 노트:
  판단/발행 로직을 /protected_regions 콜백에 종속시키지 않는다.
  human_bbox_node가 트랙을 못 받거나(사람이 없어서 발행 자체가 뜸함) 죽어있어도
  소리 방향 fallback 표시(_publish_direction_only)와 sound_timeout_sec 타임아웃
  체크가 정확한 주기로 동작해야 하므로, 자체 타이머(EVAL_HZ)로 주기적으로
  최신 캐시된 트랙 리스트를 대상으로 매칭/발행을 수행한다.
"""

import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from sound_interfaces.msg import SoundEvent


EVAL_HZ = 10.0  # 트랙 메시지 수신 여부와 무관하게 매칭/타임아웃을 평가하는 주기


def angle_diff(a: float, b: float) -> float:
    """두 각도(도)의 최소 차이 (-180 ~ 180)."""
    d = (a - b + 180.0) % 360.0 - 180.0
    return d


class TrackedSource:
    __slots__ = ("track_id", "center", "size")

    def __init__(self, track_id: float, center: np.ndarray, size: np.ndarray):
        self.track_id = int(track_id)
        self.center = center
        self.size = size


def parse_protected_regions(msg: Float32MultiArray) -> list[TrackedSource]:
    """Float32MultiArray → TrackedSource 리스트. 포맷: 7개씩 [cx,cy,cz,sx,sy,sz,tid]"""
    data = np.asarray(msg.data, dtype=np.float64)
    tracks = []
    for i in range(0, len(data) - 6, 7):
        c = data[i:i + 3]
        s = data[i + 3:i + 6]
        tid = data[i + 6]
        tracks.append(TrackedSource(tid, c, s))
    return tracks


class SoundSourceMarkerNode(Node):

    def __init__(self):
        super().__init__('sound_source_marker_node')

        # ── 파라미터 ──
        self.declare_parameter('tracks_topic', '/protected_regions')
        self.declare_parameter('sound_topic', '/sound_events')
        self.declare_parameter('output_topic', '/sound_source_track')
        self.declare_parameter('marker_topic', '/sound_source_marker')
        self.declare_parameter('frame_id', 'unilidar_lidar')

        # 소리 감지 신뢰도 임계값 (sound_localizer_node의 confidence)
        self.declare_parameter('min_confidence', 0.4)
        # 소리 이벤트 유효 시간 (이 시간 지나면 "소리 없음"으로 간주)
        self.declare_parameter('sound_timeout_sec', 1.5)
        # 트랙 각도와 소리 각도가 이 값(도) 이내면 매칭으로 인정
        self.declare_parameter('match_tolerance_deg', 30.0)
        # 마이크 어레이 정면 기준(0도)과 LiDAR 좌표계 x축 정면이 다르게
        # 장착된 경우 보정용 오프셋 (도). sound_localizer_node의
        # angle_offset과는 별개로, "마이크-라이다 간 상대 장착각" 보정용.
        self.declare_parameter('mic_to_lidar_offset_deg', 0.0)
        # 매칭되는 트랙이 없을 때, 방향 표시용 화살표를 그릴 가상 거리 (m)
        self.declare_parameter('fallback_marker_range_m', 3.0)
        # 트랙 소스(/protected_regions) 자체가 이 시간 이상 안 들어오면
        # 캐시를 비운다 (죽은 human_bbox_node의 stale 트랙으로 오매칭 방지).
        self.declare_parameter('tracks_stale_timeout_sec', 2.0)

        self.tracks_topic = self.get_parameter('tracks_topic').value
        self.sound_topic = self.get_parameter('sound_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.frame_id = self.get_parameter('frame_id').value

        # ── 상태 ──
        self._sound_angle = None       # 도 (-180~180), LiDAR 좌표계 보정 후
        self._sound_confidence = 0.0
        self._last_sound_time = self.get_clock().now()

        self._latest_tracks: list[TrackedSource] = []   # /protected_regions 캐시
        self._last_tracks_time = None                   # 트랙 메시지 마지막 수신 시각

        self._last_matched_id = None   # 로그 중복 출력 방지용
        self._last_mode = None         # 'match' | 'direction' | None (로그 중복 방지)

        # ── 구독 ──
        # 콜백에서는 상태 캐시만 갱신하고, 실제 매칭/발행 판단은 타이머가 전담한다.
        self.create_subscription(
            Float32MultiArray, self.tracks_topic, self._on_tracks, 10)
        self.create_subscription(
            SoundEvent, self.sound_topic, self._on_sound, 10)

        # ── 퍼블리셔 ──
        self.pub_track = self.create_publisher(
            Float32MultiArray, self.output_topic, 10)
        self.pub_marker = self.create_publisher(
            MarkerArray, self.marker_topic, 10)

        # ── 타이머: /protected_regions 수신 여부와 무관하게 일정 주기로 평가 ──
        self.create_timer(1.0 / EVAL_HZ, self._evaluate_and_publish)

        self.get_logger().info(
            f'SoundSourceMarkerNode 시작 | '
            f'tracks={self.tracks_topic} sound={self.sound_topic} | '
            f'tolerance=±{self.get_parameter("match_tolerance_deg").value}° '
            f'timeout={self.get_parameter("sound_timeout_sec").value}s | '
            f'eval_rate={EVAL_HZ}Hz'
        )

    # ────────────────────────────────────────────────────
    def _on_sound(self, msg: SoundEvent):
        min_conf = self.get_parameter('min_confidence').value
        if not msg.is_active or msg.confidence < min_conf:
            return

        offset = self.get_parameter('mic_to_lidar_offset_deg').value
        self._sound_angle = (float(msg.angle) + offset + 180.0) % 360.0 - 180.0
        self._sound_confidence = float(msg.confidence)
        self._last_sound_time = self.get_clock().now()

    def _sound_is_fresh(self) -> bool:
        if self._sound_angle is None:
            return False
        timeout = self.get_parameter('sound_timeout_sec').value
        elapsed = (self.get_clock().now() - self._last_sound_time).nanoseconds / 1e9
        return elapsed <= timeout

    # ────────────────────────────────────────────────────
    def _on_tracks(self, msg: Float32MultiArray):
        """트랙 메시지는 캐시만 갱신한다. 매칭/발행은 _evaluate_and_publish가 전담."""
        self._latest_tracks = parse_protected_regions(msg)
        self._last_tracks_time = self.get_clock().now()

    def _tracks_are_fresh(self) -> bool:
        if self._last_tracks_time is None:
            return False
        timeout = self.get_parameter('tracks_stale_timeout_sec').value
        elapsed = (self.get_clock().now() - self._last_tracks_time).nanoseconds / 1e9
        return elapsed <= timeout

    # ────────────────────────────────────────────────────
    def _evaluate_and_publish(self):
        """human_bbox_node(/protected_regions)의 발행 주기/유무와 무관하게
        일정한 주기(EVAL_HZ)로 소리 타임아웃과 트랙 매칭을 판단해 발행한다."""
        if not self._sound_is_fresh():
            # 소리 자체가 없거나 타임아웃 → 완전히 없음
            self._publish_no_match()
            return

        target = self._sound_angle
        tolerance = self.get_parameter('match_tolerance_deg').value

        # human_bbox_node가 죽어서 트랙 캐시가 stale해졌다면 오매칭 방지를 위해
        # 빈 트랙 리스트로 취급 → 결국 fallback(방향만 표시)으로 빠진다.
        tracks = self._latest_tracks if self._tracks_are_fresh() else []

        best_track = None
        best_diff = tolerance
        for tr in tracks:
            track_angle = np.degrees(
                np.arctan2(tr.center[1], tr.center[0])) % 360.0
            track_angle = (track_angle + 180.0) % 360.0 - 180.0
            diff = abs(angle_diff(track_angle, target))
            if diff < best_diff:
                best_diff = diff
                best_track = tr

        if best_track is not None:
            self._publish_match(best_track, best_diff)
        else:
            # 소리는 유효한데 매칭되는 트랙이 없음
            # (트랙 자체가 없거나 stale하거나, 모든 트랙이 tolerance 밖)
            # → 트랙 없이 방향만 표시 (fallback)
            self._publish_direction_only(target)

    # ────────────────────────────────────────────────────
    def _publish_direction_only(self, angle_deg: float):
        """매칭되는 트랙이 없을 때, 방향 정보만이라도 흘려보낸다."""
        rng = self.get_parameter('fallback_marker_range_m').value
        rad = np.radians(angle_deg)
        x = float(rng * np.cos(rad))
        y = float(rng * np.sin(rad))

        out = Float32MultiArray()
        # track_id=-1 : "트랙 미확인, 방향만 유효" 신호
        out.data = [-1.0, x, y, 0.0, float(self._sound_confidence)]
        self.pub_track.publish(out)

        if self._last_mode != 'direction':
            self.get_logger().info(
                f'소리 감지됐지만 매칭 트랙 없음 → 방향만 표시 '
                f'(angle={angle_deg:.1f}°, conf={self._sound_confidence:.2f})')
        self._last_mode = 'direction'
        self._last_matched_id = None

        self._publish_markers(tr=None, direction_deg=angle_deg, range_m=rng)

    # ────────────────────────────────────────────────────
    def _publish_match(self, tr: TrackedSource, angle_err: float):
        out = Float32MultiArray()
        out.data = [
            float(tr.track_id),
            float(tr.center[0]), float(tr.center[1]), float(tr.center[2]),
            float(self._sound_confidence),
        ]
        self.pub_track.publish(out)

        if self._last_mode != 'match' or self._last_matched_id != tr.track_id:
            self.get_logger().info(
                f'소리 발생원 매칭: track {tr.track_id} '
                f'(각도오차={angle_err:.1f}°, conf={self._sound_confidence:.2f})')
        self._last_mode = 'match'
        self._last_matched_id = tr.track_id

        self._publish_markers(tr=tr)

    def _publish_no_match(self):
        out = Float32MultiArray()
        out.data = []
        self.pub_track.publish(out)

        self._last_matched_id = None
        self._last_mode = None
        self._publish_markers(tr=None)

    # ────────────────────────────────────────────────────
    def _publish_markers(self, tr: TrackedSource | None = None,
                          direction_deg: float | None = None,
                          range_m: float = 3.0):
        ma = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.action = Marker.DELETEALL
        ma.markers.append(clear)

        header_stamp = self.get_clock().now().to_msg()

        if tr is not None:
            # ── 트랙 매칭 성공: 링 + 화살표 + 텍스트 (빨강) ──
            ring = Marker()
            ring.header.frame_id = self.frame_id
            ring.header.stamp = header_stamp
            ring.ns = 'sound_source'
            ring.id = 0
            ring.type = Marker.CYLINDER
            ring.action = Marker.ADD
            ring.pose.position.x = float(tr.center[0])
            ring.pose.position.y = float(tr.center[1])
            ring.pose.position.z = float(tr.center[2])
            ring.pose.orientation.w = 1.0
            ring.scale.x = float(max(tr.size[0], 0.3) + 0.3)
            ring.scale.y = float(max(tr.size[1], 0.3) + 0.3)
            ring.scale.z = 0.05
            ring.color = ColorRGBA(r=1.0, g=0.1, b=0.1, a=0.6)
            ring.lifetime.sec = 1
            ma.markers.append(ring)

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = header_stamp
            arrow.ns = 'sound_source'
            arrow.id = 1
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=float(tr.center[0]), y=float(tr.center[1]), z=float(tr.center[2])),
            ]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color = ColorRGBA(r=1.0, g=0.3, b=0.0, a=0.8)
            arrow.lifetime.sec = 1
            ma.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = header_stamp
            text.ns = 'sound_source'
            text.id = 2
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = float(tr.center[0])
            text.pose.position.y = float(tr.center[1])
            text.pose.position.z = float(tr.center[2] + tr.size[2] / 2 + 0.3)
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color = ColorRGBA(r=1.0, g=1.0, b=1.0, a=1.0)
            text.text = f'SOUND SOURCE\nT{tr.track_id}'
            text.lifetime.sec = 1
            ma.markers.append(text)

        elif direction_deg is not None:
            # ── 트랙 미확인, 방향만 유효: 화살표 + 텍스트 (노랑) ──
            rad = np.radians(direction_deg)
            x = float(range_m * np.cos(rad))
            y = float(range_m * np.sin(rad))

            arrow = Marker()
            arrow.header.frame_id = self.frame_id
            arrow.header.stamp = header_stamp
            arrow.ns = 'sound_direction'
            arrow.id = 0
            arrow.type = Marker.ARROW
            arrow.action = Marker.ADD
            arrow.points = [
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=x, y=y, z=0.0),
            ]
            arrow.scale.x = 0.05
            arrow.scale.y = 0.12
            arrow.scale.z = 0.12
            arrow.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=0.8)
            arrow.lifetime.sec = 1
            ma.markers.append(arrow)

            text = Marker()
            text.header.frame_id = self.frame_id
            text.header.stamp = header_stamp
            text.ns = 'sound_direction'
            text.id = 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.4
            text.pose.orientation.w = 1.0
            text.scale.z = 0.2
            text.color = ColorRGBA(r=1.0, g=0.9, b=0.0, a=1.0)
            text.text = 'SOUND (트랙 미확인)'
            text.lifetime.sec = 1
            ma.markers.append(text)

        self.pub_marker.publish(ma)


def main(args=None):
    rclpy.init(args=args)
    node = SoundSourceMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
