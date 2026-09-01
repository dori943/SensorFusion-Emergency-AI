"""
emergency_fusion/viz_merge_node.py
────────────────────────────────────────────────────────────────────────
목적:
  three.js 렌더러에 넘길 "한 장면"을 하나의 토픽으로 합친다.

  낙상 알림을 받은 사람이 화면을 보고 "진짜 쓰러진 게 맞나"를 판단하려면
  세 가지가 동시에 보여야 한다.

    1. 공간 맥락  : 벽·가구가 있어야 "바닥에 누웠다"를 알 수 있다.
                   전경만 띄우면 허공에 뜬 점 덩어리라 판단이 불가능하다.
    2. 움직이는 것: 시스템이 사람으로 인식했든 아니든 전경 전부.
                   /human_cloud만 띄우면 검출 실패 시 화면이 백지가 되고,
                   특히 "bbox 소실"로 위급 판정된 경우 — 가장 급한
                   알림에서 — 정의상 아무것도 안 보인다.
    3. 시스템 판단: 어느 점을 사람이라 봤는지, 어디를 낙상으로 봤는지.

  이 셋을 색으로 구분해 한 PointCloud2에 담으면 렌더러 쪽은 위치와
  색만 읽어 그리면 된다. 레이어 동기화나 판정 로직을 클라이언트가
  알 필요가 없다.

출력:
  /viz/scene_cloud (sensor_msgs/PointCloud2)  x,y,z,rgb
      배경(어두운 회색) + 전경(흰색) + 사람(주황/빨강) 합본

  /viz/scene_meta  (std_msgs/String, JSON)
      bbox·낙상 상태·알림 레벨. 점으로 그리기 어려운 것들.
      three.js에서 BoxGeometry로 그리면 된다.

      {"stamp": 1234.5, "frame_id": "unilidar_lidar",
       "counts": {"bg": 8000, "fg": 420, "human": 180},
       "tracks": [{"id": 3, "center": [x,y,z], "size": [sx,sy,sz],
                   "verticality": 0.31, "posture": "lying"}],
       "fall": {"level": "critical", "case_id": 2, "score": 0.75,
                "position": [x,y,z], "reason": "..."}}

대역폭 메모:
  배경은 정적이라 매 프레임 보낼 이유가 없다. bg_publish_period_sec
  주기로만 갱신하고, 그 사이 프레임에는 마지막으로 보낸 배경을 그대로
  재사용한다(렌더러가 누적하지 않도록 매번 합본에 포함시키되,
  다운샘플을 세게 걸어 점 수를 줄인다).
  10Hz × 배경 3만점을 그대로 웹소켓으로 밀면 링크가 못 버틴다.
"""

from __future__ import annotations

import json
import struct

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, String

import sensor_msgs_py.point_cloud2 as pc2


def read_points(msg):
    pts = pc2.read_points_numpy(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if pts is None or len(pts) == 0:
        return np.zeros((0, 3), dtype=np.float32)
    return pts.astype(np.float32)


def voxel_downsample(pts: np.ndarray, voxel: float) -> np.ndarray:
    if voxel <= 0 or len(pts) == 0:
        return pts
    keys = np.floor(pts / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    return pts[idx]


def pack_rgb(r: int, g: int, b: int) -> float:
    """ROS 관례: uint32 0x00RRGGBB를 float32로 재해석해서 담는다."""
    rgb = (int(r) << 16) | (int(g) << 8) | int(b)
    return struct.unpack('f', struct.pack('I', rgb))[0]


# 레이어별 색
COLOR_BG = pack_rgb(70, 70, 78)        # 어두운 회색 — 맥락, 눈에 안 띄게
COLOR_FG = pack_rgb(235, 235, 235)     # 흰색 — 움직이는 것
COLOR_HUMAN = pack_rgb(255, 140, 0)    # 주황 — 사람으로 판정됨
COLOR_FALL = pack_rgb(255, 40, 40)     # 빨강 — 낙상 확정/위급

LEVEL_NAME = {0: 'observing', 1: 'pending', 2: 'confirmed',
              3: 'critical', 4: 'resolved'}


class VizMergeNode(Node):

    def __init__(self):
        super().__init__('viz_merge_node')

        self.declare_parameter('bg_topic', '/ground_removed_cloud')
        self.declare_parameter('fg_topic', '/filtered_cloud')
        self.declare_parameter('human_topic', '/human_cloud')
        self.declare_parameter('tracks_topic', '/human_tracks')
        self.declare_parameter('fall_state_topic', '/emergency/fall_state')
        self.declare_parameter('fall_alert_topic', '/emergency/fall_alert')

        self.declare_parameter('scene_topic', '/viz/scene_cloud')
        self.declare_parameter('meta_topic', '/viz/scene_meta')
        self.declare_parameter('frame_id', 'unilidar_lidar')
        self.declare_parameter('publish_rate', 10.0)

        # 대역폭 조절. 배경은 세게, 사람은 거의 안 줄인다.
        #
        # 점 하나 = 16바이트(xyzrgb). 배경 3만점을 10Hz로 그대로 밀면
        # 약 42Mbps라 웹소켓으로는 불가능하다. 아래 기본값은 약 12Mbps
        # 수준으로, 로컬 네트워크에서 무난하다. 원격이면 publish_rate를
        # 5Hz로 낮추면 절반이 된다(약 6Mbps).
        self.declare_parameter('bg_voxel', 0.20)
        self.declare_parameter('fg_voxel', 0.05)
        self.declare_parameter('human_voxel', 0.0)      # 0 = 다운샘플 안 함
        self.declare_parameter('bg_max_points', 8000)
        self.declare_parameter('bg_refresh_sec', 5.0)   # 배경 재수집 주기

        # 진단: 레이어별 점 개수를 주기적으로 찍는다.
        # /filtered_cloud에는 사람이 있는데 /human_cloud가 0이면 검출 실패다.
        self.declare_parameter('log_counts', True)
        self.declare_parameter('log_every_sec', 2.0)

        self.frame_id = self.get_parameter('frame_id').value

        self._bg = np.zeros((0, 3), dtype=np.float32)
        self._bg_t = None
        self._fg = np.zeros((0, 3), dtype=np.float32)
        self._human = np.zeros((0, 3), dtype=np.float32)
        self._tracks = []
        self._fall = None
        self._fall_alert = None
        self._t_log = 0.0

        self.create_subscription(
            PointCloud2, self.get_parameter('bg_topic').value, self._on_bg, 1)
        self.create_subscription(
            PointCloud2, self.get_parameter('fg_topic').value, self._on_fg, 1)
        self.create_subscription(
            PointCloud2, self.get_parameter('human_topic').value,
            self._on_human, 1)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('tracks_topic').value,
            self._on_tracks, 10)
        self.create_subscription(
            Float32MultiArray, self.get_parameter('fall_state_topic').value,
            self._on_fall_state, 10)
        self.create_subscription(
            String, self.get_parameter('fall_alert_topic').value,
            self._on_fall_alert, 10)

        # 렌더러가 늦게 붙어도 마지막 장면을 받도록 latch.
        qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_scene = self.create_publisher(
            PointCloud2, self.get_parameter('scene_topic').value, qos)
        self.pub_meta = self.create_publisher(
            String, self.get_parameter('meta_topic').value, qos)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f'VizMergeNode 시작 → {self.get_parameter("scene_topic").value} '
            f'(배경/전경/사람 합본) + '
            f'{self.get_parameter("meta_topic").value} (bbox·상태)')

    def _now(self):
        return self.get_clock().now().nanoseconds / 1e9

    # ── 입력 ────────────────────────────────────────────────
    def _on_bg(self, msg):
        now = self._now()
        # 배경은 정적이라 매 프레임 다시 담을 이유가 없다.
        if (self._bg_t is not None
                and (now - self._bg_t) < self.get_parameter('bg_refresh_sec').value):
            return
        pts = voxel_downsample(read_points(msg),
                               self.get_parameter('bg_voxel').value)
        cap = int(self.get_parameter('bg_max_points').value)
        if len(pts) > cap:
            # 균등 추출 — 앞쪽만 자르면 공간 일부가 통째로 빠진다.
            idx = np.linspace(0, len(pts) - 1, cap).astype(int)
            pts = pts[idx]
        self._bg = pts
        self._bg_t = now

    def _on_fg(self, msg):
        self._fg = voxel_downsample(read_points(msg),
                                    self.get_parameter('fg_voxel').value)

    def _on_human(self, msg):
        self._human = voxel_downsample(read_points(msg),
                                       self.get_parameter('human_voxel').value)

    def _on_tracks(self, msg):
        data = np.asarray(msg.data, dtype=np.float64)
        out = []
        for i in range(0, len(data) - 8, 9):
            v = float(data[i + 7])
            sz = data[i + 4:i + 7]
            out.append({
                'id': int(data[i]),
                'center': [round(float(x), 3) for x in data[i + 1:i + 4]],
                'size': [round(float(x), 3) for x in sz],
                'verticality': None if v < 0 else round(v, 3),
                'posture': 'lying' if float(sz[2]) < 0.75 else 'standing',
            })
        self._tracks = out

    def _on_fall_state(self, msg):
        data = np.asarray(msg.data, dtype=np.float64)
        best = None
        for i in range(0, len(data) - 5, 6):
            level = int(data[i + 5])
            # 가장 위급한 사건 하나만 화면에 강조한다.
            if best is None or level > best['level_code']:
                best = {
                    'case_id': int(data[i]),
                    'position': [round(float(x), 3) for x in data[i + 1:i + 4]],
                    'score': round(float(data[i + 4]), 3),
                    'level_code': level,
                    'level': LEVEL_NAME.get(level, str(level)),
                }
        self._fall = best

    def _on_fall_alert(self, msg):
        try:
            p = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return
        self._fall_alert = {
            'case_id': p.get('case_id'),
            'level': p.get('level'),
            'severity': p.get('severity'),
            'reason': p.get('reason'),
            'confidence': p.get('confidence'),
            'elapsed_sec': p.get('elapsed_since_fall_sec'),
            'post_fall': p.get('post_fall'),
        }

    # ── 합본 발행 ───────────────────────────────────────────
    def _tick(self):
        now = self._now()

        # 낙상 확정/위급이면 사람 색을 빨강으로 바꾼다.
        human_color = COLOR_HUMAN
        if self._fall and self._fall['level_code'] >= 2:
            human_color = COLOR_FALL

        layers = [
            (self._bg, COLOR_BG),
            (self._fg, COLOR_FG),
            (self._human, human_color),
        ]

        rows = []
        for pts, color in layers:
            if len(pts) == 0:
                continue
            block = np.empty((len(pts), 4), dtype=np.float32)
            block[:, 0:3] = pts
            block[:, 3] = color
            rows.append(block)

        cloud = (np.vstack(rows) if rows
                 else np.zeros((0, 4), dtype=np.float32))

        header = self.get_clock().now().to_msg()
        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg = pc2.create_cloud(
            self._make_header(header), fields, cloud.tolist())
        self.pub_scene.publish(msg)

        meta = {
            'stamp': round(now, 3),
            'frame_id': self.frame_id,
            'counts': {
                'bg': int(len(self._bg)),
                'fg': int(len(self._fg)),
                'human': int(len(self._human)),
                'total': int(len(cloud)),
            },
            'colors': {
                'bg': '#46464e', 'fg': '#ebebeb',
                'human': '#ff8c00', 'fall': '#ff2828',
            },
            'tracks': self._tracks,
            'fall': self._fall,
            'alert': self._fall_alert,
        }
        out = String()
        out.data = json.dumps(meta, ensure_ascii=False, separators=(',', ':'))
        self.pub_meta.publish(out)

        self._maybe_log(now)

    def _make_header(self, stamp):
        from std_msgs.msg import Header
        h = Header()
        h.stamp = stamp
        h.frame_id = self.frame_id
        return h

    def _maybe_log(self, now):
        """레이어별 점 개수 진단.

        전경에는 점이 많은데 사람이 0이면 세그먼트가 실패한 것이고,
        둘 다 0이면 배경차감이 사람을 통째로 먹은 것이다. 화면만 봐서는
        이 둘이 구분되지 않아 숫자로 남긴다.
        """
        if not self.get_parameter('log_counts').value:
            return
        if (now - self._t_log) < self.get_parameter('log_every_sec').value:
            return
        self._t_log = now

        n_fg, n_hu = len(self._fg), len(self._human)
        mark = ''
        if n_fg > 50 and n_hu == 0:
            mark = '  ← 전경은 있는데 사람 검출 0 (세그먼트 실패 의심)'
        elif n_fg == 0:
            mark = '  ← 전경 자체가 없음 (배경차감이 다 먹었는지 확인)'

        lvl = self._fall['level'] if self._fall else '-'
        self.get_logger().info(
            f'[레이어] 배경 {len(self._bg):>6} | 전경 {n_fg:>5} | '
            f'사람 {n_hu:>5} | 트랙 {len(self._tracks)} | 낙상 {lvl}{mark}')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VizMergeNode()
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
