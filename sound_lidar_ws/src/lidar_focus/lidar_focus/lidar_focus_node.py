import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import numpy as np

from sensor_msgs.msg import PointCloud2, PointField
from sound_interfaces.msg import SoundEvent


# ── PointCloud2 파싱 헬퍼 ─────────────────────────────────────
FIELD_DTYPE = {
    PointField.FLOAT32: ('f4', 4),
    PointField.FLOAT64: ('f8', 8),
    PointField.INT8:    ('i1', 1),
    PointField.UINT8:   ('u1', 1),
    PointField.INT16:   ('i2', 2),
    PointField.UINT16:  ('u2', 2),
    PointField.INT32:   ('i4', 4),
    PointField.UINT32:  ('u4', 4),
}

def pc2_to_xyz(msg):
    dtype_list = []
    offset = 0
    for f in sorted(msg.fields, key=lambda x: x.offset):
        if f.offset > offset:
            dtype_list.append((f'_pad{offset}', f'V{f.offset - offset}'))
        dt_str, _ = FIELD_DTYPE[f.datatype]
        dtype_list.append((f.name, dt_str))
        offset = f.offset + FIELD_DTYPE[f.datatype][1]
    if msg.point_step > offset:
        dtype_list.append(('_pad_end', f'V{msg.point_step - offset}'))
    raw = np.frombuffer(bytes(msg.data), dtype=np.dtype(dtype_list))
    return np.stack([
        raw['x'].astype(np.float32),
        raw['y'].astype(np.float32),
        raw['z'].astype(np.float32),
    ], axis=1)

def xyz_to_pc2(header, xyz):
    msg = PointCloud2()
    msg.header     = header
    msg.height     = 1
    msg.width      = len(xyz)
    msg.fields     = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step   = 12
    msg.row_step     = 12 * len(xyz)
    msg.is_dense     = True
    msg.data         = xyz.astype(np.float32).tobytes()
    return msg

def angle_diff(a, b):
    """두 각도의 최소 차이 (-180~180)"""
    d = (a - b) % 360.0
    return d if d <= 180.0 else d - 360.0


# ── 메인 노드 ─────────────────────────────────────────────────
class LidarFocusNode(Node):
    def __init__(self):
        super().__init__('lidar_focus_node')

        # 파라미터
        self.declare_parameter('sector_width',    45.0)  # 집중할 섹터 반폭 (도)
        self.declare_parameter('timeout_sec',      3.0)  # 이 시간 지나면 전체 스캔
        self.declare_parameter('lidar_topic', '/unilidar/cloud')
        self.declare_parameter('min_confidence', 0.4)

        self.sector_w = self.get_parameter('sector_width').value
        self.timeout = self.get_parameter('timeout_sec').value
        self.min_conf = self.get_parameter('min_confidence').value
        lidar_topic = self.get_parameter('lidar_topic').value

        # 상태
        self._target_angle = None   # 노드에서 받은 각도
        self._last_recv    = self.get_clock().now()

        # QoS
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Subscribers
        self.create_subscription(
            SoundEvent,
            '/sound_events',       # 친구 노드 토픽
            self._mic_cb,
            10
        )
        self.create_subscription(
            PointCloud2,
            lidar_topic,            # Unitree L2 원본
            self._lidar_cb,
            sensor_qos
        )

        # Publishers
        self.pub_focused = self.create_publisher(
            PointCloud2, '/lidar/focused', sensor_qos)
        self.pub_full    = self.create_publisher(
            PointCloud2, '/lidar/full',    sensor_qos)

        self.create_timer(2.0, self._status_log)

        self.get_logger().info(
            f'LiDAR Focus 노드 시작\n'
            f'  수신 토픽: /mic/detection  (친구 노드)\n'
            f'  LiDAR 토픽: {lidar_topic}\n'
            f'  섹터 반폭: ±{self.sector_w}도\n'
            f'  타임아웃: {self.timeout}초'
        )

    def _mic_cb(self, msg: SoundEvent):
        """
        sound_localizer_node.py로부터 SoundEvent 메시지 수신
        msg.angle: 소리 방향 (도)
        msg.confidence: 0.0 ~ 1.0 신뢰도
        msg.is_active: 활성화 여부
        """
        # 1. 활성화 여부 및 신뢰도 체크 (노이즈 방지)
        if not msg.is_active or msg.confidence < self.min_conf:
            return

        # 2. 상태 갱신
        self._target_angle = msg.angle
        self._last_recv = self.get_clock().now()

        self.get_logger().info(
            f'소리 포커스 갱신: {self._target_angle:.1f}° '
            f'(Conf: {msg.confidence:.2f}, Amp: {msg.amplitude:.1f}dB)'
        )

    def _lidar_cb(self, msg: PointCloud2):
        # 전체 포인트 항상 퍼블리시 (디버그용)
        self.pub_full.publish(msg)

        # 타임아웃 체크
        elapsed = (self.get_clock().now() - self._last_recv).nanoseconds / 1e9
        if self._target_angle is None or elapsed > self.timeout:
            # 소리 없음 → 전체 포인트 그대로
            self.pub_focused.publish(msg)
            return

        # ── 섹터 필터링 ──────────────────────────────────────────
        try:
            xyz = pc2_to_xyz(msg)
        except Exception as e:
            self.get_logger().warn(f'파싱 오류: {e}')
            self.pub_focused.publish(msg)
            return

        if len(xyz) == 0:
            return

        # 각 포인트의 수평 각도 계산
        point_angles = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0])) % 360.0
        target       = self._target_angle % 360.0

        # 목표 각도 ± sector_width 안에 있는 포인트만 선택
        diffs = np.abs([angle_diff(a, target) for a in point_angles])
        mask  = diffs <= self.sector_w

        focused = xyz[mask]
        self.pub_focused.publish(xyz_to_pc2(msg.header, focused))

        self.get_logger().debug(
            f'각도={target:.1f}도 | '
            f'전체={len(xyz)} → 집중={len(focused)}포인트 '
            f'({100*len(focused)//max(len(xyz),1)}%)'
        )

    def _status_log(self):
        elapsed = (self.get_clock().now() - self._last_recv).nanoseconds / 1e9
        if self._target_angle is None:
            self.get_logger().info('대기 중... /mic/detection 수신 없음')
        elif elapsed > self.timeout:
            self.get_logger().info(f'타임아웃 ({elapsed:.1f}초) → 전체 스캔 중')
        else:
            self.get_logger().info(
                f'집중 중: {self._target_angle:.1f}도 '
                f'(마지막 감지 {elapsed:.1f}초 전)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = LidarFocusNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
