"""
eval_logger_node.py
─────────────────────────
목적:
  R값(혹은 다른 파라미터) 변경 전/후 성능을 비교하기 위해,
  human_bbox_node의 출력을 시간순으로 CSV에 그대로 쌓아두는 로깅 전용 노드.
  판단은 하지 않고 기록만 한다 — 분석은 pandas/엑셀에서.

구독:
  /human_bbox          (MarkerArray)
      최종 표시되는 트랙(=confirmed + 속도유효 + is_moving) 정보.
      ns='human_text' 텍스트 마커에서 track_id, 크기, 속도를 파싱.
  /protected_regions    (Float32MultiArray)
      is_moving 여부와 무관한 confirmed 트랙 전체 (정지/낙상 포함).

출력:
  CSV 한 줄 = 한 프레임에서 관측된 트랙 하나.
  컬럼: timestamp, source, track_id, x, y, z, sx, sy, sz, speed, is_moving

  source='human_bbox'        → is_moving=1 (정의상 움직인다고 판정된 트랙만 여기 나옴)
  source='protected_regions' → is_moving=''(unknown, 정지 포함 전체 confirmed 트랙)

사용 예 (R 변경 전/후 비교):
  ros2 run lidar_focus eval_logger_node --ros-args -p output_csv:=/tmp/before.csv
  (R 파라미터 바꾼 뒤)
  ros2 run lidar_focus eval_logger_node --ros-args -p output_csv:=/tmp/after.csv

  이후 pandas로 두 CSV 비교:
    df = pd.read_csv('/tmp/before.csv')
    moving = df[df.source == 'human_bbox']
    moving.track_id.nunique()                       # 오탐 포함 등장한 트랙 수
    moving.groupby('track_id').timestamp.agg(['min','max'])  # 트랙별 지속시간
    moving.speed.describe()                          # 속도 분포/오차 확인
"""

import re
import csv
import os
import datetime

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import MarkerArray, Marker


# human_bbox_node._text() 가 만드는 텍스트 포맷과 반드시 일치해야 함:
#   f"T{tid}\nH:{h:.2f}m  W:{w:.2f}m  D:{d:.2f}m\nspd:{spd:.2f}m/s"
TEXT_RE = re.compile(
    r'T(?P<tid>\d+)\s*'
    r'H:(?P<h>[\d.]+)m\s+W:(?P<w>[\d.]+)m\s+D:(?P<d>[\d.]+)m\s*'
    r'spd:(?P<spd>[\d.]+)m/s'
)


def parse_protected_regions(data: list[float]):
    """7개씩 [cx,cy,cz,sx,sy,sz,track_id] → dict 리스트"""
    out = []
    for i in range(0, len(data) - 6, 7):
        out.append({
            'track_id': int(data[i + 6]),
            'x': data[i], 'y': data[i + 1], 'z': data[i + 2],
            'sx': data[i + 3], 'sy': data[i + 4], 'sz': data[i + 5],
        })
    return out


class EvalLoggerNode(Node):

    HEADER = ['timestamp', 'source', 'track_id',
              'x', 'y', 'z', 'sx', 'sy', 'sz', 'speed', 'is_moving']

    def __init__(self):
        super().__init__('eval_logger_node')

        self.declare_parameter('output_csv', '')  # 비워두면 자동 생성
        self.declare_parameter('bbox_topic', '/human_bbox')
        self.declare_parameter('protected_topic', '/protected_regions')

        out_path = self.get_parameter('output_csv').value
        if not out_path:
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            out_path = f'/tmp/eval_log_{ts}.csv'

        self.out_path = out_path
        is_new = not os.path.exists(out_path)
        self._file = open(out_path, 'a', newline='')
        self._writer = csv.writer(self._file)
        if is_new:
            self._writer.writerow(self.HEADER)
            self._file.flush()

        # 요약용 인메모리 통계 (human_bbox 소스만)
        self._track_first: dict[int, float] = {}
        self._track_last: dict[int, float] = {}
        self._track_count: dict[int, int] = {}

        bbox_topic = self.get_parameter('bbox_topic').value
        protected_topic = self.get_parameter('protected_topic').value

        self.create_subscription(
            MarkerArray, bbox_topic, self._on_bbox, 10)
        self.create_subscription(
            Float32MultiArray, protected_topic, self._on_protected, 10)

        self.get_logger().warn(
            f'=== 평가 로깅 시작 ===\n'
            f'  출력 파일: {out_path}\n'
            f'  구독: {bbox_topic} (moving), {protected_topic} (confirmed 전체)\n'
            f'  Ctrl+C로 종료하면 요약을 출력합니다.'
        )

    # ────────────────────────────────────────────────────
    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _on_bbox(self, msg: MarkerArray):
        t = self._now()
        for m in msg.markers:
            if m.type != Marker.TEXT_VIEW_FACING or m.ns != 'human_text':
                continue
            match = TEXT_RE.search(m.text)
            if not match:
                self.get_logger().warn(
                    f'human_text 마커 텍스트 파싱 실패: {m.text!r} '
                    f'(human_bbox_node._text() 포맷이 바뀌었을 수 있음)')
                continue

            tid = int(match.group('tid'))
            speed = float(match.group('spd'))
            h = float(match.group('h'))
            w = float(match.group('w'))
            d = float(match.group('d'))
            x = m.pose.position.x
            y = m.pose.position.y
            z = m.pose.position.z  # bbox 상단 + 0.2 오프셋 (텍스트 표시 위치)

            self._writer.writerow(
                [t, 'human_bbox', tid, x, y, z, w, d, h, speed, 1])

            self._track_first.setdefault(tid, t)
            self._track_last[tid] = t
            self._track_count[tid] = self._track_count.get(tid, 0) + 1

        self._file.flush()

    def _on_protected(self, msg: Float32MultiArray):
        t = self._now()
        regions = parse_protected_regions(list(msg.data))
        for r in regions:
            self._writer.writerow([
                t, 'protected_regions', r['track_id'],
                r['x'], r['y'], r['z'],
                r['sx'], r['sy'], r['sz'],
                '', '',
            ])
        if regions:
            self._file.flush()

    # ────────────────────────────────────────────────────
    def _print_summary(self):
        n_tracks = len(self._track_first)
        self.get_logger().warn('=== 로깅 요약 (human_bbox / is_moving=1 기준) ===')
        self.get_logger().warn(f'  등장한 track_id 수(=오탐 포함 총 트랙 수): {n_tracks}')
        total_dur = 0.0
        for tid in sorted(self._track_first):
            dur = self._track_last[tid] - self._track_first[tid]
            total_dur += dur
            self.get_logger().info(
                f'    T{tid}: 지속 {dur:.1f}s, 관측 {self._track_count[tid]}회')
        self.get_logger().warn(f'  트랙 총 지속시간 합: {total_dur:.1f}s')
        self.get_logger().warn(f'  CSV 저장 위치: {self.out_path}')

    def destroy_node(self):
        try:
            self._print_summary()
        finally:
            self._file.close()
            super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EvalLoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
