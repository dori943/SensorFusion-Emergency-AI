#!/usr/bin/env python3
"""레이어 겹침 검증용 모니터.

4개 레이어를 겹쳤을 때 "사람이 계속 보이는가"를 눈이 아니라 숫자로
확인한다. RViz 화면만 보면 점이 몇 개 남아 반짝이는 건지, 완전히
끊긴 건지 구분이 안 된다.

한 줄에 레이어별 점 개수를 1초마다 갱신해서 출력하고, 사람 레이어가
끊긴 구간을 누적 집계한다.

    python3 layer_monitor.py

읽는 법:
    fg는 있는데 human이 0  → 세그먼트 실패 (bbox 기준을 못 넘음)
    fg도 0                 → 배경차감이 사람을 통째로 먹음
    human은 있는데 track 0 → 억제(suppression)에 걸림
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32MultiArray


class LayerMonitor(Node):

    def __init__(self):
        super().__init__('layer_monitor')

        self.n = {'bg': 0, 'fg': 0, 'human': 0, 'scene': 0}
        self.tracks = 0
        self.fall_level = 0

        # 사람 레이어 끊김 통계
        self.frames = 0
        self.human_zero_frames = 0
        self.gap_start = None
        self.longest_gap = 0.0
        self.t0 = time.time()

        for key, topic in [('bg', '/ground_removed_cloud'),
                           ('fg', '/filtered_cloud'),
                           ('human', '/human_cloud'),
                           ('scene', '/viz/scene_cloud')]:
            self.create_subscription(
                PointCloud2, topic,
                lambda m, k=key: self._on_cloud(k, m), 1)

        self.create_subscription(
            Float32MultiArray, '/human_tracks', self._on_tracks, 10)
        self.create_subscription(
            Float32MultiArray, '/emergency/fall_state', self._on_fall, 10)

        self.create_timer(1.0, self._report)
        print('레이어 모니터 시작 (Ctrl+C 종료)\n')

    def _on_cloud(self, key, msg):
        self.n[key] = msg.width * msg.height

    def _on_tracks(self, msg):
        self.tracks = len(msg.data) // 9

    def _on_fall(self, msg):
        levels = [int(msg.data[i + 5])
                  for i in range(0, len(msg.data) - 5, 6)]
        self.fall_level = max(levels) if levels else 0

    def _report(self):
        self.frames += 1
        now = time.time()

        if self.n['human'] == 0:
            self.human_zero_frames += 1
            if self.gap_start is None:
                self.gap_start = now
        else:
            if self.gap_start is not None:
                self.longest_gap = max(self.longest_gap, now - self.gap_start)
                self.gap_start = None

        # 진행 중인 끊김도 반영
        cur_gap = (now - self.gap_start) if self.gap_start else 0.0
        longest = max(self.longest_gap, cur_gap)
        uptime = 100.0 * (1 - self.human_zero_frames / max(1, self.frames))

        note = ''
        if self.n['fg'] > 50 and self.n['human'] == 0:
            note = '  세그먼트 실패?'
        elif self.n['fg'] == 0 and self.n['bg'] > 0:
            note = '  전경 없음'
        elif self.n['human'] > 0 and self.tracks == 0:
            note = '  억제 중?'

        lvl = {0: '-', 1: 'PEND', 2: 'CONF', 3: 'CRIT', 4: 'RESV'}.get(
            self.fall_level, '?')

        print(f'\rbg{self.n["bg"]:>6} fg{self.n["fg"]:>5} '
              f'human{self.n["human"]:>5} scene{self.n["scene"]:>6} | '
              f'trk{self.tracks} {lvl:>4} | '
              f'사람표시율 {uptime:5.1f}%  최장끊김 {longest:4.1f}s'
              f'{note:<18}', end='', flush=True)


def main():
    rclpy.init()
    node = LayerMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        el = time.time() - node.t0
        print(f'\n\n── 요약 ({el:.0f}초) ──')
        print(f'사람 레이어 표시율 : '
              f'{100.0 * (1 - node.human_zero_frames / max(1, node.frames)):.1f}%')
        print(f'최장 끊김          : {node.longest_gap:.1f}초')
        if node.longest_gap > 3.0:
            print('\n끊김이 3초를 넘습니다. 알림 화면에서 사람이 사라지는 구간이')
            print('생기므로, /filtered_cloud 레이어를 반드시 함께 띄워야 합니다.')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
