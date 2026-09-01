"""응급상황 감지 ROS2 노드 (패키지명: sos_voice).

- 마이크 오디오 토픽을 구독해서 3초 단위로 버퍼링
- inference.py의 EmergencyDetector로 각 3초 window를 예측
- "3초 하나만 보고 바로 알람"하지 않고, 최근 N개 window 결과를 누적해서
  과반수 이상 emergency로 판단될 때만 알람 (오탐 완화)
- 결과를 토픽으로 발행 (알람용 Bool + 확률값 String/JSON)

토픽/파라미터는 launch 파일이나 커맨드라인에서 조절 가능하게 파라미터로 뺐습니다.
사용 중인 마이크 드라이버(audio_common, respeaker 노드 등)가 어떤 메시지 타입을
쓰는지에 맞춰 audio_callback의 파싱 부분만 조정하면 됩니다.
"""
import collections
import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32MultiArray, String
from ament_index_python.packages import get_package_share_directory

from .inference import EmergencyDetector, CLASS_NAMES, TARGET_SR, CLIP_SAMPLES


def _default_model_path() -> str:
    """패키지 설치 위치(share/sos_voice/models/...) 기준으로 모델 파일 자동 탐색.
    못 찾으면 빈 문자열 반환 (이 경우 model_path 파라미터를 반드시 지정해야 함).
    """
    try:
        share_dir = Path(get_package_share_directory('sos_voice'))
        candidate = share_dir / 'models' / 'model_distilled_quantized.pt'
        if candidate.exists():
            return str(candidate)
    except Exception:
        pass
    return ''


class EmergencyDetectorNode(Node):
    def __init__(self):
        super().__init__('emergency_detector_node')

        # ---- 파라미터 ----
        self.declare_parameter('model_path', _default_model_path())
        self.declare_parameter('audio_topic', '/audio/raw')  # 마이크 드라이버가 발행하는 토픽
        self.declare_parameter('sample_rate', TARGET_SR)
        self.declare_parameter('emergency_threshold', 0.45)  # notebook 12절 threshold sweep 결과 참고
        self.declare_parameter('window_history_size', 2)     # 최근 몇 개 window를 볼지
        self.declare_parameter('window_alarm_ratio', 0.8)    # 그중 몇 % 이상이 emergency여야 알람

        model_path = self.get_parameter('model_path').value
        if not model_path:
            raise RuntimeError(
                '모델 경로를 찾을 수 없습니다. -p model_path:=<.pt 경로> 로 직접 지정하세요.'
            )
        self.audio_topic = self.get_parameter('audio_topic').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.threshold = self.get_parameter('emergency_threshold').value
        self.history_size = self.get_parameter('window_history_size').value
        self.alarm_ratio = self.get_parameter('window_alarm_ratio').value

        # ---- 모델 로드 (여기만 inference.py에 위임, 노드는 내부 구조를 모름) ----
        self.get_logger().info(f'모델 로드 중: {model_path}')
        self.detector = EmergencyDetector(model_path)
        self.get_logger().info('모델 로드 완료')

        # ---- 오디오 버퍼 ----
        self._audio_buffer = np.zeros(0, dtype=np.float32)

        # ---- 최근 window 판단 이력 (멀티윈도우 누적 판단용) ----
        self._window_history = collections.deque(maxlen=self.history_size)

        # ---- 구독/발행 ----
        self.audio_sub = self.create_subscription(
            Float32MultiArray, self.audio_topic, self.audio_callback, 10)

        self.alarm_pub = self.create_publisher(Bool, '/emergency_detector/alarm', 10)
        self.probs_pub = self.create_publisher(String, '/emergency_detector/probs', 10)

        self.get_logger().info(
            f'준비 완료. audio_topic={self.audio_topic}, '
            f'threshold={self.threshold}, history={self.history_size}, ratio={self.alarm_ratio}'
        )

    def audio_callback(self, msg: Float32MultiArray):
        """마이크에서 오디오 청크가 들어올 때마다 호출됨.
        Float32MultiArray를 가정했지만, 실제 사용 중인 마이크 드라이버 메시지 타입에
        맞춰 이 함수 안 파싱 부분만 바꾸면 됩니다 (예: audio_common_msgs/AudioData면
        바이트를 int16으로 unpack 후 float 변환).
        """
        chunk = np.array(msg.data, dtype=np.float32)
        self._audio_buffer = np.concatenate([self._audio_buffer, chunk])

        # 3초(CLIP_SAMPLES) 이상 쌓이면 window 하나 처리
        while len(self._audio_buffer) >= CLIP_SAMPLES:
            window = self._audio_buffer[:CLIP_SAMPLES]
            self._audio_buffer = self._audio_buffer[CLIP_SAMPLES:]
            self._process_window(window)

    def _process_window(self, window: np.ndarray):
        probs = self.detector.predict(window, sample_rate=self.sample_rate)

        # 확률 발행 (모니터링/로깅용, rqt_plot 등에서 바로 볼 수 있음)
        self.probs_pub.publish(String(data=json.dumps(probs)))

        is_emergency_window = probs['emergency'] >= self.threshold
        self._window_history.append(is_emergency_window)

        n_emergency = sum(self._window_history)
        n_total = len(self._window_history)
        ratio = n_emergency / n_total if n_total > 0 else 0.0

        # history가 꽉 찼을 때만(초반 워밍업 구간 오탐 방지) 알람 판단
        should_alarm = (n_total == self.history_size) and (ratio >= self.alarm_ratio)

        if should_alarm:
            self.get_logger().warn(
                f'[알람] emergency 비율 {ratio:.2f} (threshold={self.alarm_ratio}) - '
                f'현재 window: {probs}'
            )
        self.alarm_pub.publish(Bool(data=bool(should_alarm)))


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
