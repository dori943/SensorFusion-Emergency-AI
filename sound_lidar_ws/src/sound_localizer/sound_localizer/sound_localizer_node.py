# sound_localizer/sound_localizer_node.py
#
# DoA  : XVF3000 칩 내장 알고리즘을 USB로 직접 폴링
# 음량 : sounddevice 6채널 스트림 ch0에서 RMS → dBFS 변환
# VAD  : XVF3000 VOICEACTIVITY 레지스터 폴링

import threading
import struct

import numpy as np
import sounddevice as sd
import usb.core
import usb.util

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sound_interfaces.msg import SoundEvent


# ── XVF3000 USB 튜닝 레지스터 접근 헬퍼 ──────────────────────────
VENDOR_ID  = 0x2886
PRODUCT_ID = 0x0018

PARAMS = {
    'DOAANGLE':      (21, 'int'),
    'VOICEACTIVITY': (19, 'int'),
    'SPEECHDETECTED':(20, 'int'),
}

class XVF3000Tuning:
    TIMEOUT = 100

    def __init__(self, dev):
        self.dev = dev

    def read(self, name: str):
        param_id, dtype = PARAMS[name]
        try:
            data = self.dev.ctrl_transfer(
                usb.util.CTRL_IN  |
                usb.util.CTRL_TYPE_VENDOR |
                usb.util.CTRL_RECIPIENT_DEVICE,
                0, param_id, 0x1C, 8, self.TIMEOUT
            )
            if dtype == 'int':
                return struct.unpack('<i', bytes(data[:4]))[0]
            else:
                return struct.unpack('<f', bytes(data[:4]))[0]
        except usb.core.USBError:
            return None


# ── 메인 노드 ─────────────────────────────────────────────────────
class SoundLocalizerNode(Node):

    SAMPLE_RATE  = 16000
    CHANNELS     = 6
    CHUNK        = 1024
    CH_PROCESSED = 0

    def __init__(self):
        super().__init__('sound_localizer')

        self.declare_parameter('threshold_db',       50.0)
        self.declare_parameter('publish_rate',       10.0)
        self.declare_parameter('angle_offset',        0.0)
        self.declare_parameter('audio_device_index',   -1)

        self.threshold    = self.get_parameter('threshold_db').value
        self.angle_offset = self.get_parameter('angle_offset').value
        rate              = self.get_parameter('publish_rate').value
        dev_idx_param     = self.get_parameter('audio_device_index').value

        # USB 초기화
        self._tuning = self._init_usb()

        # sounddevice 장치 탐색
        dev_idx = (dev_idx_param if dev_idx_param >= 0
                   else self._find_respeaker_index())
        if dev_idx is None:
            self.get_logger().error('ReSpeaker 장치를 찾을 수 없습니다.')
            raise RuntimeError('ReSpeaker not found')

        # 공유 상태
        self._lock       = threading.Lock()
        self._amplitude  = 0.0
        self._vad_active = False

        # sounddevice 스트림 시작
        self._stream = sd.InputStream(
            device         = dev_idx,
            samplerate     = self.SAMPLE_RATE,
            channels       = self.CHANNELS,
            dtype          = 'int16',
            blocksize      = self.CHUNK,
            callback       = self._audio_callback,
        )
        self._stream.start()

        # 퍼블리셔 / 타이머
        self.publisher = self.create_publisher(SoundEvent, '/sound_events', 10)
        self.create_timer(1.0 / rate, self._publish_event)

        self.get_logger().info(
            f'Sound Localizer 시작  |  장치 idx={dev_idx}  |  '
            f'임계={self.threshold}dB  |  오프셋={self.angle_offset}°'
        )

    def _init_usb(self):
        dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if dev is None:
            self.get_logger().warn('XVF3000 USB 장치를 찾지 못했습니다.')
            return None
        # Keep kernel drivers attached so the ALSA audio capture interface
        # remains visible to sounddevice.
        self.get_logger().info('XVF3000 USB 연결 완료 → DoA 하드웨어 모드')
        return XVF3000Tuning(dev)

    def _find_respeaker_index(self):
        devices = sd.query_devices()
        for i, dev in enumerate(devices):
            if 'ReSpeaker' in dev['name'] or 'respeaker' in dev['name'].lower():
                self.get_logger().info(f'ReSpeaker 자동 탐지: idx={i}  "{dev["name"]}"')
                return i
        return None

    def _audio_callback(self, indata, frames, time, status):
        try:
            ch0 = indata[:, self.CH_PROCESSED].astype(np.float32)
            rms = np.sqrt(np.mean(ch0 ** 2)) + 1e-9
            db  = 20.0 * np.log10(rms / 32768.0) + 96.0

            if self._tuning:
                vad = bool(self._tuning.read('VOICEACTIVITY'))
            else:
                vad = db >= self.threshold

            with self._lock:
                self._amplitude  = float(db)
                self._vad_active = vad
        except Exception as e:
            self.get_logger().warn(f'오디오 콜백 오류: {e}')

    def _publish_event(self):
        with self._lock:
            amplitude = self._amplitude
            is_active = self._vad_active

        if amplitude < self.threshold:
            return

        angle = self._get_doa()
        if angle is None:
            return

        confidence = float(np.clip(
            (amplitude - self.threshold) / 30.0, 0.0, 1.0
        ))

        msg                 = SoundEvent()
        msg.header          = Header()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'mic_array'
        msg.angle           = angle
        msg.confidence      = confidence
        msg.amplitude       = amplitude
        msg.is_active       = is_active

        self.publisher.publish(msg)
        self.get_logger().info(
            f'[SoundEvent] angle={angle:.1f}°  '
            f'amp={amplitude:.1f}dB  conf={confidence:.2f}  '
            f'vad={is_active}'
        )

    def _get_doa(self):
        if self._tuning:
            raw = self._tuning.read('DOAANGLE')
            if raw is not None:
                angle = float(raw % 360)
                if angle > 180:
                    angle -= 360
                return float(angle + self.angle_offset)
        return None

    def destroy_node(self):
        self._stream.stop()
        self._stream.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = SoundLocalizerNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()
