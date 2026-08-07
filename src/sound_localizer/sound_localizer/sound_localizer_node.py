# sound_localizer/sound_localizer_node.py
#
# 6채널 dfu 4.0.0 펌웨어(6_channels_dfu_4.0.0_firmware.bin) 재플래시 후
# 실제로 동작 확인된 DOA_2.py 패턴을 그대로 따르는 구현:
#
#     dev = usb.core.find(idVendor=0x2886, idProduct=0x0018)
#     Mic_tuning = Tuning(dev)
#     Mic_tuning.direction        # 0 ~ 359°
#
# 변경 요약 (기존 노드 대비):
#   1. DOA_2.py와 동일하게 usb.core.find → Tuning(dev)만 사용.
#      kernel driver detach 로직 제거 (DOA_2.py는 detach 없이 동작 확인됨).
#   2. UAC1.0(1채널) 펌웨어 fallback 경로 전부 제거.
#      6채널 펌웨어 전제 — 기동 시 direction 1회 읽기로 검증하고,
#      실패하면 명확한 에러와 함께 종료.
#   3. USB 접근을 ROS 타이머 스레드 하나로 단일화.
#      (기존에는 오디오 콜백(PortAudio 스레드)에서 VOICEACTIVITY를,
#       타이머에서 DOAANGLE을 동시에 읽어 pyusb 동시 접근 레이스가 있었음)
#
# DoA/VAD : XVF3000 튜닝 레지스터를 타이머에서 폴링 (direction / is_voice)
# 음량    : sounddevice 6채널 스트림 ch0(처리된 채널)에서 RMS → dBFS 변환

import threading

import numpy as np
import sounddevice as sd
import usb.core

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header
from sound_interfaces.msg import SoundEvent

# respeaker 공식 usb_4_mic_array/tuning.py를 그대로 vendor한 모듈.
# (DOA_2.py가 사용한 것과 동일한 파일. array.tostring() → array.tobytes()
#  패치가 되어 있어야 Python 3.9+에서 동작함)
from .vendor.tuning import Tuning


VENDOR_ID  = 0x2886
PRODUCT_ID = 0x0018


class SoundLocalizerNode(Node):

    SAMPLE_RATE  = 16000
    CHANNELS     = 6     # 6채널 펌웨어: [ch0]=processed, [ch1~4]=raw mic, [ch5]=playback
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

        # ── XVF3000 튜닝 인터페이스 (DOA_2.py와 동일한 초기화) ──
        self._tuning = self._init_tuning()

        # ── sounddevice 장치 탐색 ──
        dev_idx = (dev_idx_param if dev_idx_param >= 0
                   else self._find_respeaker_index())
        if dev_idx is None:
            self.get_logger().error('ReSpeaker 오디오 장치를 찾을 수 없습니다.')
            raise RuntimeError('ReSpeaker not found')

        # 6채널 펌웨어 전제이지만, 실제 열리는지 사전 확인
        self._verify_stream_settings(dev_idx)

        # ── 공유 상태 (오디오 콜백 ↔ 타이머) ──
        self._lock      = threading.Lock()
        self._amplitude = 0.0

        # ── sounddevice 스트림 시작 ──
        try:
            self._stream = sd.InputStream(
                device     = dev_idx,
                samplerate = self.SAMPLE_RATE,
                channels   = self.CHANNELS,
                dtype      = 'int16',
                blocksize  = self.CHUNK,
                callback   = self._audio_callback,
            )
            self._stream.start()
        except sd.PortAudioError as e:
            self.get_logger().error(
                f'스트림 열기 실패 (channels={self.CHANNELS}, '
                f'samplerate={self.SAMPLE_RATE}): {e}. '
                f'다른 프로세스(PipeWire/PulseAudio)가 이미 이 장치를 점유 중일 '
                f'수도 있습니다 — `fuser -v /dev/snd/*`로 확인해 보세요.'
            )
            raise RuntimeError('Failed to open audio input stream') from e

        # ── 퍼블리셔 / 타이머 ──
        self.publisher = self.create_publisher(SoundEvent, '/sound_events', 10)
        self.create_timer(1.0 / rate, self._publish_event)

        self.get_logger().info(
            f'Sound Localizer 시작  |  장치 idx={dev_idx}  |  '
            f'임계={self.threshold}dB  |  오프셋={self.angle_offset}°  |  '
            f'{self.CHANNELS}ch @ {self.SAMPLE_RATE}Hz'
        )

    # ────────────────────────────────────────────────────
    def _init_tuning(self):
        """DOA_2.py와 동일: usb.core.find → Tuning(dev), 기동 시 direction 1회 검증."""
        dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if dev is None:
            self.get_logger().error(
                'ReSpeaker(XVF3000) USB 장치를 찾지 못했습니다 '
                f'(VID=0x{VENDOR_ID:04x}, PID=0x{PRODUCT_ID:04x}). '
                'USB 연결과 udev 권한(또는 sudo 실행)을 확인하세요.'
            )
            raise RuntimeError('ReSpeaker USB device not found')

        tuning = Tuning(dev)
        try:
            d = tuning.direction          # DOA_2.py의 첫 print와 동일한 검증
            v = tuning.is_voice()
        except Exception as e:
            self.get_logger().error(
                f'XVF3000 튜닝 인터페이스 검증 실패: {e}. '
                '6채널 4.0.0 펌웨어(6_channels_dfu_4.0.0_firmware.bin)로 '
                '플래시되어 있는지, sudo/udev 권한이 있는지 확인하세요.'
            )
            raise RuntimeError('XVF3000 tuning interface not responding') from e

        self.get_logger().info(
            f'XVF3000 튜닝 인터페이스 확인 완료 → 초기 DOA={d}°, VAD={v}'
        )
        return tuning

    def _verify_stream_settings(self, dev_idx):
        info = sd.query_devices(dev_idx)
        try:
            sd.check_input_settings(
                device     = dev_idx,
                channels   = self.CHANNELS,
                samplerate = self.SAMPLE_RATE,
                dtype      = 'int16',
            )
        except sd.PortAudioError as e:
            self.get_logger().error(
                f'"{info["name"]}" (idx={dev_idx})가 '
                f'{self.CHANNELS}ch/{self.SAMPLE_RATE}Hz/int16으로 열리지 '
                f'않습니다: {e}. 6채널 펌웨어가 맞는지 확인하세요 '
                f'(`arecord -D hw:CARD=ArrayUAC10 --dump-hw-params -d 1 /dev/null`).'
            )
            raise RuntimeError('6-channel stream not available') from e

    def _find_respeaker_index(self):
        for i, dev in enumerate(sd.query_devices()):
            if 'respeaker' in dev['name'].lower() or 'arrayuac' in dev['name'].lower():
                self.get_logger().info(f'ReSpeaker 자동 탐지: idx={i}  "{dev["name"]}"')
                return i
        return None

    # ────────────────────────────────────────────────────
    def _audio_callback(self, indata, frames, time, status):
        """PortAudio 스레드. RMS 계산만 하고 USB는 절대 건드리지 않는다."""
        try:
            ch0 = indata[:, self.CH_PROCESSED].astype(np.float32)
            rms = np.sqrt(np.mean(ch0 ** 2)) + 1e-9
            db  = 20.0 * np.log10(rms / 32768.0) + 96.0
            with self._lock:
                self._amplitude = float(db)
        except Exception as e:
            self.get_logger().warn(f'오디오 콜백 오류: {e}')

    # ────────────────────────────────────────────────────
    def _publish_event(self):
        """ROS 타이머 스레드. DOA/VAD 폴링(USB)과 이벤트 발행을 전담."""
        with self._lock:
            amplitude = self._amplitude

        if amplitude < self.threshold:
            return

        # DOA_2.py의 루프 본문과 동일한 폴링 — 일시적 USB 오류면 이번 프레임만 skip
        try:
            raw_doa   = self._tuning.direction        # 0 ~ 359°
            is_active = bool(self._tuning.is_voice()) # VOICEACTIVITY
        except Exception as e:
            self.get_logger().warn(f'XVF3000 폴링 실패 (이번 프레임 skip): {e}')
            return

        # 0~359° → -180~180° 변환 후 장착 오프셋 보정
        angle = float(raw_doa % 360)
        if angle > 180.0:
            angle -= 360.0
        angle = (angle + self.angle_offset + 180.0) % 360.0 - 180.0

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
            f'[SoundEvent] raw={raw_doa}°  angle={angle:.1f}°  '
            f'amp={amplitude:.1f}dB  conf={confidence:.2f}  vad={is_active}'
        )

    # ────────────────────────────────────────────────────
    def destroy_node(self):
        try:
            self._stream.stop()
            self._stream.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SoundLocalizerNode()
        rclpy.spin(node)
    except RuntimeError as e:
        print(f'[ERROR] {e}')
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()