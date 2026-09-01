# sound_localizer/sound_localizer_node.py
#
# DoA  : XVF3000 칩 내장 알고리즘을 USB로 직접 폴링
#        (respeaker 공식 usb_4_mic_array/tuning.py의 Tuning 클래스를 그대로 사용)
# 음량 : sounddevice 6채널 스트림 ch0에서 RMS → dBFS 변환
# VAD  : XVF3000 VOICEACTIVITY 레지스터 폴링

import threading
from collections import deque

import numpy as np
import sounddevice as sd
import usb.core
import usb.util

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from sound_interfaces.msg import SoundEvent

# respeaker 공식 usb_4_mic_array/tuning.py를 그대로 vendor한 모듈.
# 자체 ctrl_transfer 인코딩(파라미터별 wIndex 등)을 직접 구현하지 않고,
# 이미 검증된 프로토콜 구현을 그대로 사용해 인코딩 버그 가능성을 없앤다.
# (array.tostring() → array.tobytes() 패치가 되어 있어야 Python 3.9+에서 동작함)
from .vendor.tuning import Tuning


# ── XVF3000 USB 튜닝 레지스터 접근 헬퍼 ──────────────────────────
VENDOR_ID  = 0x2886
PRODUCT_ID = 0x0018


class XVF3000Tuning:
    """respeaker 공식 Tuning 클래스를 감싸는 얇은 어댑터.

    노드의 나머지 코드는 read(name) 인터페이스만 알면 되도록 유지해서,
    _verify_doa_capability / _get_doa / _audio_callback을 손대지 않는다.
    """

    def __init__(self, dev):
        self._t = Tuning(dev)

    def read(self, name: str):
        try:
            if name == 'DOAANGLE':
                # 공식 모듈은 direction 프로퍼티로 DOA를 노출함
                return self._t.direction
            return self._t.read(name)
        except Exception:
            return None


# ── 메인 노드 ─────────────────────────────────────────────────────
class SoundLocalizerNode(Node):

    SAMPLE_RATE      = 16000
    CHANNELS_WANTED  = 6   # 6채널 펌웨어 기준 기대값 (실제 장치에 맞춰 자동 보정됨)
    CHUNK            = 1024
    CH_PROCESSED     = 0

    def __init__(self):
        super().__init__('sound_localizer')

        self.declare_parameter('threshold_db',       50.0)
        self.declare_parameter('publish_rate',       10.0)
        self.declare_parameter('angle_offset',        0.0)
        self.declare_parameter('audio_device_index',   -1)
        # 하드웨어 DoA(XVF3000 튜닝)를 못 읽는 펌웨어(UAC1.0 등)에서도
        # 최소한 진폭/VAD 이벤트는 흘려보낼지 여부. 기본 False = 기존 동작 유지.
        self.declare_parameter('allow_angle_unavailable', False)

        # ── 원시 오디오 중계 ──────────────────────────────────
        # ALSA/PortAudio 장치는 한 프로세스만 열 수 있다. sos 분류기가 별도
        # 프로세스에서 같은 마이크를 직접 열려고 하면 PortAudioError로 죽는다.
        # 그래서 이 노드가 장치를 단독 점유하고, 원시 오디오를 토픽으로
        # 흘려보내 다른 노드들이 각자 목적대로 쓰게 한다.
        #
        #   ReSpeaker ─(단독 점유)─ sound_localizer ─┬→ /sound_events (DoA)
        #                                            └→ /audio/raw   (원시 PCM)
        #                                                    ↓
        #                                        emergency_detector_node
        #
        # 분류기를 같은 프로세스에 합치지 않는 이유: WavLM 추론은 CPU에서
        # 클립당 수백 ms가 걸려서, 같은 프로세스에 두면 그동안 DoA 발행이
        # 멈추고 오디오 콜백이 밀려 버퍼 오버런이 난다.
        self.declare_parameter('publish_audio', True)
        self.declare_parameter('audio_topic', '/audio/raw')
        # 분류기는 16kHz float32 mono를 기대한다. ch0(=칩이 처리한 빔포밍
        # 출력)을 int16 → float32 [-1,1]로 정규화해 내보낸다.
        self.declare_parameter('audio_normalize', True)

        self.threshold    = self.get_parameter('threshold_db').value
        self.angle_offset = self.get_parameter('angle_offset').value
        rate              = self.get_parameter('publish_rate').value
        dev_idx_param     = self.get_parameter('audio_device_index').value
        self._allow_no_angle = self.get_parameter('allow_angle_unavailable').value
        self._publish_audio = self.get_parameter('publish_audio').value
        self._audio_normalize = self.get_parameter('audio_normalize').value

        # 오디오 콜백은 sounddevice의 별도 스레드에서 돈다. rclpy 퍼블리셔를
        # 그 스레드에서 직접 호출하지 않고 큐에 넣은 뒤 ROS 타이머에서
        # 꺼내 발행한다(스레드 안전성 + 콜백 지연 최소화).
        # maxlen을 둬서 타이머가 밀려도 메모리가 무한정 늘지 않게 한다
        # (오래된 오디오를 붙잡고 있느니 버리는 게 낫다).
        self._audio_q = deque(maxlen=64)

        # USB 초기화
        self._tuning = self._init_usb()

        # sounddevice 장치 탐색
        dev_idx = (dev_idx_param if dev_idx_param >= 0
                   else self._find_respeaker_index())
        if dev_idx is None:
            self.get_logger().error('ReSpeaker 장치를 찾을 수 없습니다.')
            raise RuntimeError('ReSpeaker not found')

        # 실제 장치가 지원하는 입력 채널 수로 보정 (하드코딩 6채널 가정 금지)
        self.CHANNELS = self._resolve_channel_count(dev_idx)

        # 하드웨어 DoA/VAD 튜닝 인터페이스가 실제로 동작하는지 스트림 시작 "전"에 확인.
        # 콜백은 stream.start() 직후 즉시 비동기로 돌기 시작하므로, 이 속성은
        # 콜백이 참조하기 전에 반드시 먼저 정해져 있어야 한다 (레이스 컨디션 방지).
        self._hw_doa_available = self._verify_doa_capability()

        # 공유 상태
        self._lock       = threading.Lock()
        self._amplitude  = 0.0
        self._vad_active = False

        # sounddevice 스트림 시작
        try:
            self._stream = sd.InputStream(
                device         = dev_idx,
                samplerate     = self.SAMPLE_RATE,
                channels       = self.CHANNELS,
                dtype          = 'int16',
                blocksize      = self.CHUNK,
                callback       = self._audio_callback,
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

        # 퍼블리셔 / 타이머
        self.publisher = self.create_publisher(SoundEvent, '/sound_events', 10)
        self.create_timer(1.0 / rate, self._publish_event)

        if self._publish_audio:
            audio_topic = self.get_parameter('audio_topic').value
            # QoS depth를 넉넉히: 분류기가 3초 윈도우를 채워야 하는데
            # 중간에 청크가 버려지면 윈도우 경계가 어긋난다.
            self.audio_publisher = self.create_publisher(
                Float32MultiArray, audio_topic, 50)
            # CHUNK=1024 @16kHz = 64ms/청크 → 초당 약 15.6개.
            # 50Hz로 비워서 큐에 쌓이지 않게 한다.
            self.create_timer(0.02, self._drain_audio)
            self.get_logger().info(
                f'원시 오디오 중계 활성화 → {audio_topic} '
                f'({self.SAMPLE_RATE}Hz, float32, mono ch{self.CH_PROCESSED})')
        else:
            self.audio_publisher = None

        self.get_logger().info(
            f'Sound Localizer 시작  |  장치 idx={dev_idx}  |  '
            f'임계={self.threshold}dB  |  오프셋={self.angle_offset}°'
        )

    def _init_usb(self):
        dev = usb.core.find(idVendor=VENDOR_ID, idProduct=PRODUCT_ID)
        if dev is None:
            self.get_logger().warn('XVF3000 USB 장치를 찾지 못했습니다.')
            return None

        AUDIO_CLASS = 0x01  # USB Audio class — 이 인터페이스는 절대 건드리지 않는다

        for cfg in dev:
            for intf in cfg:
                if intf.bInterfaceClass == AUDIO_CLASS:
                    # 오디오(=ALSA 카드) 인터페이스는 커널 드라이버(snd-usb-audio)가
                    # 계속 잡고 있어야 sounddevice/PortAudio가 열 수 있음.
                    # 여기를 detach하면 ALSA 카드 자체가 사라져서
                    # "Cannot get card index" 류의 에러로 이어진다. 절대 detach 금지.
                    continue
                if dev.is_kernel_driver_active(intf.bInterfaceNumber):
                    try:
                        dev.detach_kernel_driver(intf.bInterfaceNumber)
                    except usb.core.USBError:
                        pass

        self.get_logger().info('XVF3000 USB 연결 완료 → DoA 하드웨어 모드')
        return XVF3000Tuning(dev)

    def _resolve_channel_count(self, dev_idx):
        """query_devices()가 보고하는 max_input_channels는 ALSA가 광고하는
        이론상 최대치일 뿐, 그 샘플레이트/포맷 조합으로 실제 열리는지는
        보장하지 않는다 (PortAudioError -9993 'Illegal combination of I/O devices'가
        바로 이 케이스). 그래서 숫자만 비교하지 않고 sd.check_input_settings()로
        큰 채널 수부터 실제로 열리는지 하나씩 negotiate한다."""
        info = sd.query_devices(dev_idx)
        max_ch = info['max_input_channels']

        if max_ch <= 0:
            self.get_logger().error(
                f'장치 idx={dev_idx} "{info["name"]}"에 입력 채널이 없습니다.'
            )
            raise RuntimeError('No input channels on selected device')

        for channels in range(min(self.CHANNELS_WANTED, max_ch), 0, -1):
            try:
                sd.check_input_settings(
                    device     = dev_idx,
                    channels   = channels,
                    samplerate = self.SAMPLE_RATE,
                    dtype      = 'int16',
                )
            except sd.PortAudioError:
                continue

            if channels < self.CHANNELS_WANTED:
                self.get_logger().warn(
                    f'{self.SAMPLE_RATE}Hz/int16 조합에서 실제로 열리는 채널 수는 '
                    f'{channels}채널입니다 (기대값 {self.CHANNELS_WANTED}채널, '
                    f'장치가 보고한 이론상 최대치 {max_ch}채널). '
                    f'"{info["name"]}"가 6채널 펌웨어가 아닐 가능성이 높습니다. '
                    f'→ {channels}채널로 스트림을 엽니다.'
                )

            if self.CH_PROCESSED >= channels:
                self.get_logger().error(
                    f'CH_PROCESSED={self.CH_PROCESSED} 인덱스가 실제 채널 수'
                    f'({channels})를 벗어납니다. ch0으로 강제 조정합니다.'
                )
                self.CH_PROCESSED = 0

            return channels

        self.get_logger().error(
            f'"{info["name"]}"에서 {self.SAMPLE_RATE}Hz/int16으로 열리는 채널 '
            f'조합을 하나도 찾지 못했습니다. 다른 샘플레이트(예: 48000Hz)를 '
            f'시도하거나 arecord로 실제 지원 포맷을 확인해 보세요: '
            f'`arecord -D hw:1,0 --dump-hw-params -d 1 /dev/null`'
        )
        raise RuntimeError('No valid input channel/samplerate combination found')

    def _verify_doa_capability(self):
        """기동 시 XVF3000 튜닝 레지스터가 실제로 읽히는지 확인.
        UAC1.0(1채널) 펌웨어는 이 벤더 컨트롤 인터페이스 자체를 지원하지 않아서
        항상 None을 반환한다 — 이 경우 조용히 아무 이벤트도 안 나가는 대신
        원인을 명확히 로그로 남긴다."""
        if self._tuning is None:
            return False

        angle_ok = self._tuning.read('DOAANGLE') is not None
        vad_ok   = self._tuning.read('VOICEACTIVITY') is not None

        if not (angle_ok or vad_ok):
            self.get_logger().error(
                '연결된 ReSpeaker가 XVF3000 튜닝 인터페이스에 응답하지 않습니다. '
                '현재 UAC1.0(1채널) 펌웨어로 동작 중이라 하드웨어 DoA/VAD 레지스터를 '
                '지원하지 않을 가능성이 높습니다. Seeed 펌웨어 업데이트 툴로 '
                '"6_channels_firmware"로 재플래시해야 DoA가 정상 동작합니다. '
                f'(allow_angle_unavailable={self._allow_no_angle} → '
                + ('진폭/VAD 이벤트만 발행합니다.' if self._allow_no_angle
                   else '재플래시 전까지 /sound_events가 발행되지 않습니다.')
            )
            return False
        return True

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

            # 원시 오디오 중계 — 임계값(threshold_db)과 무관하게 항상 흘린다.
            # 분류기는 연속된 3초 윈도우가 필요해서, 조용한 구간을 빼먹으면
            # 윈도우 경계가 어긋나고 클립 길이가 맞지 않는다.
            if self._publish_audio:
                out = ch0 / 32768.0 if self._audio_normalize else ch0
                self._audio_q.append(out.copy())

            rms = np.sqrt(np.mean(ch0 ** 2)) + 1e-9
            db  = 20.0 * np.log10(rms / 32768.0) + 96.0

            if self._hw_doa_available:
                vad = bool(self._tuning.read('VOICEACTIVITY'))
            else:
                # 펌웨어가 튜닝 인터페이스를 지원하지 않으면(UAC1.0 등)
                # self._tuning이 존재해도 read()는 항상 None → bool(None)=False로
                # 영원히 고정되는 버그가 있었음. dB 임계값 기반으로 대체.
                vad = db >= self.threshold

            with self._lock:
                self._amplitude  = float(db)
                self._vad_active = vad
        except Exception as e:
            self.get_logger().warn(f'오디오 콜백 오류: {e}')

    def _drain_audio(self):
        """오디오 콜백 스레드가 쌓아둔 청크를 ROS 스레드에서 발행한다."""
        if self.audio_publisher is None:
            return
        n = 0
        while self._audio_q:
            try:
                chunk = self._audio_q.popleft()
            except IndexError:
                break
            msg = Float32MultiArray()
            msg.data = chunk.tolist()
            self.audio_publisher.publish(msg)
            n += 1
            if n >= 16:      # 한 틱에 몰아치지 않도록 상한
                break

    def _publish_event(self):
        with self._lock:
            amplitude = self._amplitude
            is_active = self._vad_active

        if amplitude < self.threshold:
            return

        if self._hw_doa_available:
            angle = self._get_doa()
            if angle is None:
                # 순간적인 USB 통신 실패 등 일시적 오류 → 이번 프레임만 skip
                return
        elif self._allow_no_angle:
            # 펌웨어가 DoA를 지원하지 않음 → 방향 정보 없이 진폭/VAD만 발행
            angle = float('nan')
        else:
            # 기존 동작 유지: DoA 없으면 발행 안 함 (원인은 기동 시 로그로 이미 안내됨)
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
