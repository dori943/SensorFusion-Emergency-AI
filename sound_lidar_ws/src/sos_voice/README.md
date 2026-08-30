# sos_voice

노인 응급상황 소리 감지 ROS2 패키지 (라즈베리파이용, 경량 CNN 기반, distillation으로 학습).

## 구조

```
sos_voice/
  package.xml
  setup.py
  setup.cfg
  resource/sos_voice
  sos_voice/
    __init__.py
    inference.py               <- 모델 교체 시 이 파일만 바꾸면 됨
    emergency_detector_node.py <- ROS2 노드 (팀원 파일)
  models/
    model_distilled_quantized.pt   <- Colab에서 export한 모델 (직접 넣어야 함, models/PUT_MODEL_HERE.txt 참고)
```

## 설치

1. `models/` 폴더에 Colab 노트북 11절("라즈베리파이 배포용 변환")에서 만든
   `model_distilled_quantized.pt`를 넣으세요 (`models/PUT_MODEL_HERE.txt` 참고).
2. 라즈베리파이에 필요한 패키지 설치:
   ```bash
   pip install torch --index-url https://download.pytorch.org/whl/cpu
   pip install numpy librosa
   ```
3. ROS2 workspace의 `src/` 아래에 이 폴더(`sos_voice`)를 넣고 빌드:
   ```bash
   cd ~/ros2_ws
   colcon build --packages-select sos_voice
   source install/setup.bash
   ```
   빌드 시 `models/model_distilled_quantized.pt`가 `install/sos_voice/share/sos_voice/models/`로
   자동 설치됩니다 (setup.py의 data_files에 반영되어 있음).

## 실행

모델 경로는 패키지 설치 위치 기준으로 **자동으로 찾습니다** (별도 지정 불필요):
```bash
ros2 run sos_voice emergency_detector_node
```

다른 경로의 모델을 쓰고 싶으면 파라미터로 직접 지정 가능:
```bash
ros2 run sos_voice emergency_detector_node \
  --ros-args \
  -p model_path:=/home/pi/other_model.pt \
  -p audio_topic:=/audio/raw \
  -p emergency_threshold:=0.40 \
  -p window_history_size:=5 \
  -p window_alarm_ratio:=0.6
```

- `emergency_threshold`: Colab 노트북에서 threshold_sweep 결과 보고 정한 값을 넣으세요.
- `window_history_size` / `window_alarm_ratio`: 최근 N개 3초 window 중 몇 %가 emergency로
  판단돼야 실제 알람을 울릴지. 3초 clip 하나만 보고 바로 알람하지 않도록 하는 안전장치입니다.

## 토픽

- 구독: `<audio_topic>` (`std_msgs/Float32MultiArray`) - 마이크 오디오 청크
- 발행: `/emergency_detector/alarm` (`std_msgs/Bool`) - 알람 여부
- 발행: `/emergency_detector/probs` (`std_msgs/String`, JSON) - 매 3초 window의 확률값
  (모니터링/로깅/디버깅용, `ros2 topic echo /emergency_detector/probs`로 실시간 확인 가능)

## 마이크 드라이버가 다른 메시지 타입을 쓸 때

`emergency_detector_node.py`의 `audio_callback` 함수 안, 메시지 파싱하는 부분만
사용 중인 드라이버 메시지 타입에 맞게 수정하면 됩니다. 그 아래 로직(버퍼링, 예측,
멀티윈도우 판단)은 손댈 필요 없습니다.

## 전처리 관련 중요 노트

`inference.py`는 **RMS 정규화를 의도적으로 적용하지 않습니다.** 실사용 마이크(조용한 방,
작은 목소리 등) 입력에 RMS 정규화를 걸면 미세한 배경 잡음까지 크게 증폭되어 모델이
emergency로 오판하는 문제가 실측으로 확인되어 제거했습니다. 원본 음량 그대로 리샘플링 +
길이 맞춤만 하는 것이 실전 환경에서 더 안정적이었습니다.

## 모델 교체 시

`inference.py`의 `EmergencyDetector` 클래스가 아래 인터페이스만 유지하면,
내부 구현(다른 모델 구조, 다른 프레임워크 등)이 뭐든 노드 코드는 그대로 씁니다:

```python
detector = EmergencyDetector(model_path)
probs = detector.predict(audio_np_array, sample_rate)
# -> {'emergency': float, 'normal_speech': float, 'background': float}
```
