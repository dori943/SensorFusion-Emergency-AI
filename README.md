# SensorFusion Emergency AI  -  프라이버시 보호형 낙상 및 구조요청 감지 시스템

3D LiDAR, 마이크 어레이, IMU 세 종류의 이질 센서를 융합해 실내에서
**낙상(Fall)**과 **구조요청(SOS)**을 자동 감지하는 프라이버시 보호형 시스템입니다.

- **CCTV 대체**: 얼굴·의복 등 개인 식별 정보를 담지 않는 3D 포인트클라우드 기반
- **웨어러블 불필요**: 평상시 아무것도 착용하지 않아도 감지 동작
- **다중 센서 융합**: 단일 센서의 오탐을 상호 검증으로 상쇄

---

## 저장소 구조

이 레포지토리는 두 개의 하위 프로젝트로 구성됩니다.

```
SensorFusion-Emergency-AI/
├── sound_lidar_ws/              백엔드 (ROS 2)
│   ├── src/
│   │   ├── lidar_focus/         LiDAR 파이프라인
│   │   ├── sound_localizer/     오디오 파이프라인
│   │   ├── sos_voice/           음성 응급 분류기
│   │   ├── imu_fall_sos/        IMU 노드
│   │   ├── emergency_fusion/    융합 판정 (fall / sos)
│   │   └── kakao_alert/         보호자 알림
│   ├── tools/                   진단 스크립트
│   └── run_all_nodes.sh         tmux 일괄 실행
│
└── Sensor-Fusion-Home-CCTV/     프론트엔드 (웹 3D 뷰어)
    └── mobile-app/              three.js + rosbridge 클라이언트
```

---

## 시스템 개요

### 하드웨어

| 구분 | 장비 |
|---|---|
| 3D 센서 | Unitree L1 LiDAR |
| 오디오 | ReSpeaker USB 4-Mic Array |
| 관성 | WT901BLECL BLE IMU |
| 연산 | 라즈베리파이 (Ubuntu 24.04) |

### 데이터 흐름

```
[LiDAR] ─→ 포인트클라우드 처리 ─┐
[Mic]   ─→ 방향 + 음성 분류    ├→ 융합 판정 ─→ 카카오톡 알림
[IMU]   ─→ 충격/두드림 감지   ─┘         └→ 웹 3D 뷰어
```

---

## 빠른 시작

### 요구사항

- Ubuntu 24.04, ROS 2 Jazzy
- Python 3.12
- Unitree LiDAR ROS 2 드라이버

### 백엔드 실행

```bash
cd sound_lidar_ws

# 의존성 설치
pip install --break-system-packages \
    numpy scipy scikit-learn sounddevice bleak pyusb requests
sudo apt install ros-jazzy-rosbridge-suite
rosdep install --from-paths src --ignore-src -r -y

# 음성 분류기는 별도 venv (torch 격리)
python3 -m venv venv
source venv/bin/activate
pip install torch transformers
deactivate

# 빌드
colcon build --symlink-install
source install/setup.bash

# 카카오톡 설정
mkdir -p ~/.config/kakao_alert
cp src/kakao_alert/kakao_alert/kakao_config.example.json \
   ~/.config/kakao_alert/kakao_config.json
# → 토큰과 receiver_uuids 입력

# 전체 실행
chmod +x run_all_nodes.sh
./run_all_nodes.sh
```

### 프론트엔드 실행

```bash
cd Sensor-Fusion-Home-CCTV
python3 -m http.server 8080
```

브라우저에서:
```
http://localhost:8080/mobile-app/assets/webview/?rosbridge=ws://라즈베리파이IP:9090
```

---

## tmux 조작

`run_all_nodes.sh`는 백그라운드 tmux 세션 `sos_system`을 만듭니다.

```bash
tmux attach -t sos_system        # 세션 접속
# Ctrl+b, 숫자키   창 이동
# Ctrl+b, d        나가기 (노드는 계속 실행)
tmux kill-session -t sos_system  # 전체 종료
```

---

## 알림 예시

`/emergency/fall_alert`와 `/emergency/sos_alert`로 JSON이 발행되고,
`kakao_alert` 노드가 이를 받아 보호자에게 카카오톡을 보냅니다.
알림에는 판정 근거(어떤 센서가 얼마나 기여했는지)가 포함됩니다.

---

## 보안 주의

카카오 API 토큰과 IMU MAC 주소는 절대 커밋하지 마세요.
`.gitignore`에 이미 포함되어 있으며, 실제 값은 `kakao_config.example.json`을
복사해 별도로 관리합니다.

토큰이 노출되면 즉시 [카카오 개발자 콘솔](https://developers.kakao.com)에서
재발급하세요.

---

## 라이선스

Apache License 2.0

three.js는 MIT, roslibjs는 BSD, WavLM은 MIT 라이선스를 따릅니다.

---

## 문의

이슈 페이지를 통해 문의해주세요.