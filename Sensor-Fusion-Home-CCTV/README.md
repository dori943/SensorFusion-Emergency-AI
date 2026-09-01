# Raspberry Pi LiDAR Fall Detection — Point Cloud Viewer

`data/pointcloud_frames.bin`은 라즈베리 파이 서버가 보낸 기록 데이터입니다. 파일 안에는 아래 형식의 Float32 프레임이 연속해서 저장되어 있습니다.

```text
[point_count, x1, y1, z1, x2, y2, z2, ...]
```

현재 제공된 기록은 356개 프레임이며, WebView는 이를 10 FPS로 반복 재생합니다. 재생 중에는 하나의 GPU 위치 버퍼를 재사용하므로 프레임마다 대규모 메모리 할당이 일어나지 않습니다.

## 실행 방법

프로젝트 루트에서 정적 HTTP 서버를 실행합니다. `file://`로 HTML을 직접 열면 브라우저 보안 정책 때문에 기록 파일을 읽지 못할 수 있으므로, 반드시 HTTP로 실행하세요.

```powershell
cd C:\code\fall-detection-lidar
python -m http.server 8080
```

그 다음 브라우저에서 아래 주소를 엽니다.

```text
http://localhost:8080/mobile-app/assets/webview/
```

좌측 상단에 프레임별 포인트 수가 표시되고, `data/pointcloud_frames.bin`이 자동으로 렌더링됩니다. Three.js 모듈은 CDN에서 가져오므로 최초 실행 시 인터넷 연결이 필요합니다.

## 다른 기록 파일 재생

동일한 Float32 프레임 형식의 파일을 웹 서버 경로에 둔 뒤 `recording` 쿼리로 지정합니다.

```text
http://localhost:8080/mobile-app/assets/webview/?recording=/data/another_capture.bin
```

모바일 네이티브 브리지에서 실시간 데이터를 보낼 때는 `window.renderPointCloud(arrayBuffer)`를 호출하면 됩니다. 입력은 `serializer.pack_xyz_float32()`가 생성하는 한 프레임의 bytes여야 합니다.

## ROS2 실시간 연결

WebView는 `rosbridge_suite`를 통해 다음 시각화 전용 토픽을 구독합니다.

- `/viz/scene_cloud` (`sensor_msgs/msg/PointCloud2`): `x`, `y`, `z`, `rgb` 필드가 포함된 합성 장면
- `/viz/scene_meta` (`std_msgs/msg/String`): bbox, 트랙, 낙상 상태 JSON

라즈베리 파이의 ROS2 환경에서 rosbridge WebSocket 서버를 실행합니다.

```bash
sudo apt install ros-$ROS_DISTRO-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

WebView URL에는 rosbridge 주소를 지정합니다. 기본 rosbridge 포트는 `9090`입니다.

```text
http://<web-server>:8080/mobile-app/assets/webview/?rosbridge=ws://<raspberry-pi>:9090
```

두 구독 모두 `reliable`, `transient_local`, depth 1 QoS를 요청하므로 늦게 연결된 WebView도 마지막 장면을 받을 수 있습니다. 클라우드에 저장된 RGB를 그대로 표시하며 사람색 `#ff8c00`과 낙상색 `#ff2828` 포인트는 3배 크게 렌더링합니다.

## 마네킹과 낙상 유지

`/viz/scene_meta`의 `tracks[].center`, `tracks[].size`를 이용해 사람마다 가벼운 3D 마네킹을 표시합니다. 경보의 `track_id`가 일치하고 `level`이 `confirmed`, `critical`, `emergency`, `danger`, `fall` 중 하나이면 마네킹을 빨간색으로 눕히고, 포인트나 트랙이 사라져도 마지막 위치에 유지합니다. `level`이 `resolved` 또는 `recovered`가 되면 잠금을 해제합니다.

## 낙상 사고 클립

`lidar_focus`의 `incident_recorder_node`는 `/viz/scene_cloud`와 `/viz/scene_meta`를 5 FPS로 구독해 최근 30초를 메모리에 유지합니다. 낙상 확정 시점부터 10초를 더 기록한 뒤 다음 위치에 JSON 클립과 목록을 저장합니다.

```text
~/Sensor-Fusion-Home-CCTV/data/incidents/
├── index.json
└── fall_YYYYMMDD_HHMMSS_case-N.json
```

ROS2 워크스페이스에서 노드를 빌드하고 실행합니다.

```bash
cd ~/sound_lidar_ws
colcon build --packages-select lidar_focus
source install/setup.bash
ros2 run lidar_focus incident_recorder_node
```

저장 경로와 녹화 설정은 ROS2 파라미터로 변경할 수 있습니다.

```bash
ros2 run lidar_focus incident_recorder_node --ros-args \
  -p pre_event_sec:=30.0 \
  -p post_event_sec:=10.0 \
  -p record_fps:=5.0 \
  -p output_dir:=$HOME/Sensor-Fusion-Home-CCTV/data/incidents
```

저장된 클립은 `clip` 쿼리로 재생합니다.

```text
http://<web-server>:8080/mobile-app/assets/webview/?clip=/data/incidents/fall_YYYYMMDD_HHMMSS_case-N.json
```
