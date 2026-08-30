#!/bin/bash
#
# run_all_nodes.sh
# LiDAR + Sound + IMU + Emergency Fusion + Kakao + Web CCTV 전체 실행
#
# 사용법:
#   chmod +x run_all_nodes.sh
#   ./run_all_nodes.sh
#
# 종료:
#   tmux kill-session -t sos_system
#
# tmux 창 이동: Ctrl+b 후 숫자키 (0~15)  또는  Ctrl+b, n / p
# 세션 나가기(백그라운드 유지): Ctrl+b, d
# 다시 접속: tmux attach -t sos_system

SESSION="sos_system"

ROS_SETUP="source /opt/ros/jazzy/setup.bash"
LIDAR_WS_SETUP="source ~/ros2_ws/install/setup.bash"
SOUND_WS_SETUP="source ~/sound_lidar_ws/install/setup.bash"
VENV_SETUP="source ~/sound_lidar_ws/venv/bin/activate"

# 카카오 설정 파일. 워크스페이스 밖에 두는 게 안전하다
# (rm -rf install 로 토큰 날아가는 것 방지). 없으면 패키지 안의 기본 경로 사용.
KAKAO_CONFIG_EXPORT="export KAKAO_CONFIG=$HOME/.config/kakao_alert/kakao_config.json"

# ─────────────────────────────────────────────────────────────
# 사전 점검 — 시연 중 "왜 안 되지" 를 미리 잡는다
# ─────────────────────────────────────────────────────────────
missing=()
[ -f "$HOME/ros2_ws/install/setup.bash" ] || missing+=("~/ros2_ws (LiDAR 드라이버 워크스페이스)")
[ -f "$HOME/sound_lidar_ws/install/setup.bash" ] || missing+=("~/sound_lidar_ws (colcon build 필요)")
[ -d "$HOME/sound_lidar_ws/venv" ] || missing+=("~/sound_lidar_ws/venv (sos_voice용 torch 환경)")
[ -f "$HOME/.config/kakao_alert/kakao_config.json" ] \
    || [ -f "$HOME/sound_lidar_ws/src/kakao_alert/kakao_alert/kakao_config.json" ] \
    || missing+=("kakao_config.json (토큰·UUID 입력 필요)")

if [ ${#missing[@]} -gt 0 ]; then
    echo "[경고] 다음 항목이 없습니다:"
    for m in "${missing[@]}"; do echo "  - $m"; done
    echo "그대로 진행하지만 해당 노드는 실패합니다. Ctrl+C로 중단하거나 5초 뒤 계속..."
    sleep 5
fi

# 이미 같은 이름의 세션이 있으면 종료 후 새로 시작
if tmux has-session -t $SESSION 2>/dev/null; then
    echo "기존 '$SESSION' 세션 종료 후 재시작"
    tmux kill-session -t $SESSION
fi

# ─────────────────────────────────────────────────────────────
# 창 배치는 데이터 흐름 순 (센서 → 처리 → 융합 → 알림 → 뷰)
# ─────────────────────────────────────────────────────────────

# 0: LiDAR 드라이버
tmux new-session -d -s $SESSION -n "lidar_driver"
tmux send-keys -t $SESSION:0 \
    "$ROS_SETUP && $LIDAR_WS_SETUP && ros2 launch unitree_lidar_ros2 launch.py" C-m

# 1: ground_removal (배경 학습에 100프레임 필요 → 뒤 노드들이 뜨는 사이 완료)
tmux new-window -t $SESSION -n "ground_removal"
tmux send-keys -t $SESSION:1 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run lidar_focus ground_removal_node" C-m

# 2: human_bbox
tmux new-window -t $SESSION -n "human_bbox"
tmux send-keys -t $SESSION:2 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run lidar_focus human_bbox_node" C-m

# 3: fall_detection
tmux new-window -t $SESSION -n "fall_detection"
tmux send-keys -t $SESSION:3 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run lidar_focus fall_detection_node" C-m

# 4: sound_localizer (마이크 단독 점유 + /audio/raw 중계)
tmux new-window -t $SESSION -n "sound_localizer"
tmux send-keys -t $SESSION:4 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run sound_localizer sound_localizer_node" C-m

# 5: sound_source_marker (DoA ↔ 사람 트랙 매칭)
tmux new-window -t $SESSION -n "sound_marker"
tmux send-keys -t $SESSION:5 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run sound_localizer sound_source_marker_node" C-m

# 6: sos_voice (WavLM 분류기 — torch venv 필요)
tmux new-window -t $SESSION -n "sos_voice"
tmux send-keys -t $SESSION:6 \
    "$ROS_SETUP && $SOUND_WS_SETUP && $VENV_SETUP && \
python3 /home/user/sound_lidar_ws/install/sos_voice/lib/sos_voice/emergency_detector_node \
--ros-args -p model_path:=/home/user/sound_lidar_ws/src/sos_voice/models/model_distilled_float.pt" C-m

# 7: imu_fall_sos (BLE 연결에 5~10초 걸림)
tmux new-window -t $SESSION -n "imu_fall_sos"
tmux send-keys -t $SESSION:7 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run imu_fall_sos imu_fall_sos_node --ros-args -p save_csv:=false" C-m

# 8: fall_fusion
tmux new-window -t $SESSION -n "fall_fusion"
tmux send-keys -t $SESSION:8 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run emergency_fusion fall_fusion_node" C-m

# 9: sos_fusion
tmux new-window -t $SESSION -n "sos_fusion"
tmux send-keys -t $SESSION:9 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run emergency_fusion sos_fusion_node" C-m

# 10: kakao_alert (최종 판정을 받아 보호자에게 카톡)
#     confirmed/critical 만 발송 — resolved(회복 종료) 알림은 자동 스킵
tmux new-window -t $SESSION -n "kakao_alert"
tmux send-keys -t $SESSION:10 \
    "$ROS_SETUP && $SOUND_WS_SETUP && $KAKAO_CONFIG_EXPORT && ros2 run kakao_alert kakao_alert_node" C-m

# 11: viz_merge (three.js 렌더러용 합본 토픽)
tmux new-window -t $SESSION -n "viz_merge"
tmux send-keys -t $SESSION:11 \
    "$ROS_SETUP && $SOUND_WS_SETUP && ros2 run emergency_fusion viz_merge_node" C-m

# 12: incident_recorder
tmux new-window -t $SESSION -n "incident_recorder"
tmux send-keys -t $SESSION:12 \
    "cd ~/sound_lidar_ws && $ROS_SETUP && source install/setup.bash && ros2 run lidar_focus incident_recorder_node" C-m

# 13: rosbridge websocket (웹 렌더러 통신)
tmux new-window -t $SESSION -n "rosbridge"
tmux send-keys -t $SESSION:13 \
    "cd ~/sound_lidar_ws && $ROS_SETUP && source install/setup.bash && ros2 launch rosbridge_server rosbridge_websocket_launch.xml" C-m

# 14: 웹서버 (CCTV 프론트엔드)
tmux new-window -t $SESSION -n "webserver"
tmux send-keys -t $SESSION:14 \
    "cd ~/Sensor-Fusion-Home-CCTV && python3 -m http.server 8080" C-m

# 15: rviz2 (마지막 — 위 노드들이 토픽 발행을 시작한 뒤에 떠야 즉시 렌더링됨)
tmux new-window -t $SESSION -n "rviz2"
tmux send-keys -t $SESSION:15 "$ROS_SETUP && rviz2" C-m

# ─────────────────────────────────────────────────────────────
echo ""
echo "모든 노드를 tmux 세션 '$SESSION'에서 시작했습니다."
echo ""
echo "  접속:      tmux attach -t $SESSION"
echo "  창 목록:   tmux list-windows -t $SESSION"
echo "  창 이동:   Ctrl+b 다음 숫자키 (0~15)"
echo "  세션 종료: tmux kill-session -t $SESSION"
echo ""
echo "  창 순서 (데이터 흐름 순):"
echo "    0 lidar_driver     1 ground_removal   2 human_bbox"
echo "    3 fall_detection   4 sound_localizer  5 sound_marker"
echo "    6 sos_voice        7 imu_fall_sos"
echo "    8 fall_fusion      9 sos_fusion       10 kakao_alert"
echo "    11 viz_merge       12 incident_recorder"
echo "    13 rosbridge       14 webserver       15 rviz2"
echo ""
echo "  카카오 알림이 실제로 나가는지 시연 전 확인:"
echo "    tmux attach -t $SESSION \\; select-window -t 10"
echo "    (다른 터미널에서)"
echo "    ros2 topic pub --once /emergency/fall_alert std_msgs/msg/String \\"
echo "      '{data: \"{\\\"event_type\\\":\\\"fall_alert\\\",\\\"level\\\":\\\"confirmed\\\",\\\"case_id\\\":1}\"}'"
echo ""
echo "  웹 접속 (라즈베리파이 IP는 hostname -I 로 확인):"
echo "  http://localhost:8080/mobile-app/assets/webview/?rosbridge=ws://192.168.1.100:9090"
echo ""

# 실행 후 바로 세션에 붙어서 보고 싶으면 아래 줄의 주석을 해제
# tmux attach -t $SESSION