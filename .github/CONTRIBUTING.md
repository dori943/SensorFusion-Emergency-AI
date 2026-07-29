# Contributing Guide

이 프로젝트(라즈베리파이5 기반 실내 응급상황 감지 - ROS2 센서 퓨전)에 기여하는 방법입니다.

## 1. 브랜치 전략

| 브랜치 | 용도 |
|---|---|
| `main` | 항상 실행 가능한 안정 버전 |
| `feature/*` | 신규 기능/센서 추가 (예: `feature/gas-leak-detection`) |
| `fix/*` | 버그 수정 (예: `fix/imu-noise-filter`) |
| `docs/*` | 문서 작업 |

- `main`에 직접 push 금지, 항상 PR을 통해서만 머지
- PR은 최소 1인 리뷰 승인 필요

## 2. 커밋 메시지 컨벤션 (Conventional Commits)

```
<type>(<scope>): <설명>

예시)
feat(fall-detection): IMU 각속도 기반 낙상 판별 로직 추가
fix(sensor-fusion): PIR 센서 타임아웃 시 노드 크래시 수정
docs(readme): 라즈베리파이5 설치 가이드 추가
refactor(alert-node): 콜백 구조 정리
test(fall-detection): 낙상 오탐 케이스 테스트 추가
```

**type 종류**
- `feat`: 새 기능
- `fix`: 버그 수정
- `refactor`: 기능 변화 없는 코드 개선
- `docs`: 문서
- `test`: 테스트 추가/수정
- `chore`: 빌드, 설정, 의존성 등

## 3. 코드 스타일

- Python: **PEP8** 준수, `black` + `isort` + `flake8` 사용 권장
- ROS2 노드/토픽 네이밍은 `snake_case` 통일
  - 토픽 예: `/sensor_fusion/emergency_status`
  - 노드 예: `fall_detection_node`
- 매직 넘버 대신 설정 파일(`config/*.yaml`)에서 파라미터 관리

## 4. 이슈 & PR 규칙

- 버그/기능 제안은 반드시 이슈 템플릿을 통해 등록
- PR 생성 시 관련 이슈 번호를 `Closes #이슈번호` 형식으로 연결
- **감지 로직(낙상/화재/가스 등) 관련 PR**은 반드시 오탐(false positive)/미탐(false negative) 테스트 결과를 PR 설명에 포함

## 5. 코드 실행 관련 안내

- 이 저장소는 코드 공유용이며, 자동 배포는 하지 않습니다.
- 각자 로컬/라즈베리파이 환경에서 직접 `git clone` 후 아래 순서로 빌드/실행합니다.

```bash
git clone <repo-url>
cd <repo>
source /opt/ros/<distro>/setup.bash
pip install -r requirements.txt
colcon build --symlink-install
source install/setup.bash
ros2 launch <package_name> <launch_file>.py
```

## 6. 리뷰 시 확인 사항 (리뷰어용)

- [ ] 커밋 메시지 컨벤션 준수 여부
- [ ] 실기(라즈베리파이) 테스트 여부 명시됐는지
- [ ] 감지 로직 변경 시 회귀 테스트 결과 포함 여부
- [ ] 불필요한 디버그 코드/주석 없는지
