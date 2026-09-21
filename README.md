# physical_ai_ws

ROS 2 Jazzy + Gazebo Harmonic(gz-sim)으로 만드는 차동구동 AMR(`physical_ai_amr`) 학습 프로젝트. macOS에서 Docker로 ROS 2를 돌리고, 시뮬레이션은 Docker 컨테이너 안에서 헤드리스로 실행한다.

## 요구 사항

- Docker Desktop (macOS, Apple Silicon)

## 빌드 & 실행

```bash
docker compose up -d
docker compose exec ros2 bash -lc "source /opt/ros/jazzy/setup.bash && cd /workspace && colcon build --packages-select physical_ai_amr"
```

Dockerfile에 Nav2(`ros-jazzy-navigation2`, `ros-jazzy-nav2-bringup`)가 포함돼 있다. Dockerfile을 바꿨거나 이미지를 처음 받는 경우엔 `docker compose up -d --build`로 재빌드한다.

이후 컨테이너 안에서 실행하는 명령은 모두 아래를 먼저 source한다고 가정한다.

```bash
source /opt/ros/jazzy/setup.bash && source /workspace/install/setup.bash
```

## 테스트

`robot_command_node`의 순찰/회피/Nav2 핸드오프 로직에 대한 단위 테스트(시뮬레이션 불필요, `Twist`/액션 클라이언트를 mock으로 검사):

```bash
colcon test --packages-select physical_ai_amr --event-handlers console_direct+
```

## 시뮬레이션 실행

```bash
ros2 launch physical_ai_amr simulation.launch.py
```

Gazebo 서버(`empty_world.sdf`, 장애물 박스 1개 포함), `robot_state_publisher`, `ros_gz_bridge`(`/cmd_vel`, `/odom`, `/tf`)를 띄우고 로봇을 원점(0,0)에 스폰한다. Docker 컨테이너에는 디스플레이가 없어 GUI 없이 서버만 실행된다(`-s -r`).

## 순찰 + 장애물 회피 + Nav2 목표점 이동

```bash
ros2 launch physical_ai_amr navigation.launch.py   # Nav2 스택 (아래 참고)
ros2 run physical_ai_amr robot_command_node
```

기본 상태에서는 `/odom`을 구독해 반응형으로 순찰한다.

- 알려진 장애물 위치(`2.0, 0.0`)에서 1m 이내로 접근하면 2초간 후진+회전 후 다시 전진 — 즉각 반응이 필요해서 Nav2를 거치지 않는 단순 반사 동작이다.
- 원점에서 4m(`PATROL_RADIUS`) 밖으로 벗어나면 Nav2에게 원점 복귀를 맡긴다. 처음엔 이것도 고정 방향 후진+회전으로 처리했는데, 방향 보장이 없어 경계선에서 영원히 왔다갔다 진동하는 버그가 있었다 — Nav2의 전역 경로 계획으로 바꿔서 확실히 경계 안으로 돌아오게 고쳤다.

실제 라이다/범퍼 센서가 아니라 오도메트리 기반 "가짜 센서"다. 이유는 아래 제약 참고.

`geometry_msgs/PoseStamped`를 `/go_to_pose`에 보내면 순찰을 멈추고 Nav2의 `navigate_to_pose` 액션으로 그 목표까지 이동한 뒤, 도착하면 자동으로 순찰을 재개한다.

```bash
ros2 topic pub --once /go_to_pose geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: map}, pose: {position: {x: -3.0, y: -2.0, z: 0.0}, orientation: {w: 1.0}}}"
```

내부적으로 `robot_command_node`가 직접 `/cmd_vel`을 발행하는 것과 Nav2의 `controller_server`가 발행하는 것 중 하나만 활성화되도록 상태를 전환한다(순찰 중엔 Nav2 대기, 목표 이동 중엔 순찰 정지) — 두 발행자가 동시에 `/cmd_vel`을 놓고 싸우지 않는다.

## Nav2 스택만 따로 쓰기

```bash
ros2 launch physical_ai_amr navigation.launch.py
```

사전 제작한 정적 지도(`maps/patrol_world.yaml`, world의 장애물 위치를 그대로 반영)로 `map_server` + `planner_server` + `controller_server`(MPPI) + `bt_navigator` + `behavior_server`를 띄운다. AMCL 대신 `map`→`odom` 항등 변환을 쓴다(시뮬레이션 오도메트리가 드리프트 없음을 이용).

`robot_command_node` 없이 액션을 직접 호출할 수도 있다:

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 4.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
```

지도를 다시 만들어야 하면(장애물 위치가 바뀌는 등):

```bash
python3 src/physical_ai_amr/maps/generate_map.py
```

## 알려진 제약 (이 Docker 환경 한정)

- **네이티브 Gazebo GUI 불가**: Docker Desktop이 만드는 vpnkit/utun 인터페이스가 gz-transport 멀티캐스트 디스커버리를 깨뜨려서 `scripts/mac/gz_server.sh` + `gz_gui.sh`로 macOS 네이티브 GUI를 띄워도 화면이 안 뜬다. 시각적 확인이 필요하면 `/odom`, `/cmd_vel` 등을 echo해서 검증한다.
- **라이다/카메라 불가**: 컨테이너에 GPU 렌더링 접근이 없어 `gpu_lidar` 센서가 렌더 스레드 초기화에서 무한 대기한다. `type="lidar"`는 이 gz-sim 버전 자체가 미지원.
- **Contact(범퍼) 센서 불가**: 물리 충돌 자체는 되지만(확인함), contact 센서가 겹침 상황에서도 메시지를 발행하지 않는 원인 미상의 버그가 있다.
- 위 세 가지 때문에 장애물 회피는 실제 센서 대신 오도메트리 거리 계산으로, Nav2는 SLAM/AMCL 대신 사전 제작 지도 + 항등 변환으로 대체했다.

## 프로젝트 구조

```
src/physical_ai_amr/
  physical_ai_amr/robot_command_node.py  # 순찰 + 장애물 회피 + Nav2 핸드오프 노드
  test/test_robot_command_node.py        # 위 노드의 단위 테스트
  urdf/physical_ai_amr.urdf.xacro        # 로봇 모델 (섀시, 바퀴, 캐스터)
  worlds/empty_world.sdf                 # 시뮬레이션 월드 (장애물 박스 포함)
  launch/simulation.launch.py            # Gazebo + 브릿지 + 스폰
  launch/navigation.launch.py            # Nav2 스택
  maps/                                  # 정적 지도 + 생성 스크립트
  config/nav2_params.yaml                # 축소판 Nav2 파라미터
scripts/mac/                             # 네이티브 Gazebo 스크립트 (현재 GUI 미작동, 위 제약 참고)
```
