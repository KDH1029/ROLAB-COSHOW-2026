# 코드 구조

주행에 실제로 쓰는 코드가 어디에 있고, 어떻게 이어지는지 정리했다. 실행 명령은 [RUN.md](RUN.md)에 있다.

## 1. 전체 흐름

```
 [로봇 내부 PC · Jetson Orin NX · ROS 2 Foxy]          [NUC · ROS 2 Jazzy]
                                                     
 D435i ─ realsense2_camera                           XT16 ─ hesai_ros_driver ─ /lidar_points
          │ /camera/color/image_raw                                              │
          ├─ traffic_light_trt_node ─ /mission/traffic_light_state ─┐            ├─ cloud_bridge ─ /velodyne_points ─┐
          └─ hoop_trt_node ─────────── /mission/hoop_state ─────────┤            │                └ /scan            │
                                                                    │  /lowstate ─ imu_bridge ─ /imu/body ──────────┤
                                                                    │                                               ▼
                                                                    │                                   fastlio_mapping ─ /Odometry
                                                                    │                                               │
                                                                    │                                    odom_tf ─ TF odom→base_link
                                                                    ▼                                               │
                                                        day1_runner_node  ◄── /lidar_points (출발할 때 벽 정합) ──────┘
                                                          │ nav_msgs/Path (0.2초마다, 6 m 앞까지)
                                                          ▼
                                                 controller_server (Nav2 RPP) ─ /cmd_vel
                                                          ▼
                                                 cmd_vel_to_sport ─ /api/sport/request ─ Go2 Sport API
```

- NUC가 위치추정, 경로 추종, 판단을 모두 한다. 내부 PC는 카메라와 YOLO만 돌린다.
- 두 기기는 ROS 2 배포판이 다르다(Foxy / Jazzy). 그래서 서로 주고받는 메시지는 `std_msgs`(Bool, String JSON)만 쓴다.
- DDS는 CycloneDDS다. 설정은 `config/cyclonedds_go2.xml`이고, 실기는 ROS 도메인 0을 쓴다. 시험은 별도 도메인(81~89)에서 루프백으로만 돌린다.

## 2. 좌표계

| 이름 | 원점·축 | 누가 만드나 |
|---|---|---|
| `odom` | FAST-LIO를 켠 순간의 로봇 위치 | FAST-LIO → `odom_tf` (TF odom→base_link) |
| track (트랙 좌표) | 경기장 중심, x = 긴 축, 반시계 방향 주행 | 출발할 때 `wall_align`이 9 × 7 m 벽에 맞춰 한 번 잡음 |
| station `s` | 경로를 따라 잰 거리(m) | `track_path.PinLoop.station()` |

경로 파일과 구간 파일은 모두 트랙 좌표로 적는다. 주행기는 출발할 때 `T_track_odom`을 한 번 정하고, 경로를 odom 좌표로 바꿔 Nav2에 보낸다. 경기장은 180° 돌려도 같은 모양이라, 벽만으로는 두 가지 답(쌍둥이)이 나온다. 출발 표시 근처에서는 경로 출발점에 가까운 쪽을 고른다.

## 3. 프로세스

### NUC (`scripts/day1.sh up`이 띄움)

| 프로세스 | 패키지 | 하는 일 |
|---|---|---|
| `hesai_ros_driver_node` | `src/core/hesai_ros_driver` | XT16 점구름 `/lidar_points` 20 Hz (`day1.sh xt16`) |
| `imu_bridge_node` | `src/core/go2_bridge` | `/lowstate` 500 Hz → `/imu/body` |
| `cloud_bridge_node` | `src/core/go2_bridge` | `/lidar_points` → 시각을 맞춘 `/velodyne_points`(FAST-LIO 입력), 2D 스캔 `/scan`(중력 기준, 360칸. 완만한 경사는 바닥으로, 급한 단차만 장애물로) |
| `fastlio_mapping` | `src/localization/fast_lio` | 라이다 + IMU 오도메트리 `/Odometry`. 설정 `config/fastlio/fastlio_xt16_odometry.yaml` |
| `odom_tf_node` | `src/core/go2_bridge` | `/Odometry` → TF odom→base_link |
| `cmd_vel_to_sport_node` | `src/core/go2_bridge` | `/cmd_vel` → Sport API. 속도 상한·하한, 직진 쏠림 보정. 주행기가 켜고 끔(lifecycle) |
| `controller_server` | Nav2 | Regulated Pure Pursuit. 설정 `config/day1_nav2.yaml` |
| `day1_runner_node.py` | `src/mission/track_day1` | 주행기. 사용자가 터미널에서 직접 실행 |

### 내부 PC (`~/junsu_vision`, `jetson_vision.sh`가 띄움)

| 프로세스 | 파일 | 하는 일 |
|---|---|---|
| `realsense2_camera` | (설치본) | D435i 컬러 영상 |
| `traffic_light_trt_node.py` | `src/vision/start_light` | 신호등 YOLO. 모델 `tl_cone_1104`, TensorRT FP16 (INT8은 초록 재현율이 떨어져 예비). `/mission/camera_active`가 True일 때만 판단 |
| `hoop_trt_node.py` | `src/vision/start_light` | 훌라후프 YOLO. 팀 6클래스 모델(tl6_1119 계열, 빨강 = 2, 노랑 = 3: `hoop_classes.txt`), TensorRT FP16. `/mission/hoop_active`가 True일 때만 판단 |
| `yolo_view_node.py` | `src/vision/yolo_test` | 시험용 화면. 주행 노드와 공유하는 것 없음 (`~/yolo_test`) |

NUC에서 `src/vision/start_light/deploy_onboard_vision.sh push`가 내부 PC `~/junsu_vision`으로 파일을 복사한다. 훌라후프 모델을 바꿀 때는 `day1.sh hoop use <model.pt|onnx> red=<id> yellow=<id>`(ONNX 384×640으로 변환, 클래스 번호 기록) 뒤 `scripts/hoop_update.sh`를 쓴다. TensorRT 엔진은 기기마다 새로 만들어야 해서 내부 PC에서 빌드한다(`trt_build.py`, 로봇을 엎드려 둔 채 약 10분).

## 4. 토픽

| 토픽 | 형식 | 보내는 쪽 → 받는 쪽 |
|---|---|---|
| `/lidar_points` | PointCloud2 | Hesai 드라이버 → cloud_bridge, 주행기(벽 정합) |
| `/velodyne_points` | PointCloud2 | cloud_bridge → FAST-LIO |
| `/scan` | LaserScan | cloud_bridge → 콘 회피 주행기 (본 주행기는 안 씀) |
| `/imu/body` | Imu | imu_bridge → FAST-LIO |
| `/Odometry` | Odometry | FAST-LIO → odom_tf |
| `/track/path` | Path | 주행기 → (RViz 확인용) |
| `/track/status` | String | 주행기 → (상태 표시) |
| `/cmd_vel` | Twist | RPP / 주행기(제자리 회전·정지) → 모터 브리지 |
| `/api/sport/request`, `/api/sport/response` | unitree_api | 모터 브리지·주행기 ↔ Go2 (걸음새, StopMove) |
| `/mission/camera_active` | Bool | 주행기 → 신호등 노드 (1초마다 다시 보냄) |
| `/mission/traffic_light_state` | String JSON | 신호등 노드 → 주행기 |
| `/mission/hoop_active` | Bool | 주행기 → 훌라후프 노드 |
| `/mission/hoop_state` | String JSON `{"red": {"bearing_deg": 왼쪽 +, "conf": ..}, "yellow": {...}}` | 훌라후프 노드 → 주행기 |

## 5. `src/` 패키지

```
src/
├── core/
│   ├── go2_bridge/          C++ · imu_bridge, cloud_bridge, odom_tf, cmd_vel_to_sport, odom_nav
│   │   └── config/bridge_params.yaml   라이다 장착값, 속도 상한(max_vx 0.8), 쏠림 보정(0.0156·vx + 0.0397)
│   └── hesai_ros_driver/    XT16 드라이버 (외부 코드)
├── interfaces/              unitree_api, unitree_go 메시지
├── localization/
│   ├── fast_lio/            FAST-LIO (외부 코드, 팀 설정)
│   ├── go2_localization/    AMCL 설정 (이번 대회 주행에는 안 씀)
│   ├── livox_ros_driver2/   FAST-LIO 빌드 의존성
│   └── start_align/         지도 기반 출발 정렬 (예전 방식, 안 씀)
├── mission/track_day1/      ★ 주행기와 경로 계산 (아래)
├── navigation/ego_planner/  A* + 반발 포텐셜 B-스플라인 지역 플래너 (이번 대회 주행에는 안 씀)
└── vision/
    ├── start_light/         신호등·훌라후프 TensorRT 노드, 엔진 빌드·평가, 내부 PC 배포
    └── yolo_test/           카메라 + YOLO 상자 실시간 확인 노드
```

### `src/mission/track_day1/`

주행기는 ROS 노드와 ROS를 모르는 계산 모듈로 나뉜다. 계산 모듈은 녹화 재생이나 오프라인 시뮬에서 그대로 시험할 수 있다.

| 파일 | 종류 | 내용 |
|---|---|---|
| `day1_runner_node.py` | 노드 | **본 주행기**. 출발 위치 잡기 → 신호등 → 경로 추종 → 훌라후프 → 끝 정지 |
| `practice_runner_node.py` | 노드 | 연습용. 경로 위 아무 데서나 출발해 출발 표시에서 멈춤 (본 주행기를 상속) |
| `cone_runner_node.py` | 노드 | 예비. 콘 구간에서 `/scan`으로 옮겨진 콘을 비켜 감 (본 주행기를 상속) |
| `start_align_node.py` | 노드 | 출발 정렬. 경로 방향으로 돌고 옆걸음으로 경로 위에 올라선 뒤 멈춤 |
| `track_keep_node.py` | 노드 | 출발 표시에서 잡은 트랙 위치를 저장해 두고 계속 표시 (`logs/day1/track_keep.json`) |
| `day1_teach_node.py` | 노드 | 리모컨으로 한 바퀴 몰며 경로 기록 (모터 명령 없음) |
| `wall_align.py` | 계산 | 서 있는 동안의 XT16 몇 프레임 → 직사각형 벽에 맞춘 로봇 자세 |
| `track_path.py` | 계산 | 핀 경로(`PinLoop`, Catmull-Rom), station 계산, 출발 잠금(`StartLock`), 레인 계획(`LanePlan`), 진행도(`Progress`) |
| `track_spec.py` | 상수 | 규정집 트랙 치수 |
| `day1_logic.py` | 계산 | 신호등 판정(`LightGate`: 초록 0.5초 연속, 최근 1초 빨강 없음), 시뮬용 RPP |
| `cone_avoid.py` | 계산 | 콘 회피 경로 계산 (`cone_runner_node`가 씀) |
| `lane_path.py`, `band_vision_*.py` | 계산·노드 | 카메라로 바닥 띠를 보는 예전 방식 (예비, 안 씀) |
| `sim/` | 시험 | 가짜 로봇·가짜 세계 노드, 오프라인 시뮬 |
| `tools/` | 도구 | 회차별 기록 표(`bench_table.py`), 주행 계획 그림, 오도 분석 |

## 6. 주행기 상태

`day1_runner_node.py`는 0.05초마다 상태에 따라 한 가지 일만 한다.

| 상태 | 하는 일 | 다음 |
|---|---|---|
| `GAIT` | BalanceStand → 0.5초 → ClassicWalk 요청. 1.5초 안에 응답이 없으면 다시, 3번 거부되면 STOP | LOCK |
| `LOCK` | 서 있는 동안 벽 정합 5번이 일치하면 트랙 위치 확정. 경로에서 옆 0.35 m, 방향 15° 넘게 벗어나면 거부 | WAIT |
| `WAIT` | 신호등 초록 대기. `debug=true`면 `g` 키로 출발 가능 | RUN |
| `RUN` | 0.2초마다 경로를 6 m 앞까지 RPP에 보냄. 구간에 들어가면 속도·앞보기 거리 변경. 훌라후프 보는 자리 1.5 m 전부터는 걸으면서 비전 판단, 답이 나오면 경로를 바꾸고 그대로 RUN | LOOK / FINISH |
| `LEGS` | 구간이 `mode: legs`이면 RPP 없이 모서리에서 모서리로 직접 걸음. 지금 경로의 지그재그는 `mode: rpp`라 쓰지 않음 | RUN |
| `LOOK` | 걸으면서 답을 못 얻었을 때(또는 `look_mode=stop`): 갈래가 갈리는 곳에서 멈추고 177°로 돈 뒤 비전에 물어 갈래를 고름 | REGAIT |
| `REGAIT` | 서 있는 동안 걸음새가 풀렸을 수 있어 BalanceStand → 0.5초 → ClassicWalk 다시 보냄 | RUN |
| `FINISH` | 경로 끝 0.12 m 안이나 2.5초 뒤 주행기가 직접 정지 (목표 취소, 정지 명령, StopMove, 모터 브리지 끔) | DONE |
| `STOP` | 비상 정지 (스페이스·x·q) | |

### 구간 속도

`routes/<경로>.zones.json`의 상자(트랙 좌표) 안에 들어가면 그 구간 값으로 바꾼다. 상자를 사방으로 0.3 m 넓혀서 보므로, 상자보다 0.3 m 먼저 들어가고 0.3 m 늦게 나온다. 속도는 `min(주행 v, 구간 v)`다. 구간을 나오면 주행 `v`와 `lookahead`로 돌아간다.

| 구간 | 상자 (x0, y0, x1, y1) | 속도 | 앞보기 | 제자리 돌기 |
|---|---|---|---|---|
| `zigzag` | −3.8, −2.55, −1.3, −0.15 | 0.45 | 0.3 m | 35° 넘는 모서리 |
| `cones` | −0.8, −3.15, 3.75, 0.1 | 0.6 | 0.5 m | 안 함 |

이번 주행만 바꾸려면 `zigzag_v=`, `cones_v=`를 준다.

### 훌라후프 갈래 판단

1. 구간 파일의 `type: look` 항목: 보는 자리 (2.69, 1.78), 바라볼 방향 176.8°, 갈래별 훌라후프 좌표(lap1 (0.15, 1.635), lap2 (0.156, 2.386)).
2. 비전은 지정 색 상자 가운데의 화면 위치를 각도로 바꿔 보낸다: `bearing = atan2(cx − u, fx)`.
3. 주행기는 로봇 방향을 더해 경기장 기준 방향으로 바꾸고, 최근 5개를 평균한다.
4. 로봇에서 각 훌라후프 좌표까지의 방향과 비교해 가장 가까운 갈래를 고른다(`look_decide`).
   - 걸으면서 볼 때(`look_mode=move`, 기본): 보는 자리 1.5 m 전부터 비전을 켠다. 프레임마다 그 프레임 때의 자세로 갈래를 따로 판단하고, 3/3, 4/4, 최근 5개 중 4개가 같으면 정한다. 경로는 두 갈래가 0.08 m 넘게 벌어지는 곳에서 0.3 m 앞(`fork_after`, rec105326은 보는 자리 0.25 m 뒤)까지만 보낸다. 거기까지 답이 없으면 멈추고 아래 서서 보는 방식으로 넘어간다.
   - 서서 볼 때(`look_mode=stop` 또는 위에서 넘어옴): 177°로 돈 뒤 한 자세에서 최근 5개 방향(3개 이상)을 평균해 고른다.
5. 고른 갈래의 경로 파일(`rec105326_lap1.csv` / `_lap2.csv`)로 바꿔 남은 경로를 다시 만든다(`look_switch`).
6. 10초 안에 답이 없으면 무작위로 고른다. `hoop=in|out`을 주면 보지 않고 처음부터 그 갈래로 간다.

## 7. 데이터 파일

### 경로 `routes/<이름>.csv`

```
lane,station_m,x_m,y_m,yaw_deg
1,0.000,-0.4830,-2.0920,-3.2
1,0.360,-0.1238,-2.1119,-3.2
```
- 핀(트랙 좌표) 목록이다. 주행기는 핀을 Catmull-Rom으로 이어 0.1 m 간격 경로로 만든다.
- 미션 코스 기본 경로는 `rec105326_lap1`(안쪽 갈래)과 `rec105326_lap2`(바깥쪽 갈래)다. 훌라후프 앞까지는 두 경로가 같다.
- `rec1120p1~3_*`은 11-20 콘 배치에서 녹화한 통과 1·2·3번으로 만든 변형이다(`try_route.sh`).
- 실행할 때는 `logs/day1/routes/`의 작업본을 먼저 쓰고, 없으면 `routes/`를 쓴다. 편집기는 작업본에 저장하고 옛 파일을 `.bak_HHMMSS`로 남긴다.

### 구간 `routes/<이름>.zones.json`, 콘 `routes/<이름>.cones.json`

- `zones.json`: 위 표의 구간 상자와 훌라후프 보는 자리.
- `cones.json`: 콘 회피 주행기가 쓰는 콘 구간 상자.

### 지도 `maps/coshow_1119/`

| 파일 | 내용 |
|---|---|
| `map.pgm`, `map.yaml` | 2D 점유 지도 |
| `map_clean.npz` | 경로 편집기·콘 도구가 쓰는 지도 |
| `map_top.png` | 위에서 본 그림 |
| `report.txt` | 지도를 만든 녹화와 정렬 결과 |
| `routes/` | 콘 구간 바꿔 끼우기(`cone_splice`) 결과 |
| `map_track.npz`, `*.pcd` | 3D 지도. 용량(35 MB 안팎) 때문에 git에 넣지 않았다. `cone_splice`의 3D 지도 교체 단계에만 필요하고, 없으면 `scripts/map_from_bag.sh`로 다시 만든다 |

### 기록 `logs/day1/` (git에 넣지 않음)

| 파일 | 내용 |
|---|---|
| `run_HHMMSS.log` | 주행기 로그 (판단 근거가 모두 찍힘) |
| `plan_HHMMSS.png` | 그 회차 출발 위치와 경로 그림 |
| `bench.csv` | 끝까지 간 회차마다 한 줄 (걸음새, v, 랩타임, 옆 오차, 흔들림) |
| `*.log`, `*.pid` | `day1.sh up`이 띄운 프로세스의 로그와 pid |
| `routes/` | 경로 작업본 |

## 8. 설정 파일 `config/`

| 파일 | 주요 값 |
|---|---|
| `cyclonedds_go2.xml` | 인터페이스 `robot-br`. 멀티캐스트는 디스커버리(spdp)만 쓴다. 데이터까지 멀티캐스트하면 점구름이 100 Mb/s 포트를 넘쳐 XT16이 7.6 Hz로 떨어진다 |
| `day1_nav2.yaml` | RPP: 20 Hz, 앞보기 0.8 m(0.6~1.2), 목표 허용 0.10 m. 속도는 주행기가 `v`로 덮어씀 |
| `fastlio/fastlio_xt16_odometry.yaml` | `max_range` 15 m(먼 관중·구조물 제외), `det_range` 30, `blind` 0.5, 입력 `/velodyne_points`, `/imu/body` |

## 9. `scripts/`

| 묶음 | 파일 |
|---|---|
| 실행 | `setup_all.sh`(한 번에 준비), `day1.sh`(xt16·light·up·down·run·status·edit·sync·bench·selftest), `kill_all.sh` |
| 출발·연습 | `day1_align.sh`, `track_keep.sh`, `practice_run.sh`, `look_test.sh`, `try_route.sh` |
| 비전 점검 | `hoop_which.*`(지금 어느 갈래로 갈지), `hoop_check.*`, `hoop_update.sh`, `hoop_model.py`, `yolo_view.*` |
| 경로 | `route_edit.py`(지도 위 핀 편집), `route_sync.py`(lap1↔lap2 공통 구간), `route_look.py`(보는 자리), `route_from_bag.py`, `route_merge.py`, `route_corridor.py`, `route_cones.py` |
| 지도·콘 | `map_build.py`, `map_from_bag.sh`, `map_view.py`, `view_map.sh`, `cone_splice.*`(콘 구간만 바꿔 끼우기), `passes_view.sh`, `cones_view.py`, `day1_cones.sh` |
| 기록 분석 | `zone_times.py`(구간별 통과 시간), `go2_yaw_drift.py`(직진 쏠림 측정) |

각 명령의 인자는 [RUN.md](RUN.md)의 표에 있다.

## 10. 시험 `tests/`

모두 루프백과 별도 ROS 도메인에서만 돌아서 로봇에 닿지 않는다.

| 파일 | 내용 |
|---|---|
| `selftest.sh` | 가짜 로봇으로 주행 6가지를 동시에 돌리고 통과/실패 표 (`day1.sh selftest`, 약 3분) |
| `day1_ros_integration.sh` | 실제 odom TF + 모터 브리지 + RPP + 주행기, 가짜 Go2·벽·신호등 (도메인 81) |
| `fastlio_replay.sh` | 녹화에서 FAST-LIO만 돌려 `/Odometry`를 CSV로 |
| `cones/` | 콘 회피 녹화 재생 시험 |
| `ego/` | ego_planner 시험 (안 씀) |
