# 실행 방법

모든 명령은 NUC에서 실행한다. 로봇을 움직이는 명령은 운영자가 자기 터미널에서 직접 실행하고, 스페이스나 `x`로 언제든 멈출 수 있게 한다. 코드가 어떻게 이어지는지는 [STRUCTURE.md](STRUCTURE.md)에 있다.

## 0. 처음 한 번

### 빌드

```bash
cd ~/ws/ROLAB-COSHOW-2026
source /opt/ros/jazzy/setup.bash
source ~/unitree_ros2/cyclonedds_ws/install/setup.bash
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

- 빌드하는 것은 C++ 패키지(go2_bridge, FAST-LIO, Hesai 드라이버, 메시지)다. 주행기 같은 파이썬 노드는 스크립트가 `src/`에서 바로 실행하므로 빌드할 필요가 없다.
- 필요한 것: Ubuntu 24.04, ROS 2 Jazzy, Nav2(`nav2_controller`, `nav2_regulated_pure_pursuit_controller`, `nav2_lifecycle_manager`), CycloneDDS(`rmw_cyclonedds_cpp`), Unitree ROS 2 메시지(`~/unitree_ros2`).
- 파이썬: numpy, scipy, OpenCV, `rosbag2_py`(녹화 도구).
- 커널 수신 버퍼를 늘린다. 기본값이면 XT16 점구름이 30% 가까이 버려진다.
  ```bash
  sudo sysctl -w net.core.rmem_max=134217728 net.core.wmem_max=134217728
  ```

### 내부 PC 배포

```bash
src/vision/start_light/deploy_onboard_vision.sh push     # 노드·모델을 내부 PC ~/junsu_vision 으로
scripts/day1.sh hoop build                               # 훌라후프 TensorRT 엔진 빌드 (로봇 엎드려, ~10분)
```
새 훌라후프 모델로 바꿀 때는 `scripts/hoop_update.sh`를 쓴다. 예전 엔진을 백업하고, 올리고, 빌드하고, 노드를 다시 띄운다. `hoop_update.sh rollback`은 예전 엔진으로 되돌린다.

### 별칭

`~/.bash_aliases`에 `d`로 시작하는 별칭이 있다. 새 PC라면 [부록](#부록-별칭-정의)의 정의를 붙여 넣는다. 목록은 `dhelp`로 본다. 별칭 뒤에 인자를 더 붙일 수 있다(예: `dred debug=true`). 아래 표는 별칭과 실제 명령을 같이 적었다. 실제 명령은 저장소 루트 기준이다.

## 1. 준비 (전원 켤 때 한 번)

로봇을 엎드린 채 켜고, 리모컨으로 세운 뒤 가만히 둔다. 서 있기만 해도 뒷다리가 데워지므로 준비가 끝나면 바로 출발한다.

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `dsetup` | `scripts/setup_all.sh` | 아래 네 단계를 한 번에. 이미 켜진 것은 건너뛰고 마지막에 상태를 보여 줌 |
| `dxt` | `scripts/day1.sh xt16` | XT16 드라이버. `/lidar_points` 약 20 Hz |
| `dlight` | `scripts/day1.sh light` | 내부 PC 신호등 YOLO |
| `dhoopon` | `scripts/day1.sh hoop start` | 내부 PC 훌라후프 YOLO (로봇 재부팅 뒤) |
| `dup` | `scripts/day1.sh up` | FAST-LIO, 모터 브리지, RPP. 끝에 `■ up: odom lio … active`. 이때 로봇이 움직이면 안 됨(IMU 초기화) |
| `dstat` | `scripts/day1.sh status` | 토픽 주기 확인 |

`dsetup`이 끝에 `■ ready`를 띄우면 준비가 끝난 것이다. `★ ready with problems`면 적힌 부분부터 고친다.

`up`의 환경변수:

| 변수 | 기본 | 뜻 |
|---|---|---|
| `MAX_VX` | 0.8 | 모터 브리지 전진 속도 상한 |
| `TRIM` | 1 | 0이면 직진 쏠림 보정을 끔 (보정값은 ClassicWalk에서 잰 것) |
| `ODOM` | lio | `builtin` = 내장 오도(×1.189, 예비), `fake` = 시험용 |

## 2. 회차마다

1. 로봇을 출발 표시에 놓는다.
2. `dalign`으로 출발 정렬을 한다. 끝에 `aligned: lateral … heading …`이 뜬다.
3. 받은 색에 맞는 주행 명령을 실행한다.
4. 한 바퀴를 돌면 주행기가 저절로 멈추고 모터 브리지를 끈다. `q`로 주행기를 닫는다.

| 별칭 | 명령 | 언제 |
|---|---|---|
| `dalign` | `scripts/day1_align.sh route=rec105326_lap1` | 주행 전마다 |
| `dred` | `scripts/day1.sh run route=rec105326_lap1 hoop=red v=0.8` | 빨강을 받았을 때 |
| `dyellow` | `scripts/day1.sh run route=rec105326_lap1 hoop=yellow v=0.8` | 노랑을 받았을 때 |
| `din` | `scripts/day1.sh run route=rec105326_lap1 hoop=in v=0.8` | 안쪽 갈래로 정해 두고 (비전 안 봄) |
| `dout` | `scripts/day1.sh run route=rec105326_lap1 hoop=out v=0.8` | 바깥쪽 갈래로 정해 두고 |
| `dtry N` | `scripts/try_route.sh N` | 콘 구간 변형 경로 1·2·3 (기본 `hoop=in v=0.8`) |
| `dcones` | `scripts/day1_cones.sh run route=rec105326_lap1 v=0.8` | 콘이 옮겨졌을 때 예비. 콘 구간에서 옮겨진 콘을 비켜 감. 색은 `dcones hoop=red`처럼 붙임 ([CONES.md](CONES.md)) |
| – | `scripts/day1.sh run in v=0.8` / `run out v=0.8` | 1일차 트랙 경기 (끝남). 경로 파일 없이 규정 트랙 레인 중심선을 따라감. 출발 정렬 안 씀 |

출발 정렬이 필요한 이유: 출발 표시는 기록한 경로 출발점에서 왼쪽으로 약 0.2 m, 방향은 14° 틀어져 있다. 그대로 출발하면 1 m 앞 첫 콘에 닿는다. 정렬은 벽으로 위치를 찾고, 경로 방향으로 제자리에서 돈 뒤 옆걸음으로 경로 위에 올라선다. 먼저 odom TF와 모터 브리지가 5초 안에 보이는지 확인하므로, `up` 전에 실행하면 바로 알려 준다.

### 정렬 인자 (`day1_align.sh`)

| 인자 | 기본 | 뜻 |
|---|---|---|
| `route=` | rec105326_lap1 | 맞출 경로 |
| `tol_lat=` | 0.03 | 허용 옆 오차 (m) |
| `tol_head=` | 3 | 허용 방향 오차 (°) |
| `gait=` | none | 정렬 전에 걸음새를 보낼지 |

### 주행 인자 (`day1.sh run`, 모두 `key=value`)

| 인자 | 기본 | 뜻 |
|---|---|---|
| `in` / `out` | in | 1일차 트랙 레인 (in = 2레인 안쪽, out = 1레인 바깥쪽). `route=`를 주면 경로 파일이 길을 정하므로 쓰지 않음 |
| `route=` | – | 경로 이름. `logs/day1/routes/` 작업본을 먼저 찾고, 없으면 `routes/` |
| `v=` | 0.5 (별칭 0.8) | 최고 속도 (m/s). 브리지 상한 0.8을 넘으면 0.8로 낮춤. ClassicWalk 실제 속도는 0.65 안팎 |
| `hoop=` | – | `red` `yellow` = 아치 앞에서 비전으로 고름 / `in` `out` = 처음부터 그 갈래 |
| `light=` | model | `none` = 신호등 안 봄 (`debug=true`일 때만) |
| `light_timeout=` | 0 | 초. 그동안 초록이 없으면 그냥 출발. 0 = 안 씀 |
| `look_timeout=` | 10 | 아치 앞에서 비전 답을 기다리는 초. 지나면 무작위 |
| `look_mode=` | move | `move` = 보는 자리 앞에서 서지 않고 걸으면서 판단. 갈래가 갈리는 곳까지 답이 없으면 거기서 서서 `stop` 방식 / `stop` = 보는 자리에서 서서 돌고 봄 (11-20 방식) |
| `look_pre=` | 1.5 | 보는 자리 몇 m 전부터 비전을 켤지 (`look_mode=move`) |
| `debug=` | false | `true` = 연습 모드 (`g`·`1`·`2` 키, `light=none`) |
| `gait=` | classic | classic / freewalk / trotrun / static / economic |
| `lookahead=` | 0.5 | RPP 앞보기 (m). 경로 주행 기본값 |
| `min_radius=` | 0.4 | 최소 회전 반경 (m) |
| `zigzag_v=` (줄여서 `zv=`) | 0 | 이번 주행만 지그재그 구간 속도. 0 = 구간 파일 값(0.45) |
| `cones_v=` (줄여서 `cv=`) | 0 | 이번 주행만 라바콘 구간 속도. 0 = 구간 파일 값(0.6) |
| `zz` | – | 경로 이름의 `_lap`을 `zz_lap`으로 바꾼 사본으로 주행 (`rec105326_lap1` → `rec105326zz_lap1`). 사본은 `logs/day1/routes/` 작업본에만 있고 git에는 없다 |
| `max_lat=` | 0.35 | 출발 잠금 허용 옆 오차 (m) |
| `max_head=` | 15 | 출발 잠금 허용 방향 오차 (°) |
| `track_dx=` `track_dy=` `track_dyaw_deg=` | 0 | 트랙 좌표 수동 보정 |
| `extra=` | 1.2 | 경로 끝을 출발선 너머 몇 m에 둘지 (뒷발이 선을 넘도록) |

### 주행기 흐름

1. **걸음새**: BalanceStand → ClassicWalk. 응답 코드 0을 확인한다.
2. **출발 위치 잡기**: 서 있는 동안 벽 정합 5번이 맞으면 `start found: …`. 회차 그림이 `logs/day1/plan_HHMMSS.png`로 저장된다. 경로 출발점에서 1 m 넘게 떨어져 있으면 거부한다.
3. **신호등**: 초록이 0.5초 이어지고 최근 1초 안에 빨강이 없으면 출발한다.
4. **구간 속도**: 나머지는 `v`, 지그재그는 0.45 m/s(앞보기 0.3 m, 35° 넘는 모서리는 제자리 회전), 라바콘 밭은 0.6 m/s(앞보기 0.5 m).
5. **훌라후프** (`hoop=red|yellow`): 보는 자리 (2.69, 1.78) 1.5 m 전부터 걸으면서 비전을 켠다. 프레임마다 그때 자세로 갈래를 판단해 3/3, 4/4, 최근 5개 중 4개가 같으면 바로 그 갈래로 바꾸고 계속 걷는다. 두 갈래가 갈리는 곳(보는 자리 0.25 m 뒤에서 0.3 m 앞)까지 답이 없으면 거기서 멈추고 177°로 돌아 다시 본다. 10초 안에 답이 없으면 무작위로 고른다. 서서 본 경우에만 다시 걷기 전에 BalanceStand → ClassicWalk를 한 번 더 보낸다. `look_mode=stop`이면 처음부터 서서 본다.
6. **끝**: 경로 끝 0.12 m 안에 들거나 마지막 구간 2.5초 뒤, 주행기가 목표를 취소하고 정지 명령과 StopMove를 보낸 뒤 모터 브리지를 끈다. 로그에 `■ stopped at the lap end`가 뜬다.

### 대회 모드와 `debug=true`

| 기능 | 대회 모드 (기본) | `debug=true` |
|---|---|---|
| `g` 키 (신호 없이 출발) | 무시 | 됨 |
| `1` / `2` 키 (훌라후프 갈래) | 무시 | 됨 |
| `light=none` | 실행 거부 | 됨 |

시작할 때 `competition mode`와 `★ DEBUG` 중 하나가 뜬다. 출발 전에 주는 `hoop=in|out`은 두 모드 모두 된다. 규정은 수동 출발과 경기 중 무선 조작을 금지하므로, 경기에서는 `debug=`를 붙이지 않는다.

### 키

| 키 | 언제 | 동작 |
|---|---|---|
| `스페이스` / `x` | 언제나 | 비상 정지 |
| `g` | 신호등 기다리는 동안 (`debug=true`) | 바로 출발 |
| `1` / `2` | 훌라후프를 보는 동안 (`debug=true`) | 안쪽(`rec105326_lap1`) / 바깥쪽(`rec105326_lap2`) |
| `q` | 언제나 | 정지 후 주행기 종료 |

## 3. 연습 (로봇이 움직인다)

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `dkeep` | `scripts/track_keep.sh` | 출발 표시에서 트랙 위치를 잡아 두고 계속 표시. 이후 `dhere`가 어디서든 그 위치로 출발 (창 닫아도 됨) |
| `dhere` | `scripts/practice_run.sh` | 경로 위 아무 데서나 출발해 출발 표시에서 멈춤 |
| `dlook` | `scripts/look_test.sh` | 출발 표시에서 아치 보는 자리까지 가서 아치를 보고 서 있음 (훌라후프 노드 잠깐 끔) |
| `dlookhere` | `scripts/look_test.sh here` | 콘 밭 뒤에서 출발해 아치 보는 자리에 서 있음 |

- `practice_run.sh` 기본값: `route=rec105326_lap1 hoop=in v=0.8 lookahead=0.5 min_radius=0.4 max_lat=0.5 max_head=30 light=none debug=true`. 주행 인자를 모두 받는다.
- 위치 잡는 방식 `lock=auto|keep|walls` (기본 auto): 같은 FAST-LIO로 15분 안에 `dkeep`이 저장한 위치가 있으면 그것을 쓰고, 없으면 벽과 지도로 잡는다. 지도 안쪽 구조물과 스캔이 얼마나 겹치는지로 쌍둥이를 고르고, 둘 다 안 맞으면 거부한다.
- `look_test.sh` 인자: `route=` `v=0.8` `look_timeout=600`.

## 4. 비전 점검 (로봇이 안 움직인다)

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `dlc` | `scripts/day1.sh lightcheck [초]` | 실제 신호등을 주행기처럼 지켜봄 (기본 20초) |
| `dhoop` | `scripts/day1.sh hoop status` | 훌라후프 노드 상태 |
| `dwhich` | `scripts/hoop_which.sh` | 빨강·노랑이 지금 안쪽·바깥쪽 중 어디로 갈지 (주행기 판단 함수 그대로). 인자 `seconds=15 route= pose=x,y,yaw_deg` |
| `dhc` | `scripts/hoop_check.sh` | 실제 카메라로 훌라후프 비전 20초 (`seconds=`) |
| `dhce` | `scripts/hoop_check.sh engine` | 내부 PC FP16 엔진이 라벨 사진을 맞히는지 |
| `dyolo` | `scripts/yolo_view.sh` | 카메라 + YOLO 상자 실시간 창. `tl` = 신호등 모델, `<엔진 파일>`. 창 키: `c` 한 장, `r` 연속 저장, `s` 화면 저장, `q` 종료 (사진은 `logs/yolo_test/captures/`) |

## 5. 경로·지도 손보기 (로봇이 안 움직인다)

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `deditin` | `scripts/day1.sh edit rec105326_lap1` | 안쪽 경로 편집. 핀을 끌어 옮기고 `s` 저장 (옛 파일은 `.bak_HHMMSS`). 인자 `map=maps/coshow_1119` |
| `deditout` | `scripts/day1.sh edit rec105326_lap2` | 바깥쪽 경로 편집 |
| `dsyncin` | `scripts/day1.sh sync rec105326_lap1 rec105326_lap2` | lap1에서 고친 공통 구간을 lap2에 반영 |
| `dsyncout` | `scripts/day1.sh sync rec105326_lap2 rec105326_lap1` | 반대 방향 |
| `droutes` | `scripts/day1.sh routes` | 경로 목록 (작업본 → 저장소) |
| `dsplice` | `scripts/cone_splice.sh <녹화 폴더> [pass=N]` | 콘만 옮겨졌을 때. 짧은 리모컨 녹화로 경로와 지도의 콘 구간만 바꿔 끼움 (격리 재생, 5~10분) |
| `dpasses` | `scripts/passes_view.sh` | 콘 녹화의 통과별 경로를 편집기에 색깔별로 겹쳐 봄 (`r` 숨기기, `s` 저장) |
| – | `scripts/view_map.sh maps/coshow_1119` | 3D 지도 보기 |

콘 배치가 바뀌었는데 다시 녹화할 시간이 없으면, `day1.sh run` 대신 `scripts/day1_cones.sh run …`(별칭 `dcones`)을 쓴다. 인자는 같고, 콘 구간에서 옮겨진 콘을 `/scan`으로 비켜 간다. 자세한 내용은 [CONES.md](CONES.md).

## 6. 새 코스에서 경로 만들기

콘 배치만 바뀌었으면 위의 `dsplice`로 충분하다. 경기장이나 코스가 새로 바뀌었을 때의 순서다.

1. **경로 기록**: `dup` 뒤 로봇을 출발 표시에 세우고 실행한다. 리모컨으로 코스를 한 바퀴 몰면, 출발선을 지날 때 저장된다(Enter = 바로 저장). 모터 명령은 나가지 않는다. 결과는 `logs/day1/routes/<이름>.csv`다. 갈래가 둘이면 두 번 기록한다.
   ```bash
   scripts/day1.sh teach in name=<이름>_lap1
   ```
2. **지도** (같은 주행을 `/lidar_points`, `/lowstate`와 함께 녹화해 두었을 때): 녹화를 실제 속도로 재생해 FAST-LIO 지도를 만들고, 바닥을 수평으로 맞춘 뒤 벽에 맞춰 트랙 좌표로 둔다. 결과는 `maps/<이름>/`다. 느리게 재생하면 IMU 시각이 깨진다.
   ```bash
   scripts/map_from_bag.sh <녹화 폴더> <이름>
   ```
3. **두 갈래 맞추기**: 같이 가는 구간을 한 줄로 만든다.
   ```bash
   python3 scripts/route_merge.py logs/day1/routes/<이름>_lap1.csv logs/day1/routes/<이름>_lap2.csv map=maps/<이름>
   ```
4. **지그재그 구간**: 상자 안의 경로를 지도의 통로 가운데 선으로 바꾸고 구간 파일(`.zones.json`)을 쓴다.
   ```bash
   python3 scripts/route_corridor.py logs/day1/routes/<이름>_lap1.csv logs/day1/routes/<이름>_lap2.csv zone=x0,y0,x1,y1 name=zigzag
   ```
5. **훌라후프 보는 자리**: 두 갈래가 아직 같은 곳에 찍는다. 갈래별 훌라후프 좌표도 같이 기록된다.
   ```bash
   python3 scripts/route_look.py logs/day1/routes/<이름>_lap1.csv logs/day1/routes/<이름>_lap2.csv at=x,y
   ```
6. **라바콘 구간**: `.zones.json`에 `cones` 상자(속도·앞보기)를 넣는다. 콘 회피 주행기를 쓸 거면 `day1_cones.sh zone`으로 `.cones.json`도 만든다.
7. **확인**: `day1.sh edit`로 핀을 다듬고, `dself`로 가짜 로봇 점검을 한 뒤 실기로 넘어간다. 저장소에 남기려면 `logs/day1/routes/`의 파일을 `routes/`로 복사해 커밋한다(`dself`는 `routes/`의 파일로 시험한다).

녹화에서 바로 경로를 뽑는 `scripts/route_from_bag.py`(녹화 + FAST-LIO 재생 CSV + 지도 → 핀)도 있다. FAST-LIO 재생 CSV는 `tests/fastlio_replay.sh`로 만든다.

## 7. 기록 보기

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `dbench` | `scripts/day1.sh bench` | 회차별 기록 표 (걸음새, v, 랩타임, 옆 오차, 흔들림) |
| `dzones` | `python3 scripts/zone_times.py [N]` | 최근 N회(기본 6) 구간별 통과 시간 |
| `dlast` | `scripts/day1_cones.sh last` | 콘 회피 마지막 기록 그림 |
| `dself` | `scripts/day1.sh selftest` | 가짜 로봇으로 전체 점검 (약 3분, 격리 도메인) |

## 8. 끝낼 때

| 별칭 | 명령 | 하는 일 |
|---|---|---|
| `ddown` | `scripts/day1.sh down` | `up`으로 띄운 것 끄기 |
| `dkill` | `scripts/kill_all.sh` | NUC에서 주행 관련 프로세스를 강제로 모두 끔 (도메인 0만, 녹화는 그대로). 로봇에 StopMove 먼저 보냄 |
| `dkill robot` | `scripts/kill_all.sh robot` | 위 + 내부 PC 신호등·훌라후프·yolo 노드와 카메라 |

## 9. 문제가 생기면

| 증상 | 확인 |
|---|---|
| `up`이 `already running`으로 거부 | 다른 스택이 같은 도메인에 떠 있음. `dkill` 뒤 `dsetup` |
| `scans 0`에서 멈춤 (출발 위치 못 잡음) | `dxt`. 라이다 데이터가 안 옴 (`ping 192.168.123.20`) |
| `/lidar_points`가 20 Hz보다 한참 낮음 | 커널 수신 버퍼(`rmem_max`), DDS 멀티캐스트 설정 |
| `motor bridge / controller_server parameters not reachable` | `up`이 안 떠 있음 |
| `dalign`이 바로 끝남 | odom TF나 모터 브리지가 없음. `dup` 먼저 |
| `★ 신호등 메시지 없음` | `dlight`. `dlc`로 로봇 안 움직이고 확인 |
| 훌라후프에서 늘 무작위로 고름 | `dhoop`으로 노드 확인, `dwhich`로 실제 판단 확인 |
| `robot is … from where the route was taught to start` | 출발 표시가 다름 |
| 터미널이 입력을 안 보여 줌 | `reset` |

## 10. 브랜치와 태그

| | `main` (최종) | `coshow-dev` / 태그 `run-good-1120` (11-20) |
|---|---|---|
| 구간 기본 속도 | 콘 0.6, 지그재그 0.45 | 콘 0.6, 지그재그 0.4 |
| 훌라후프 | `look_mode=move`: 걸으면서 보고 바로 바꿈. 갈래가 갈리는 곳까지 답이 없으면 서서 봄 | 보는 자리에서 서서 돌고 봄 |
| 줄임 인자 | `cv=` `zv=` `zz` | 없음 |

main에서 11-20처럼 돌리려면 `look_mode=stop zv=0.4`를 붙인다. 11-20 코드를 그대로 쓰려면 `git checkout run-good-1120`.

11-21까지는 두 폴더(`~/ws/ROLAB-COSHOW-2026` = coshow-dev, `~/ws/ROLAB-COSHOW-next` = coshow-next)에 받아 `d…` / `e…` 별칭으로 나눠 돌렸다. `e…` 별칭은 아래 `d…` 정의에서 `R`만 next 폴더로 바꾼 것이다. 두 쪽은 같은 로봇 노드를 띄우므로 한쪽만 켜 두고, 바꿀 때는 먼저 `dkill`을 한다.

## 부록: 별칭 정의

`~/.bashrc` 또는 `~/.bash_aliases`에 붙여 넣는다. `R`은 저장소 위치다.

```bash
R=~/ws/ROLAB-COSHOW-2026
alias dsetup="$R/scripts/setup_all.sh"
alias dxt="$R/scripts/day1.sh xt16"
alias dlight="$R/scripts/day1.sh light"
alias dup="$R/scripts/day1.sh up"
alias ddown="$R/scripts/day1.sh down"
alias dkill="$R/scripts/kill_all.sh"
alias dstat="$R/scripts/day1.sh status"
alias dlc="$R/scripts/day1.sh lightcheck"
alias dhoop="$R/scripts/day1.sh hoop status"
alias dhoopon="$R/scripts/day1.sh hoop start"
alias dhoopup="$R/scripts/hoop_update.sh"
alias dwhich="$R/scripts/hoop_which.sh"
alias dhc="$R/scripts/hoop_check.sh"
alias dhce="$R/scripts/hoop_check.sh engine"
alias dyolo="$R/scripts/yolo_view.sh"
alias dkeep="$R/scripts/track_keep.sh"
alias dhere="$R/scripts/practice_run.sh"
alias dlook="$R/scripts/look_test.sh"
alias dlookhere="$R/scripts/look_test.sh here"
alias dalign="$R/scripts/day1_align.sh route=rec105326_lap1"
alias dred="$R/scripts/day1.sh run route=rec105326_lap1 hoop=red v=0.8"
alias dyellow="$R/scripts/day1.sh run route=rec105326_lap1 hoop=yellow v=0.8"
alias din="$R/scripts/day1.sh run route=rec105326_lap1 hoop=in v=0.8"
alias dout="$R/scripts/day1.sh run route=rec105326_lap1 hoop=out v=0.8"
alias dtry="$R/scripts/try_route.sh"
alias dcones="$R/scripts/day1_cones.sh run route=rec105326_lap1 v=0.8"
alias dlast="$R/scripts/day1_cones.sh last"
alias dsplice="$R/scripts/cone_splice.sh"
alias dpasses="$R/scripts/passes_view.sh"
alias deditin="$R/scripts/day1.sh edit rec105326_lap1"
alias deditout="$R/scripts/day1.sh edit rec105326_lap2"
alias dsyncin="$R/scripts/day1.sh sync rec105326_lap1 rec105326_lap2"
alias dsyncout="$R/scripts/day1.sh sync rec105326_lap2 rec105326_lap1"
alias droutes="$R/scripts/day1.sh routes"
alias dzones="python3 $R/scripts/zone_times.py"
alias dbench="$R/scripts/day1.sh bench"
alias dself="$R/scripts/day1.sh selftest"
alias dhelp='alias | grep "^alias d"'
```
