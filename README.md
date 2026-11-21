# ROLAB-COSHOW-2026

Unitree Go2 EDU 사족보행 로봇의 자율주행 스택(ROS 2)입니다. 2026 CO-SHOW 4족보행 로봇경진대회 자율주행 부문에 참가했습니다.

실행 방법은 [docs/RUN.md](docs/RUN.md)(빌드, 준비, 회차마다, 연습, 점검, 경로 손보기, 문제 해결), 코드 구조는 [docs/STRUCTURE.md](docs/STRUCTURE.md)(프로세스, 토픽, 패키지, 주행기 상태, 데이터 파일)에 있습니다.

## 대회

- 대회: 2026 CO-SHOW 4족보행 로봇경진대회 (공유대학 SHARE 주관), 자율주행 부문
- 1일차 트랙 기록 경기(30%): 9 × 7 m 트랙, 900 mm 레인을 한 바퀴 돕니다. 인레인과 아웃레인 최단 기록을 합산합니다.
- 3일차 미션 경기(70%): 라바콘, 색상 아치(훌라후프), 경사로, 타이어, 700 mm 협소 통로를 지나 완주합니다.
- 연산은 로봇 위에서만 합니다. 경기 중 무선 조종은 할 수 없고, 출발은 신호등 녹색을 스스로 인식해야 합니다.
- 사전에 입력한 경로를 센서 피드백으로 따라가는 방식이 허용됩니다(규정 4.4조).

## 하드웨어

| 구성 | 모델 | 역할 |
|---|---|---|
| 로봇 | Unitree Go2 EDU | 보행, 몸통 IMU 500 Hz |
| 라이다 | Hesai XT16 (16채널, 20 Hz) | FAST-LIO 오도메트리, 벽 정합, 세로 스캔 |
| 카메라 | Intel RealSense D435i | 신호등, 훌라후프 색 인식 |
| 주 연산 | Intel NUC (Core Ultra 7, Ubuntu 24.04, ROS 2 Jazzy) | 위치추정, 경로 추종, 주행 판단 |
| 내부 PC | Jetson Orin NX (JetPack 5, ROS 2 Foxy) | 카메라, YOLO 추론 (TensorRT FP16) |

## 주행 방식

1. **위치추정**: FAST-LIO가 odom → base_link를 냅니다. 출발할 때 라이다 스캔을 9 × 7 m 경기장 벽에 맞춰 트랙 좌표를 한 번 잡습니다(`wall_align`).
2. **출발 정렬**: `start_align_node`가 경로 방향으로 제자리에서 돌고, 옆걸음으로 경로 위에 올라선 뒤 멈춥니다.
3. **출발**: 내부 PC의 YOLO 신호등 노드가 녹색을 보면 출발합니다.
4. **경로 추종**: 리모컨으로 한 번 몰아 기록한 경로(`routes/*.csv`)를 Nav2 Regulated Pure Pursuit로 따라갑니다. 0.2초마다 6 m 앞까지 보냅니다.
5. **구간 속도**: `routes/<경로>.zones.json`에 구간마다 속도와 앞보기 거리를 적습니다. 지그재그와 라바콘 구간은 느리게 갑니다.
6. **색상 아치**: 보는 자리에서 지정 색 훌라후프의 방향을 YOLO로 잽니다. 그 방향에 있는 훌라후프로 가는 갈래(lap1 안쪽 / lap2 바깥쪽)로 경로를 바꿉니다.
7. **끝**: 한 바퀴를 마치면 주행기가 직접 목표를 취소하고 정지 명령을 보낸 뒤 모터 브리지를 끕니다.

## 디렉토리

```
ROLAB-COSHOW-2026/
├── config/            CycloneDDS, Nav2(RPP), FAST-LIO 설정
├── docs/              실행 순서(RUN), 콘 회피(CONES), 경기 설계, 규정집
├── maps/coshow_1119/  경기장 지도 (2D·3D, 경로 편집용 그림)
├── routes/            기록 경로(lap1·lap2)와 구간 파일(.zones.json, .cones.json)
├── scripts/           실행 스크립트 (day1.sh, setup_all.sh, day1_align.sh, practice_run.sh …)
│                      경로·지도 도구 (route_edit, route_sync, cone_splice, map_build …)
├── src/
│   ├── core/          go2_bridge (Sport API, 직진 쏠림 보정), Hesai 드라이버
│   ├── interfaces/    unitree_api, unitree_go 메시지
│   ├── localization/  FAST-LIO, go2_localization, start_align
│   ├── mission/track_day1/  주행기 (day1_runner_node, 연습·콘 회피 주행기, 출발 정렬, 경로 계산)
│   ├── navigation/    ego_planner (이번 대회 주행에는 쓰지 않음)
│   └── vision/        start_light (신호등·훌라후프 TensorRT 노드, 내부 PC 배포), yolo_test
└── tests/             가짜 로봇 통합 시험(selftest), 녹화 재생 시험
```

## 문서

| 문서 | 내용 |
|---|---|
| [docs/RUN.md](docs/RUN.md) | 빌드, 실행 명령과 인자 전체, 주행기 흐름, 키, 문제 해결 |
| [docs/STRUCTURE.md](docs/STRUCTURE.md) | 프로세스·토픽·좌표계, 패키지와 파일, 주행기 상태, 경로·구간·지도 파일 형식 |
| [docs/CONES.md](docs/CONES.md) | 콘 배치가 바뀌었을 때 쓰는 콘 회피 주행기 |
| [docs/01_1일차_트랙레이스.md](docs/01_1일차_트랙레이스.md) | 1일차 트랙 경기 분석과 설계 |
| [docs/02_3일차_미션수행.md](docs/02_3일차_미션수행.md) | 3일차 미션 경기 분석과 설계 |
| [docs/rulebook](docs/rulebook) | 공식 규정집 |

## 브랜치

- `main`: 대회에서 쓰는 코드입니다. 태그 `run-good-1120`이 11-20 잘 되던 상태입니다.
- `coshow-next`: 기록 단축용 변경을 시험하는 브랜치입니다. main에 합치지 않습니다.

## 라이선스

외부 코드의 라이선스는 [src/EXTERNAL_LICENSES.md](src/EXTERNAL_LICENSES.md)와 각 모듈의 LICENSE 파일을 따릅니다(FAST-LIO GPL-2.0, Hesai ROS Driver 등).
