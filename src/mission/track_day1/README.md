# track_day1

Day 1 track race: one lap of the assigned lane. No map, no camera: FAST-LIO odometry, the start pose from the 0.6 m arena wall, and the rulebook lane geometry.

```
XT16 /lidar_points ── standing: wall_align (9.00 x 7.00 m rectangle) ──▶ robot pose in the track frame
                                                                          │ + odom pose at that moment
FAST-LIO /Odometry ── odom_tf ──▶ TF odom → base_link                    ▼ track ← odom (fixed for the run)
                                           day1_runner_node: rulebook lane centre (track_path) → odom frame
                                                   │ nav_msgs/Path, re-sent every 0.2 s from the robot onwards
                                                   ▼
                                  Nav2 controller_server (RPP) ── /cmd_vel ──▶ go2_bridge (yaw trim, ClassicWalk)
YOLO traffic light ── /mission/traffic_light_state ──▶ GREEN 0.5 s, no RED within 1 s
```

The arena wall and the track are both symmetric under a 180° turn. Which of the two wall fits is picked does not matter: either gives the same path relative to the robot.

## Files
| File | |
|---|---|
| `track_spec.py` | Rulebook track (lane 900 mm, R1185–R3005, straight 2000, counter-clockwise) and the wall |
| `wall_align.py` | Robot pose in the track frame from one XT16 frame (numpy only) |
| `track_path.py` | Lane centre by station, start lock (steady wall poses, lane and placement checks), lap progress |
| `day1_logic.py` | Start light gate, lap end, RPP core for the offline sim |
| `day1_runner_node.py` | NUC runner (Jazzy), `mode` `wall` (default) or `vision` |
| `tools/` | On recordings: `wall_align_bag.py` (wall poses), `odom_vs_wall.py` (any odometry vs the wall poses), `odom_decompose.py` (built-in odometry error by part) |
| `band_vision_*`, `lane_path.py`, `jetson_track.sh` | Spare: lane from the D435 (`mode:=vision`, `ODOM=builtin`) |
| `sim/` | Fake robot for the ROS test (walls + FAST-LIO pose + another robot, or camera), synthetic floor for the vision spare |

Config: `config/day1_nav2.yaml` (controller_server + RPP), `config/fastlio/fastlio_xt16_odometry.yaml`. Launcher: `scripts/day1.sh`.

## Usage
```bash
scripts/day1.sh light                # YOLO traffic-light node (internal PC)
scripts/day1.sh up                   # robot standing still: imu/cloud bridge, FAST-LIO, odom TF, motor bridge, controller_server
scripts/day1.sh run lane=2 v=0.5     # robot on its start mark; space / x = STOP
scripts/day1.sh down
```
Runner parameters: `lane` (1 outer, 2 inner), `v` (m/s), `extra` (m past one lap, default 1.2), `green_s`, `light` (`model`, `none` for tests only), `gait`, `max_lat` (0.35 m), `max_head` (15°), `lock_n` (5 scans).

## On site
1. `up` refuses if bridges, FAST-LIO or a controller from another stack run on the same ROS domain. Stop those first. FAST-LIO initialises its IMU at start-up, so the robot must stand still.
2. `run` logs `start found: lane …, lateral …, heading …` within about 1 s of ClassicWalk. If it says `robot is in lane X, not lane Y` or `placement: …`, re-place the robot. The run does not start until the start is found.
3. `light`: the model was trained on the RoboWorld light. Check it sees the CO SHOW light before the first run.
4. Lane 1 needs ≥ 0.36 m/s average to finish inside 60 s. `v=0.5` gives ~44 s on lane 1 and ~33 s on lane 2. Raise it only after a clean run.
5. The runner stops 1.2 m after the lap. Timing ends when the rear feet cross the line, so the overshoot costs nothing.

## Tests (no robot)
```bash
tests/day1_ros_integration.sh lane=1 off0=-0.25 yaw0=-10        # real Nav2 + bridge + runner, fake robot (domain 81, loopback)
tests/fastlio_replay.sh <bag> out.csv domain=82                 # FAST-LIO on a recording (loopback)
python3 tools/odom_vs_wall.py <bag> traj=out.csv scales=1.0     # its error against the wall poses
```

| Check (recordings of 11-18, arena) | Result |
|---|---|
| Wall pose, 6 laps (3,600 frames) | passes 99.8 %, rms 0.9–1.3 cm |
| FAST-LIO, anchored once at the start, 6 recordings of 1–3 laps | lateral error max 2.9–8.0 cm (median position error 1.7–4.8 cm); leg fusion on/off or ×1.189: within 0.5 cm |
| Built-in odometry ×1.189, same anchoring | lateral error max 22–33 cm: under-reads forward distance by 14–19 %, more at speed (×1.155 at 0.42 m/s, ×1.23 at 1.0 m/s); yaw within 0.5 %, no sideways slip |
| Start lock | 6/6 in the right lane after 5 scans (0.5 s); the other lane refused 6/6 |
| `day1.sh up` + runner on a recording | up 16 s, start found as offline |

| ROS integration (fake robot, another robot in the other lane) | Result |
|---|---|
| lane 2 / lane 1 at 0.5 m/s | 32.6 s / 44.5 s, no line crossing, foot margin ≥ 24 cm |
| lane 1 at 0.8 m/s from the curve | 28.1 s, no line crossing |
| placed 0.25 m off and 10° towards the line | no line crossing (margin 0.9–1.9 cm, set by the placement itself) |

Not yet run on the robot.
