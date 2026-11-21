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
scripts/day1.sh run in v=0.5        # inner lane (out = outer lane); robot on its start mark; space / x = STOP
scripts/day1.sh down
```
`run` drives one counter-clockwise lap of the chosen lane, plus `extra`, then stops.

Runner parameters: `in` / `out` (or `lane` = 2 / 1), `v` (m/s), `extra` (m past one lap, default 1.2), `green_s`, `light` (`model`, `none` for tests only), `gait`, `max_lat` (0.35 m), `max_head` (15°), `lock_n` (5 scans), `track_dx` / `track_dy` / `track_dyaw_deg`, `path_file` (below).

### Taught route (cones on the track)
Fixed obstacles: drive the route once with the remote, then the runner repeats it.
```bash
scripts/day1.sh teach in name=cones_in   # robot standing on its start mark (after up): drive ONE lap with the remote,
                                          # saved when it comes back past the start (Enter = save now, x = discard)
scripts/day1.sh run in route=cones_in v=0.5
scripts/day1.sh routes                    # list; each route has a .png next to it
```
- Nothing goes to the motors while teaching. The route is stored in the track frame (from the wall), so a start placed a bit differently still uses it. The lap end is one route lap + `extra`.
- ★ Start from the mark you taught from. The wall cannot tell the two straights apart (180° symmetry), so a route taught from one start mark is wrong from the opposite one: teach one route per start mark. The runner refuses a start more than 1 m from where teaching began.
- `route=` runs default to `lookahead=0.5` and `min_radius=0.4` (RPP slows down only below 0.4 m radius). On a ±18 cm / 2 m slalom: route error max 6.7 cm (0.8 m look-ahead: 13.6 cm), 0.5 m/s ~34 s, 0.6 m/s ~29 s on the inner course.

### Changing the path (default: none)
The wall fit gives the centre of the wall rectangle. By default the drawn track is assumed to sit exactly in it, 0.495 m from the wall on every side (rulebook).

**Track offset**: `track_dx`, `track_dy` (m), `track_dyaw_deg`. This is the drawn track centre relative to the wall centre, in the **start-relative frame**: the robot's start straight is at the bottom (y < 0) and travel along it is +x. Measure the gap from the track's outer line to the wall with a tape on all four sides:
- `track_dx` = (gap behind the robot − gap ahead) / 2
- `track_dy` = (gap on the start-straight side − gap on the opposite side) / 2
- `track_dyaw_deg` = degrees((gap on the start side at the front end of the straight − at the back end) / 2.0), measured at both ends of the 2.0 m straight

★ Starting on the opposite straight turns this frame by 180°: `track_dx` and `track_dy` change sign, `track_dyaw_deg` stays the same.

**Pin file**: `path_file=/path/pins.csv`. Columns `lane,station_m,x_m,y_m,yaw_deg`, in the same start-relative track frame. The pins of the chosen lane are read in station order as a closed loop, smoothed and resampled every 0.1 m. Progress and the lap end are measured along that loop. `yaw_deg` is not used. The 0.5 m rulebook pins (`logs/pins/day1_pins.csv`) give the default path within 4–6 mm. `track_dx/dy/dyaw` apply on top.

## On site
1. `up` refuses if bridges, FAST-LIO or a controller from another stack run on the same ROS domain. Stop those first. FAST-LIO initialises its IMU at start-up, so the robot must stand still.
2. `run` logs `start found: lane …, lateral …, heading …` within about 1 s of ClassicWalk and saves `logs/day1/plan_HHMMSS.png` (start and end on the lane centre, the robot where it stands). If it says `robot is in lane X, not lane Y` or `placement: …`, re-place the robot. The run does not start until the start is found.
3. `light` before `run`: the runner starts only after GREEN for 0.5 s with no RED in the last 1 s (same light as RoboWorld). Without light messages it waits and warns; press `g` in the runner terminal to start anyway. `lightcheck [s]` watches the light without moving the robot.
4. Lane 1 needs ≥ 0.36 m/s average to finish inside 60 s. `v=0.5` gives ~44 s on lane 1 and ~33 s on lane 2. Raise it only after a clean run.
5. The runner stops 1.2 m after the lap. Timing ends when the rear feet cross the line, so the overshoot costs nothing.

## Tests (no robot)
```bash
tests/day1_ros_integration.sh lane=1 off0=-0.25 yaw0=-10        # real Nav2 + bridge + runner, fake robot (domain 81, loopback)
tests/fastlio_replay.sh <bag> out.csv domain=82                 # FAST-LIO on a recording (loopback)
python3 tools/odom_vs_wall.py <bag> traj=out.csv scales=1.0     # its error against the wall poses
python3 tools/plot_plan.py                                      # picture: start and end on the lane centre, both courses
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
| `path_file=logs/pins/day1_pins.csv`, in / out | same as the default: lap time within 0.3 s, path within 0.3 cm (rms 0.1–0.15 cm) |
| `track_dx=0.05` / `track_dy=0.05` | path moved 4.9 / 5.0 cm in x / y, no line crossing |
| through `day1.sh run out` | 44.5 s, no line crossing |
| no traffic-light messages | waits at the start (moved 0.0 m), warns every 3 s |

Not yet run on the robot.
