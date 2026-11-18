# track_day1

Day 1 track race: follow the assigned lane for one lap with the D435 and the built-in odometry. No map, no SLAM.

```
D435 (internal PC)                       NUC
band_vision_node ── /track/edges ──▶ day1_runner_node ── nav_msgs/Path (odom) ──▶ Nav2 controller_server (RPP)
  floor plane from depth                 LanePath: centre line in the odom frame       │ /cmd_vel
  bird's-eye view, colour edges          start: lane search ±0.35 m ±12°              ▼
  band-boundary test                     curve entry: curvature hypotheses        go2_bridge (yaw trim, ClassicWalk)
                                         lap end: odom distance (×1.189)
YOLO traffic light ── /mission/traffic_light_state ──▶ GREEN 0.5 s, no RED within 1 s
```

## Files
| File | |
|---|---|
| `track_spec.py` | Rulebook track (lane 900 mm, R1185–R3005, straight 2000, counter-clockwise) |
| `band_vision_core.py` | Floor edges from one colour + depth pair (Python 3.8, numpy, OpenCV) |
| `band_vision_node.py`, `jetson_track.sh` | Internal PC node (Foxy) and its start script |
| `lane_path.py` | Lane centre line in the odometry frame |
| `day1_logic.py` | Start light gate, lap end, RPP core for the offline sim |
| `day1_runner_node.py` | NUC runner (Jazzy) |
| `sim/` | Synthetic floor and camera, vision evaluation, closed-loop sim, fake robot for the ROS test |

Config: `config/day1_nav2.yaml` (controller_server + RPP). Launcher: `scripts/day1.sh`.

## Usage
```bash
scripts/day1.sh vision push          # once: copy the edge node to the internal PC
scripts/day1.sh vision start         # camera + edge node
scripts/day1.sh light                # YOLO traffic-light node
scripts/day1.sh up                   # odom TF ×1.189, motor bridge, controller_server
scripts/day1.sh vision snap          # check images -> logs/day1/snap/
scripts/day1.sh run lane=2 v=0.5     # robot standing in its lane; space / x = STOP
scripts/day1.sh down
```
Runner parameters: `lane` (1 outer, 2 inner), `v` (m/s), `extra` (m past one lap, default 1.2), `green_s`, `light` (`model`, `none` for tests only), `gait`, `coast_s`.

## On site
1. Tilt the D435 down 15–30°. Level, the near 1.1 m is not visible and the own-lane line drops out on curves (81 % of frames).
2. Robot at the start position, `vision snap`: red points must lie on the lane lines and band edges, none on arrows or numbers. If the print has little contrast, lower `side_de` in `band_vision_core.py`.
3. `light`: the model was trained on the RoboWorld light. Check it sees the CO SHOW light before the first run.
4. Place the robot within ±0.35 m and ±12° of the lane centre. The runner logs `lane found: centre … heading …`; compare with where the robot actually is.
5. Lane 1 needs ≥ 0.36 m/s average to finish inside 60 s (20.0 m + rear feet). `v=0.5` gives ~44 s, `v=0.65` ~33 s on lane 2 → raise only after a clean run.
6. The runner stops 1.2 m after the lap. Timing ends when the rear feet cross the line, so the overshoot costs nothing.
7. The motor bridge runs with `min_wz 0` / `min_vy 0`: the default 0.15 rad/s floor turns small steering corrections into zig-zag.

## Tests (no robot)
```bash
python3 sim/eval_vision.py           # vision over 120 random poses × 9 conditions
python3 sim/sim_day1.py              # 16 closed-loop scenarios
tests/day1_ros_integration.sh lane=1 off0=0.15 yaw0=8   # real Nav2 + bridge + runner, fake robot (domain 81, loopback)
```

| Check | Result |
|---|---|
| Floor plane | 100 % of frames, camera height p95 1.6 cm |
| Boundary position | median 1.2 cm, p95 3.4 cm |
| False features within 28 cm of a boundary | ≤ 0.1 % (arrows, numbers, start lines, glare, seams rejected by the band test) |
| Closed loop, 16 scenarios (placement ±15 cm / ±8°, curve start, drops 25 %, glare, other robot, dim light) | no line crossing, min foot margin 11.5 cm |
| ROS integration (real RPP) | lane 2 32.2 s, lane 1 44.0 s at 0.5 m/s, no line crossing |

Not yet verified on the real floor, light and internal PC.
