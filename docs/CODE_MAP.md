# Code Map

Every Python file in the repository, grouped by folder. *Generated from the same
descriptions that are written into each file's `REPO MAP` header comment, so the two
agree.* Status key: **ACTIVE** used by the running robot · **WORKSPACE** current
experiments · **SUPERSEDED** replaced but kept · **ARCHIVED** history · **DEV TOOL** bench
utility · **PROTOTYPE** voice/vision · **SENSOR BENCH TOOL** stand-alone · **UNUSED** nothing imports it.

## At a glance

| Area | Python files | Lines |
|---|--:|--:|
| ★ `robot/` — the active system | 7 | 2,917 |
| `robot/lib/` — libraries (driver → runtime → homing → IK → gaits) | 6 | 2,542 |
| `robot/config/` — configuration | 1 | 81 |
| `robot/tests/` — bench tools (run as `python3 -m tests.<name>`) | 8 | 1,445 |
| `sensors/mpu_6050/` — IMU bench scripts | 5 | 687 |
| `cur_test/` — experiment workspace | 23 | 7,707 |
| `cur_test/extra_files/` — older gait / calibration / pre-test scripts | 16 | 5,332 |
| `cur_test/BYTE_VA/` — voice assistant | 10 | 1,200 |
| `cur_test/BYTE_VISION/` — vision | 6 | 840 |
| `archive/01_single_motor_experiments/` | 15 | 4,024 |
| `archive/02_multi_motor_socket_control/` | 11 | 3,014 |
| `archive/03_single_leg_ik/` | 5 | 708 |
| `archive/03_single_leg_ik/main_ik_runtime/` | 6 | 1,303 |
| `archive/04_imu_balancing/` | 14 | 4,297 |
| `archive/05_four_leg_bringup/final/` | 3 | 1,073 |
| `archive/05_four_leg_bringup/with_ik/` | 9 | 2,319 |
| `archive/05_four_leg_bringup/controller/` | 1 | 40 |
| **Total** | **146** | **39,529** |


## ★ `robot/` — the active system

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`keyboard_controller.py`](../robot/keyboard_controller.py) | 306 | ACTIVE* | WASD terminal client with a live key display. *Currently incompatible with v2 (port/body) — KNOWN_ISSUES F1. <br>▶ `sudo python3 keyboard_controller.py` |
| [`main_with_ik.py`](../robot/main_with_ik.py) | 1175 | ACTIVE | **Motor runtime.** Opens both CAN buses, verifies feedback, optionally homes (two bus threads in lock-step), sits and re-zeroes, then runs the 100 Hz live loop: socket packet → IK → flip/gear → rate limit → MIT frames. Also CSV loggers and an orderly shutdown. <br>▶ `python3 main_with_ik.py y\|n` |
| [`mpu_leveling.py`](../robot/mpu_leveling.py) | 490 | ACTIVE | `MPULeveler`: reads the MPU-6050 (averaged), computes per-leg Z corrections from pitch/roll vs a sitting reference, sends one corrective packet. Includes an axis-verification mode. <br>▶ `python3 mpu_leveling.py` |
| [`state_manager_server.py`](../robot/state_manager_server.py) | 199 | SUPERSEDED | First version of the FastAPI state manager: same /change-state API as v2 but WITHOUT the CW/CCW rotation states, and SIT/STAND use dx = 0 (v2 uses 1.5). Full documentation: state_manager_server_v2.py. |
| [`state_manager_server_v2.py`](../robot/state_manager_server_v2.py) | 380 | ACTIVE | **HTTP API + gait runner** (FastAPI :8000). `POST /change-state` → SIT/STAND or N cycles of trot/strafe/rotate, sent as foot-offset packets to the runtime. One motion at a time (429 if busy). <br>▶ `python3 state_manager_server_v2.py` |
| [`state_manager_server_v3.py`](../robot/state_manager_server_v3.py) | 281 | EXPERIMENT | Variant of the state server that creates an MPULeveler at start-up (needs the MPU-6050 on I2C) so SIT captures a tilt reference and STAND applies a level correction. It lacks the rotation states (CW/CCW) of v2 and adds an unused next_state_is_walk() placeholder. Full documentation: state_manager_server_v2.py. <br>⚠ Its STAND uses dx = 0 while mpu_leveling.py sends STAND_DX = 4. |
| [`test_client.py`](../robot/test_client.py) | 86 | ACTIVE | Interactive prompt that POSTs a state name to the server. <br>▶ `python3 test_client.py` |

## `robot/lib/` — libraries (driver → runtime → homing → IK → gaits)

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../robot/lib/ak60_v3_control.py) | 424 | ACTIVE | **CAN driver** for the AK60-6 V3.0: MIT-mode frame packing, origin setting, current-brake (=disable), status-frame decoding. Protocol cheat-sheet in the docstring. |
| [`can_runtime.py`](../robot/lib/can_runtime.py) | 611 | ACTIVE | **Per-bus runtime**: `MotorState`, the feedback listener thread, watchdogs (stale motor / bus silence / BUS-OFF / listener death), latched faults, hold targets, startup gate. |
| [`homing_controller.py`](../robot/lib/homing_controller.py) | 747 | ACTIVE | **Hard-stop homing**: profiled search → creep → current-spike contact latch → confirm → temporary zero → nudge back. `ControlTuning` holds every gain/threshold. |
| [`inverse_kinematics.py`](../robot/lib/inverse_kinematics.py) | 281 | ACTIVE | **Leg kinematics**: closed-form IK/FK for the 3-DOF leg, per-side sign handling, conversion to sit-relative motor angles. Checked numerically (round-trip < 1e-13 cm). <br>▶ `python3 -m lib.inverse_kinematics` |
| [`trot_gait.py`](../robot/lib/trot_gait.py) | 377 | ACTIVE | **Foot-trajectory generators**: `TrotGaitController` (forward/back/strafe, cycloidal swing + linear stance) and `RotationGaitController` (arc trajectories). Return per-tick increments. |
| [`trot_gait_lib.py`](../robot/lib/trot_gait_lib.py) | 102 | UNUSED | Stateless trot engine computing deltas from absolute positions (design reference; nothing imports it). |

## `robot/config/` — configuration

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`robot_config.py`](../robot/config/robot_config.py) | 81 | ACTIVE | **Single source of truth**: network address, default speed limit, `LEG_ORDER`, `SIT_COORDS`, `SIT_TARGETS_DEG`. |

Data / config files here: `motor_config.json`.

## `robot/tests/` — bench tools (run as `python3 -m tests.<name>`)

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ik_sender.py`](../robot/tests/ik_sender.py) | 59 | DEV TOOL | Send ONE hand-edited foot offset (dx, dy, dz in cm, from the sit pose) to the motor runtime and hold it for HOLD_SECONDS. NOTE: the same offset is applied to ALL four legs (the single-leg variant is commented out). Frame: +x forward, +y right, +z up. <br>▶ `python3 -m tests.ik_sender   (from robot/, with main_with_ik.py running)` <br>⚠ The inline comment 'make +14 to stand' is wrong: STAND is dz = -14. |
| [`joint_sender.py`](../robot/tests/joint_sender.py) | 65 | DEV TOOL | Interactive prompt: type a motor ID (1-12) and an angle; sends {motor_id: angle, 'speed': 30} (raw joint space, no IK) to joint_tester.py. <br>▶ `python3 -m tests.joint_sender   (with tests.joint_tester running)` <br>⚠ Its error message still names the old 'main_raw_tester.py'. |
| [`joint_tester.py`](../robot/tests/joint_tester.py) | 193 | DEV TOOL | A STAND-ALONE alternative to main_with_ik.py for bare-metal joint testing: opens both CAN buses, ZEROES every motor exactly where it is (no homing), then moves single motors to raw angles received over the socket, capped at 30 deg/s. No gear ratio, no flip, no IK. <br>▶ `python3 -m tests.joint_tester   (from robot/)` <br>⚠ Never run it together with main_with_ik.py: both would grab the same CAN buses and TCP port. |
| [`mpu_levelingold.py`](../robot/tests/mpu_levelingold.py) | 461 | SUPERSEDED | Earlier revision of the one-shot MPU-6050 chassis leveler (its docstring talks about correcting leg Y; the current version corrects Z). See robot/mpu_leveling.py. |
| [`state_manager.py`](../robot/tests/state_manager.py) | 76 | SUPERSEDED | Earliest QuadrupedStateManager: only 'sit' and 'stand', sends the pose over the socket. Replaced by the async FastAPI server. |
| [`state_manager_sidd.py`](../robot/tests/state_manager_sidd.py) | 195 | EXPERIMENT | Synchronous state manager (sit/stand) that can also start/stop a trot gait script as a subprocess. Sends deltas (target - current) instead of absolute offsets. |
| [`trot_gait_node_daniel.py`](../robot/tests/trot_gait_node_daniel.py) | 106 | SUPERSEDED | Phase-based trot that streams foot positions to the socket. Parameters at the top (stride 6/4 cm, lift 4, ground push 3, 800 deg/s). Superseded by lib/trot_gait.py + the state server. |
| [`trot_gait_node_sid.py`](../robot/tests/trot_gait_node_sid.py) | 290 | SUPERSEDED | Interactive terminal trot (f/ff/b/bb/r/rr/l/ll) built on a phase-based cycloidal gait; assumes the robot is already standing (sit/stand belongs to the state manager). Superseded by lib/trot_gait.py. |

## `sensors/mpu_6050/` — IMU bench scripts

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`aarush_imu.py`](../sensors/mpu_6050/aarush_imu.py) | 271 | SENSOR BENCH TOOL | Interactive MPU-6050 tool: R = take reference, M = capture maximum tilt angles (saved to imu_max_angles.json), S = start. Works out the effective height change at each corner of the body cuboid and renders a headless matplotlib plot to /tmp/imu_plot.png. <br>▶ `python3 aarush_imu.py` |
| [`calibration.py`](../sensors/mpu_6050/calibration.py) | 13 | SENSOR BENCH TOOL | Auto-generated MPU-6050 bias values (ACCEL_BIAS, GYRO_BIAS) from a Raspberry Pi 4B; sensor must be flat, Z up. Data only. |
| [`check_leg_lifts.py`](../sensors/mpu_6050/check_leg_lifts.py) | 195 | SENSOR BENCH TOOL | Reads the IMU and turns posture into a 0-10 'lift level' per leg (R = reference, S = start sensing, Q = stop, X = exit). <br>▶ `python3 check_leg_lifts.py` |
| [`get_roll_pitch.py`](../sensors/mpu_6050/get_roll_pitch.py) | 153 | SENSOR BENCH TOOL | Prints roll/pitch from the MPU-6050 accelerometer after a reference capture; applies calibration offsets and limit bands. <br>▶ `python3 get_roll_pitch.py` |
| [`raw_check.py`](../sensors/mpu_6050/raw_check.py) | 55 | SENSOR BENCH TOOL | Smallest possible MPU-6050 test: wake the chip over I2C bus 1 (address 0x68) and print raw readings. <br>▶ `python3 raw_check.py` |

Data / config files here: `imu_max_angles.json`.

## `cur_test/` — experiment workspace

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../multiprocess_code/all_4_legs/cur_test/ak60_v3_control.py) | 238 | WORKSPACE | Byte-identical copy of robot/lib/ak60_v3_control.py as it was before documentation (the same driver also lives in archive/04_imu_balancing, cur_test/ and BYTE_VISION/). Documentation lives in the robot/lib version. |
| [`bound_gait_sidd.py`](../multiprocess_code/all_4_legs/cur_test/bound_gait_sidd.py) | 351 | WORKSPACE | Interactive BOUND gait (front pair and back pair alternate), derived from trot_gait_node_sid.py. Sends raw XYZ foot targets over the socket; uses mpu_leveling.py. Starts with SIT then eases to STAND. <br>▶ `python3 bound_gait_sidd.py` |
| [`byte_balance.py`](../multiprocess_code/all_4_legs/cur_test/byte_balance.py) | 426 | WORKSPACE | IMU balance controller v2.3: MPU-6050 + complementary filter, computes per-joint knee deltas to keep the body level. Interactive shell: R = capture flat reference, S = start, Q = pause, X = exit. <br>▶ `python3 byte_balance.py   (after main.py homing + main_sender.py)` <br>⚠ Written against the older runtime (it mentions main.py / transform_live_angles, i.e. joint-angle payloads). |
| [`can_runtime.py`](../multiprocess_code/all_4_legs/cur_test/can_runtime.py) | 390 | WORKSPACE | Older flat copy of robot/lib/can_runtime.py (imports 'from ak60_v3_control import'). Same bus runtime, before the lib/ package layout. Documentation: robot/lib/can_runtime.py. |
| [`controller_test.py`](../multiprocess_code/all_4_legs/cur_test/controller_test.py) | 72 | WORKSPACE | pygame gamepad check: prints what the connected controller reports so you can find axis/button numbers for kriss_trot_controller.py. <br>▶ `python3 controller_test.py` |
| [`daniel_ik_gait_promaxultra.py`](../multiprocess_code/all_4_legs/cur_test/daniel_ik_gait_promaxultra.py) | 324 | WORKSPACE | Interactive terminal trot controller. f/b = forward/back, r/l = strafe (repeat the letter for more steps), s = sit, stand, q = quit. Two diagonal pairs alternate (FR+BL, FL+BR). Works in the older Y-up frame (forward/back on the Z axis). <br>▶ `python3 daniel_ik_gait_promaxultra.py` |
| [`homing_controller.py`](../multiprocess_code/all_4_legs/cur_test/homing_controller.py) | 520 | WORKSPACE | Older flat copy of robot/lib/homing_controller.py (imports 'from can_runtime import'). Documentation of the algorithm: robot/lib/homing_controller.py. |
| [`ik_sender.py`](../multiprocess_code/all_4_legs/cur_test/ik_sender.py) | 61 | WORKSPACE | Send one test coordinate to the legs and watch them move (run after the runtime shows LIVE CONTROL READY). No 'speed' key is sent, so the runtime's default speed limit applies. <br>▶ `python3 ik_sender.py` <br>⚠ HOST is hard-coded to a LAN IP; edit it for your network. |
| [`imu_correction.py`](../multiprocess_code/all_4_legs/cur_test/imu_correction.py) | 154 | WORKSPACE (new) | Wraps MPULeveler in a background thread that polls the MPU-6050 and exposes per-leg Y corrections via a thread-safe get(); also bundles two start-up health gates: validate_mpu() and validate_socket(). Used by kriss_trot_controller.py --imu. |
| [`imu_reader.py`](../multiprocess_code/all_4_legs/cur_test/imu_reader.py) | 323 | WORKSPACE | ImuReader: background thread reading the MPU-6050 at ~200 Hz and keeping complementary-filtered roll/pitch; get_angles() returns degrees. Mounting signs MOUNT_ROLL_SIGN / MOUNT_PITCH_SIGN are tunable (roll>0 right side low, pitch>0 front low). |
| [`inverse_kinematics.py`](../multiprocess_code/all_4_legs/cur_test/inverse_kinematics.py) | 208 | WORKSPACE | A DIFFERENT IK design from robot/lib/inverse_kinematics.py: it calibrates each leg from a measured table (LEG_CALIBRATION: sit coordinates + the motor angles at that pose) in the older Y-up frame, instead of SIT_COORDS in config/robot_config.py. |
| [`kriss_exp.py`](../multiprocess_code/all_4_legs/cur_test/kriss_exp.py) | 244 | WORKSPACE | Single-motor bench tool for motor 2 on can0. Modes: --position <deg> (hold), --velocity <val> (spin), --none (zero torque, free spin by hand). Prints live telemetry and records it to CSV (kriss_exp_recording.csv). <br>▶ `sudo python kriss_exp.py --position 60   (kill: sudo kill -9 $(pgrep -f kriss_exp.py))` |
| [`kriss_trot_controller.py`](../multiprocess_code/all_4_legs/cur_test/kriss_trot_controller.py) | 390 | WORKSPACE | Gamepad (pygame) trot controller. Left stick: forward/back/strafe; buttons 1/2 = sit/stand, 3 = emergency stop; D-pad tilts the body (nose/butt/left/right) and returns to flat on release. --imu enables live MPU-6050 disturbance rejection via imu_correction.py. <br>▶ `python3 kriss_trot_controller.py [--imu]` |
| [`main_sender.py`](../multiprocess_code/all_4_legs/cur_test/main_sender.py) | 76 | WORKSPACE | Sends the initial standing pose to all four legs over the socket (the second step of the older start-up order: runtime -> main_sender -> gait/test script). <br>▶ `python3 main_sender.py` <br>⚠ HOST is hard-coded to '0.0.0.0': works only when run on the same machine as the runtime; use the runtime's IP otherwise. |
| [`main_with_current_temp.py`](../multiprocess_code/all_4_legs/cur_test/main_with_current_temp.py) | 844 | WORKSPACE | Predecessor of main_with_ik.py: the same homing + 2-bus runtime + current/temperature CSV logging + socket receiver, but the socket payload is JOINT ANGLES (no IK import). See robot/main_with_ik.py for the documented flow. <br>▶ `python3 main_with_current_temp.py y` <br>⚠ Writes logs to an absolute path inside cur_test/. |
| [`main_with_ik.py`](../multiprocess_code/all_4_legs/cur_test/main_with_ik.py) | 908 | WORKSPACE | cur_test copy of the runtime. Differs from robot/main_with_ik.py in: flat imports, motor_config.json in the working directory, a hard-coded bind IP, MAX_LIVE_DEG_PER_S defined locally, an extra transform_live_angles(), and ABSOLUTE log paths into cur_test/. Flow and safety notes: robot/main_with_ik.py. <br>▶ `python3 main_with_ik.py y\|n` |
| [`motor_current_plotter.py`](../multiprocess_code/all_4_legs/cur_test/motor_current_plotter.py) | 44 | WORKSPACE | Plots motor_currents.csv (written by the runtime) with matplotlib, grouped by motor group, time axis starting at 0. <br>▶ `python3 motor_current_plotter.py` <br>⚠ The CSV path is absolute; this folder must not be moved without editing it. |
| [`mpu_leveling.py`](../multiprocess_code/all_4_legs/cur_test/mpu_leveling.py) | 459 | WORKSPACE | cur_test variant of the one-shot MPU-6050 chassis leveler; it corrects leg Y (older frame). The current version, which corrects Z, is robot/mpu_leveling.py. |
| [`pace_gait_sidd.py`](../multiprocess_code/all_4_legs/cur_test/pace_gait_sidd.py) | 351 | WORKSPACE | Interactive PACE gait (legs on the same side move together), derived from trot_gait_node_sid.py: phase-based cycloidal gait, raw XYZ foot targets over the socket, SIT then ease to STAND. <br>▶ `python3 pace_gait_sidd.py` |
| [`sidd_crawl_gait.py`](../multiprocess_code/all_4_legs/cur_test/sidd_crawl_gait.py) | 399 | WORKSPACE | Interactive CRAWL (static walk) gait: only one leg swings at a time so three feet are always on the ground (stable on stairs), at the cost of speed. Same socket/IK pipeline as the trot scripts. <br>▶ `python3 sidd_crawl_gait.py` |
| [`sidd_gait.py`](../multiprocess_code/all_4_legs/cur_test/sidd_gait.py) | 410 | WORKSPACE | Interactive terminal trot controller (f/ff = forward, b/bb, r/rr strafe right, l/ll strafe left; double letter = continuous until 'y'). Starts SITTING then eases to STANDING. <br>▶ `python3 sidd_gait.py` |
| [`single_ik.py`](../multiprocess_code/all_4_legs/cur_test/single_ik.py) | 169 | WORKSPACE | leg_commander: interactive per-leg coordinate sender. Keys 1-4 edit FL/BL/FR/BR, a = all legs, s = send, v = view payload. <br>▶ `python3 single_ik.py` |
| [`trot_gait_node_sid.py`](../multiprocess_code/all_4_legs/cur_test/trot_gait_node_sid.py) | 346 | WORKSPACE | Interactive terminal trot (phase-based cycloidal gait ported from trot_gait_node.py): f/ff, b/bb, ... IK is external - raw XYZ foot targets go over the socket. Uses mpu_leveling. <br>▶ `python3 trot_gait_node_sid.py` |

Data / config files here: `kriss_exp_recording.csv`, `motor_config.json`, `motor_currents.csv`, `motor_temps.csv`, `motor_urdf_stand.csv`.

## `cur_test/extra_files/` — older gait / calibration / pre-test scripts

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`daniel_ik_gait.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/daniel_ik_gait.py) | 269 | SUPERSEDED | Earlier version of the interactive trot controller (f/ff.. = N half cycles, s = sit, stand, q = quit) with smooth transitions. |
| [`fr_leg_calibrate.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/fr_leg_calibrate.py) | 767 | SUPERSEDED | Front-right leg (motors 7/8/9, can1) cycloidal-trajectory calibration tool: build a trajectory, run it, and record observations. Start-up order: main_with_ik.py, main_sender.py, then this. |
| [`gait_cycloidal.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/gait_cycloidal.py) | 222 | SUPERSEDED | Cycloidal gait for the FR leg only (others hold sit). 'f' = one step cycle, 'q' = quit. |
| [`gait_pattern_daniel_latest.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/gait_pattern_daniel_latest.py) | 305 | SUPERSEDED | Configurable diagonal-trot generator that sends grouped leg coordinate payloads {fl, bl, fr, br: [x,y,z]} to main_with_ik.py. |
| [`gait_walk_real.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/gait_walk_real.py) | 710 | SUPERSEDED | Walk gait for the real robot (3-DOF IK): port of a Gazebo simulation walker (gait_walk_3dof.py) to the physical 'byte' robot. |
| [`ik_fl_sender.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/ik_fl_sender.py) | 366 | SUPERSEDED | IK sender for the FL leg (motors 1,2,3): moves to the known stand pose, prints the forward-kinematics coordinates, then lets you type new (x,y,z) targets interactively. |
| [`ik_visualizer.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/ik_visualizer.py) | 97 | SUPERSEDED | Leg IK plus a joint-position calculator for drawing the leg geometry (L1=5.995, linkConst=-9.094, L2=22, L3=21.5). |
| [`main.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/main.py) | 657 | SUPERSEDED | Early runtime (homing + sockets) that takes joint-angle payloads with transform_live_angles(); the ancestor of main_with_ik.py. |
| [`main_only_homing.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/main_only_homing.py) | 350 | SUPERSEDED | Homing-only program for the two buses: runs both phases and exits, no live control. |
| [`pretest_feedback_only.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/pretest_feedback_only.py) | 83 | SUPERSEDED | Bench pre-test: on can0 motors 4,5,6 only read and print status feedback for 20 s (no commands sent). |
| [`pretest_idle_hold.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/pretest_idle_hold.py) | 98 | SUPERSEDED | Bench pre-test: hold can0 motors 4,5,6 at their current position with kp=40, kd=1 at 100 Hz. |
| [`pretest_single_leg_phase1.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/pretest_single_leg_phase1.py) | 138 | SUPERSEDED | Bench pre-test: run only homing phase 1 for the back-left leg (motors 5,6,4 on can0) before trusting the full routine. |
| [`tapping_gait.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/tapping_gait.py) | 417 | SUPERSEDED | Tapping gait with IMU balance correction: lifts and replaces each foot in the order fl, br, fr, bl; sends 'speed': 900 to override the runtime's default speed limit. |
| [`trot_all_legs_daniel.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/trot_all_legs_daniel.py) | 218 | SUPERSEDED | Trot expressed as ANGLE offsets on top of the standing pose (no IK) using the live socket protocol of main_sender.py. |
| [`trot_gait.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/trot_gait.py) | 263 | SUPERSEDED | Trot with diagonal pairs: each 'f' press runs one cycle (pair A swings while pair B pushes, then swap). Pair A = FR+BL, pair B = FL+BR. |
| [`walk_gait_sender.py`](../multiprocess_code/all_4_legs/cur_test/extra_files/walk_gait_sender.py) | 372 | SUPERSEDED | Streams a walk gait (2-DOF IK, swing/stance trajectories) as leg-angle payloads to main.py over the TCP socket. |

Data / config files here: `leg_check.png`.

## `cur_test/BYTE_VA/` — voice assistant

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`app.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/app.py) | 154 | PROTOTYPE | Voice-assistant main loop: LISTENING (VAD + wake-word) -> RECORDING (3 s) -> TRANSCRIBING (faster-whisper). If the text contains 'stand' or 'sit' it runs byte_stand.py / byte_sit.py as a subprocess. <br>▶ `python3 app.py   (inside BYTE_VA/, with its byte_env)` <br>⚠ byte_stand.py / byte_sit.py use the OLD absolute-coordinate protocol; with the robot/ runtime they would be interpreted as offsets from SIT_COORDS (wrong or unreachable pose). |
| [`audio_stream.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/audio_stream.py) | 150 | PROTOTYPE | Non-blocking 16 kHz mono microphone capture (sounddevice) into a thread-safe ring buffer of float32 samples. |
| [`byte_sit.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/byte_sit.py) | 58 | PROTOTYPE | One-shot: send the SIT pose [-9.65, 11, 6] cm to all legs on 127.0.0.1:50000, hold 3 s. Triggered by app.py for 'sit'. <br>⚠ Old coordinate protocol - not compatible with robot/. |
| [`byte_stand.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/byte_stand.py) | 58 | PROTOTYPE | One-shot: send the STAND pose [-9.65, 20, 6] cm to all legs on 127.0.0.1:50000, hold 3 s. Triggered by app.py for 'stand'. <br>⚠ Old coordinate protocol - not compatible with robot/. |
| [`check_audio.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/check_audio.py) | 21 | PROTOTYPE | Prints the list of audio devices PortAudio can see (sanity check for the microphone). <br>▶ `python3 check_audio.py` |
| [`download_models.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/download_models.py) | 220 | PROTOTYPE | One-time setup: downloads silero_vad.onnx and the Zipformer KWS model into models/, writes keywords_raw.txt and tokenizes it to keywords.txt. <br>▶ `python3 download_models.py` |
| [`ranscriber.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/ranscriber.py) | 96 | PROTOTYPE | Offline speech-to-text with faster-whisper (tiny.en, CPU, int8), biased by an initial prompt toward short robot commands. <br>⚠ File name is a typo for 'transcriber.py'; app.py imports it under this name, so rename both together. |
| [`sherpa_mic_example.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/sherpa_mic_example.py) | 203 | PROTOTYPE | Upstream sherpa-onnx example: real-time keyword spotting from a microphone. Kept as a reference for wake_word_engine.py. |
| [`test_kws.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/test_kws.py) | 41 | PROTOTYPE | Quick test: WakeWordEngine fed from AudioStream, prints when the wake word fires. <br>▶ `python3 test_kws.py` |
| [`wake_word_engine.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VA/wake_word_engine.py) | 199 | PROTOTYPE | Two-stage wake word: Silero VAD (sherpa-onnx) gates audio, then a Zipformer keyword spotter looks for the keywords. Includes a Windows DLL-path workaround for onnxruntime. |

Data / config files here: `keywords.txt`, `keywords_raw.txt`.

## `cur_test/BYTE_VISION/` — vision

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/ak60_v3_control.py) | 238 | PROTOTYPE | Byte-identical copy of robot/lib/ak60_v3_control.py as it was before documentation (the same driver also lives in archive/04_imu_balancing, cur_test/ and BYTE_VISION/). Documentation lives in the robot/lib version. |
| [`enrollment.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/enrollment.py) | 90 | PROTOTYPE | Enroll a person: asks for a name in the terminal, then captures face images with an Intel RealSense camera into team_members/. <br>▶ `python3 enrollment.py` |
| [`integrated.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/integrated.py) | 228 | PROTOTYPE | Voice + vision agent: a voice thread (speech_recognition + pyttsx3, 'Yes?' prompt) answers through Google Gemini, while a vision thread runs YOLOv8 on RealSense frames. <br>▶ `python3 integrated.py` <br>⚠ SECURITY: contains a hard-coded Google API key (line ~13) that is in git history and on the remote. Revoke it and read a new one from an environment variable. |
| [`object_detection.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/object_detection.py) | 122 | PROTOTYPE | YOLOv8n-seg object detection/segmentation on an Intel RealSense colour+depth stream (nano model chosen for the Pi). <br>▶ `python3 object_detection.py` |
| [`udp_receiver.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/udp_receiver.py) | 44 | PROTOTYPE | Debug listener: prints 'Target Lock' packets received on UDP 0.0.0.0:5005. <br>▶ `python3 udp_receiver.py` |
| [`vision.py`](../multiprocess_code/all_4_legs/cur_test/BYTE_VISION/vision.py) | 118 | PROTOTYPE | Face recognition against the photos in team_members/, using a RealSense colour+depth stream (depth is used to estimate how far away the face is). <br>▶ `python3 vision.py` |

Data / config files here: `yolov8l-seg.pt`, `yolov8n-seg.pt`.

## `archive/01_single_motor_experiments/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../archive/01_single_motor_experiments/ak60_v3_control.py) | 331 | ARCHIVED | Early AK60 driver revision used by the first single/dual-motor tests. Superseded by robot/lib/ak60_v3_control.py. |
| [`amshu.py`](../archive/01_single_motor_experiments/amshu.py) | 240 | ARCHIVED | Variant of the single-motor position step test (MIT mode, smooth stepping with error tracking). |
| [`anglespeedtorque.py`](../archive/01_single_motor_experiments/anglespeedtorque.py) | 207 | ARCHIVED | Position step test variant (the name suggests it records angle, speed and torque). |
| [`diagnostic.py`](../archive/01_single_motor_experiments/diagnostic.py) | 356 | ARCHIVED | AK60-6 V3.0 + Waveshare CAN HAT diagnostic: checks every aspect of the CAN set-up (interface, bitrate, motor response). Same file as archive/02.../diagnostic.py. |
| [`dual_motor_simple.py`](../archive/01_single_motor_experiments/dual_motor_simple.py) | 108 | ARCHIVED | Simple dual-motor script: drives motor 1 and motor 2 simultaneously. |
| [`easing_graph.py`](../archive/01_single_motor_experiments/easing_graph.py) | 34 | ARCHIVED | Plots the easing curves (ease_in_cubic and a custom exponent) used to shape motor motion. |
| [`library_backup.py`](../archive/01_single_motor_experiments/library_backup.py) | 330 | ARCHIVED | Backup of an earlier driver revision (before the shared-bus modification). |
| [`motor_tracker.py`](../archive/01_single_motor_experiments/motor_tracker.py) | 145 | ARCHIVED | Moves one motor in proportion to a pixel error (P_GAIN degrees per pixel, REVERSE_MOTOR flag) - a target-tracking demo. |
| [`multi_motor_easing.py`](../archive/01_single_motor_experiments/multi_motor_easing.py) | 461 | ARCHIVED | Multiple motors on S-curve time-based trajectories. |
| [`single_motor.py`](../archive/01_single_motor_experiments/single_motor.py) | 232 | ARCHIVED | Single-motor position step test (MIT mode). |
| [`single_motor_backup.py`](../archive/01_single_motor_experiments/single_motor_backup.py) | 229 | ARCHIVED | Backup of single_motor.py. |
| [`single_motor_easing.py`](../archive/01_single_motor_experiments/single_motor_easing.py) | 246 | ARCHIVED | Single-motor step test with easing. |
| [`stand_up.py`](../archive/01_single_motor_experiments/stand_up.py) | 239 | ARCHIVED | Stand-up variant derived from the position step test (MIT mode). |
| [`two_motor_easing.py`](../archive/01_single_motor_experiments/two_motor_easing.py) | 268 | ARCHIVED | Two motors on S-curve time-based trajectories. |
| [`two_motor_easing_new.py`](../archive/01_single_motor_experiments/two_motor_easing_new.py) | 598 | ARCHIVED | Two-motor easing test (bug-fixed revision of two_motor_easing.py). |

## `archive/02_multi_motor_socket_control/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../archive/02_multi_motor_socket_control/ak60_v3_control.py) | 332 | ARCHIVED | Shared-bus AK60 driver revision ('MODIFIED: supports shared CAN bus for multiple motors'), the base of the multi-motor programs. Same file in 03/main_ik_runtime and 05/with_ik. |
| [`diagnostic.py`](../archive/02_multi_motor_socket_control/diagnostic.py) | 354 | ARCHIVED | CAN HAT / motor diagnostic script (same as archive/01 diagnostic.py). |
| [`display_saved_readings.py`](../archive/02_multi_motor_socket_control/display_saved_readings.py) | 57 | ARCHIVED | Reads motor_positions.txt and displays the recorded motor positions. |
| [`main_json_motor_controller_easing_multiprocess.py`](../archive/02_multi_motor_socket_control/main_json_motor_controller_easing_multiprocess.py) | 482 | ARCHIVED | Multi-motor controller, true multiprocessing: socket listener in its own process, motor control in the main process, settings from motor_config.json, S-curve easing. |
| [`main_motor_controller_easing_receiver_multiprocess.py`](../archive/02_multi_motor_socket_control/main_motor_controller_easing_receiver_multiprocess.py) | 379 | ARCHIVED | Multiprocess receiver variant with easing: listener process + motor-control process. |
| [`motor_controller_receiver.py`](../archive/02_multi_motor_socket_control/motor_controller_receiver.py) | 196 | ARCHIVED | Socket receiver: gets np.array destinations from any sender over TCP and drives motors with a 50 Hz loop and S-curve motion. |
| [`motor_controller_receiver_multiprocess.py`](../archive/02_multi_motor_socket_control/motor_controller_receiver_multiprocess.py) | 214 | ARCHIVED | Same receiver with the socket listener in a separate process (own CPU core). |
| [`motor_controller_sender.py`](../archive/02_multi_motor_socket_control/motor_controller_sender.py) | 183 | ARCHIVED | send_array_to_motor(): sends numpy arrays to the receiver via TCP (default 127.0.0.1:50000) - an early form of the socket protocol the robot still uses. |
| [`replay_saved_readings.py`](../archive/02_multi_motor_socket_control/replay_saved_readings.py) | 518 | ARCHIVED | Replays recorded positions (motor_positions.txt) on motors 1-3; includes the motor error-code table. |
| [`save_motor_readings.py`](../archive/02_multi_motor_socket_control/save_motor_readings.py) | 155 | ARCHIVED | Records motor positions to motor_positions.txt using a SoftRealtimeLoop (motors 1-3); the record half of the record/replay pair with replay_saved_readings.py. |
| [`sender_receiver_controller.py`](../archive/02_multi_motor_socket_control/sender_receiver_controller.py) | 144 | ARCHIVED | Combined sender+receiver controller built on SoftRealtimeLoop. |

Data / config files here: `motor_config.json`, `motor_positions.txt`.

## `archive/03_single_leg_ik/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`_3dof_ik_with_shift_circle_method.py`](../archive/03_single_leg_ik/_3dof_ik_with_shift_circle_method.py) | 108 | ARCHIVED | First 3-DOF leg IK ('shift circle' method): inverse_kinematics(x, y, z, L1, linkConst, L2, L3). Replaced by robot/lib/inverse_kinematics.py. |
| [`animated_movement.py`](../archive/03_single_leg_ik/animated_movement.py) | 43 | ARCHIVED | Animates the leg along a path using the IK + matplotlib (link lengths in cm). |
| [`forward_kinematics_plot.py`](../archive/03_single_leg_ik/forward_kinematics_plot.py) | 57 | ARCHIVED | Forward-kinematics plot of a leg (linkConst sign differs left/right). |
| [`stand_up.py`](../archive/03_single_leg_ik/stand_up.py) | 234 | ARCHIVED | Stand-up sequence: IK + S-curve timing, sends arrays over the socket to the motor controller. |
| [`visualization_functions.py`](../archive/03_single_leg_ik/visualization_functions.py) | 266 | ARCHIVED | 3D matplotlib helpers (animate_3dof) to draw the leg links. |

## `archive/03_single_leg_ik/main_ik_runtime/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`_3dof_ik_with_shift_circle_method.py`](../archive/03_single_leg_ik/main_ik_runtime/_3dof_ik_with_shift_circle_method.py) | 105 | ARCHIVED | IK copy used by this runtime snapshot. |
| [`ak60_v3_control.py`](../archive/03_single_leg_ik/main_ik_runtime/ak60_v3_control.py) | 330 | ARCHIVED | Driver copy (shared-bus revision). |
| [`main_motor.py`](../archive/03_single_leg_ik/main_ik_runtime/main_motor.py) | 333 | ARCHIVED | Multi-motor controller with socket listener process + IK for ONE leg. |
| [`stand_up.py`](../archive/03_single_leg_ik/main_ik_runtime/stand_up.py) | 186 | ARCHIVED | Stand-up sender for one leg (leg_side = 'left'/'right'). |
| [`stand_up_animation.py`](../archive/03_single_leg_ik/main_ik_runtime/stand_up_animation.py) | 82 | ARCHIVED | get_position(t): smooth trajectory used by the stand-up. |
| [`visualization_functions.py`](../archive/03_single_leg_ik/main_ik_runtime/visualization_functions.py) | 267 | ARCHIVED | Plot helpers (copy). |

Data / config files here: `motor_config.json`.

## `archive/04_imu_balancing/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../archive/04_imu_balancing/ak60_v3_control.py) | 238 | ARCHIVED | Byte-identical copy of robot/lib/ak60_v3_control.py as it was before documentation (the same driver also lives in archive/04_imu_balancing, cur_test/ and BYTE_VISION/). Documentation lives in the robot/lib version. |
| [`can_runtime.py`](../archive/04_imu_balancing/can_runtime.py) | 388 | ARCHIVED | Flat copy of the bus runtime (see robot/lib/can_runtime.py). |
| [`homing_controller.py`](../archive/04_imu_balancing/homing_controller.py) | 519 | ARCHIVED | Flat copy of the homing controller (see robot/lib/homing_controller.py). |
| [`ik_sender.py`](../archive/04_imu_balancing/ik_sender.py) | 56 | ARCHIVED | Single-coordinate test sender. |
| [`imu_balance_test.py`](../archive/04_imu_balancing/imu_balance_test.py) | 162 | ARCHIVED | IMU balance experiment: reads the MPU-6050 and sends corrected foot positions around a base stand pose (X_NOM = -9.094) to 127.0.0.1:50000. |
| [`imu_balance_test2.py`](../archive/04_imu_balancing/imu_balance_test2.py) | 272 | ARCHIVED | Second iteration of the IMU balance controller: PD tuning, filter, deadzone, limits and calibration blocks at the top. |
| [`imu_ik_test.py`](../archive/04_imu_balancing/imu_ik_test.py) | 184 | ARCHIVED | IMU-stabilised weight-shift test: shifts the body (e.g. right / backward, away from FL) while an IMU feedback loop keeps it stable; has stability and step-size tuning blocks. |
| [`inverse_kinematics.py`](../archive/04_imu_balancing/inverse_kinematics.py) | 203 | ARCHIVED | Calibration-table IK (LEG_CALIBRATION), an earlier design than robot/lib/inverse_kinematics.py. |
| [`level_stand.py`](../archive/04_imu_balancing/level_stand.py) | 171 | ARCHIVED | Levelling-stance maths (calculate_leveled_stance) and smooth transitions (40 Hz, 120 deg/s). INCOMPLETE: the MPU hardware read (read_raw_mpu_hardware) is a TODO stub, so this script cannot run end-to-end. |
| [`main_sender.py`](../archive/04_imu_balancing/main_sender.py) | 68 | ARCHIVED | Sends the initial standing pose. |
| [`main_with_current_temp.py`](../archive/04_imu_balancing/main_with_current_temp.py) | 839 | ARCHIVED | Runtime with current/temperature logging, joint-angle payloads. Logs to an ABSOLUTE path inside cur_test/. |
| [`main_with_ik.py`](../archive/04_imu_balancing/main_with_ik.py) | 903 | ARCHIVED | Runtime with IK (earlier snapshot of robot/main_with_ik.py). Logs to an ABSOLUTE path inside cur_test/. |
| [`single_ik.py`](../archive/04_imu_balancing/single_ik.py) | 166 | ARCHIVED | Interactive per-leg coordinate sender (leg_commander). |
| [`swing_balance.py`](../archive/04_imu_balancing/swing_balance.py) | 128 | ARCHIVED | Swing-balance test: stand up, capture an IMU reference, shift weight to one side, then run a low-gain balancing loop (low-pass filter + deadzone). |

Data / config files here: `motor_config.json`, `motor_currents.csv`, `motor_temps.csv`.

## `archive/05_four_leg_bringup/final/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`ak60_v3_control.py`](../archive/05_four_leg_bringup/final/ak60_v3_control.py) | 342 | ARCHIVED | Driver revision used by the first complete 4-leg program. |
| [`pre_and_post_homing.py`](../archive/05_four_leg_bringup/final/pre_and_post_homing.py) | 557 | ARCHIVED | First complete 4-leg program: sequential homing of 12 motors on 2 CAN buses in 2 parallel phases (FR+BL, then BR+FL) followed by post-homing socket control. The ancestor of robot/main_with_ik.py. |
| [`send_targets.py`](../archive/05_four_leg_bringup/final/send_targets.py) | 174 | ARCHIVED | Sender library: converts logical joint-frame angles to raw motor angles using gear_ratio and flipped from motor_config.json. |

Data / config files here: `motor_config.json`.

## `archive/05_four_leg_bringup/with_ik/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`_3dof_ik_with_shift_circle_method.py`](../archive/05_four_leg_bringup/with_ik/_3dof_ik_with_shift_circle_method.py) | 136 | ARCHIVED | 4-leg IK (documents the Y-up / XZ-ground coordinate convention). |
| [`_3dof_ik_with_shift_circle_method_old.py`](../archive/05_four_leg_bringup/with_ik/_3dof_ik_with_shift_circle_method_old.py) | 106 | ARCHIVED | Previous IK revision. |
| [`ak60_v3_control.py`](../archive/05_four_leg_bringup/with_ik/ak60_v3_control.py) | 330 | ARCHIVED | Driver copy (shared-bus revision). |
| [`main_motor.py`](../archive/05_four_leg_bringup/with_ik/main_motor.py) | 378 | ARCHIVED | 4-leg multi-motor controller with socket listener process and IK. |
| [`main_motor_old.py`](../archive/05_four_leg_bringup/with_ik/main_motor_old.py) | 374 | ARCHIVED | Previous revision of main_motor.py. |
| [`stand_up.py`](../archive/05_four_leg_bringup/with_ik/stand_up.py) | 431 | ARCHIVED | Stand-up sequence plus an interactive per-leg tester; sends joint angle commands to main_motor.py over TCP. |
| [`stand_up_animation.py`](../archive/05_four_leg_bringup/with_ik/stand_up_animation.py) | 83 | ARCHIVED | Trajectory helper get_position(t, leg, ...) for the stand-up. |
| [`stand_up_old.py`](../archive/05_four_leg_bringup/with_ik/stand_up_old.py) | 214 | ARCHIVED | Previous revision of stand_up.py. |
| [`visualization_functions.py`](../archive/05_four_leg_bringup/with_ik/visualization_functions.py) | 267 | ARCHIVED | Plot helpers (copy). |

Data / config files here: `motor_config.json`.

## `archive/05_four_leg_bringup/controller/`

| File | Lines | Status | What it does |
|---|--:|---|---|
| [`controller_test.py`](../archive/05_four_leg_bringup/controller/controller_test.py) | 40 | ARCHIVED | Prints all inputs of the first detected gamepad (buttons and sticks). Ctrl+C to exit. |

---

**Duplicates to be aware of** (before documentation these were byte-identical):
the AK60 driver in `robot/lib/`, `cur_test/`, `cur_test/BYTE_VISION/` and
`archive/04_imu_balancing/`; the shared-bus driver revision in `archive/02…`,
`archive/03…/main_ik_runtime/` and `archive/05…/with_ik/`. `robot/` is the documented
source of truth.
