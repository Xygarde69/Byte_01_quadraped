# Project History

How the code got to where it is. The `archive/` folders are numbered in the
order the ideas were built, so reading them 01 → 05 replays the project.
Dates come from `git log` (68 commits, 8 Jan – 5 May 2026).

## Timeline

| When | What happened | Where the code is now |
|---|---|---|
| **8–12 Jan** | First motors spin. Single-motor position steps, easing curves, then several motors; *multiprocessing* introduced (socket receiver in its own process); per-motor kp/kd read from JSON. | [`archive/01_…`](../archive/01_single_motor_experiments/), [`archive/02_…`](../archive/02_multi_motor_socket_control/) |
| **17–26 Jan** | Load tests ("working with high load", "lifted successfully"); record / replay of motor positions; error + temperature logging; time-based coordinates. | `archive/02_…` (`save_/replay_/display_saved_readings.py`) |
| **3–10 Mar** | 3-DOF leg inverse kinematics ("shift-circle" method), stand-up animation, all 12 motors held, then each side's legs, "working (air)". | [`archive/03_…`](../archive/03_single_leg_ik/), [`archive/05_…/with_ik`](../archive/05_four_leg_bringup/with_ik/) |
| **13–18 Mar** | **Homing by current feedback** designed and tested; MPU-6050 verified; forward-kinematics plots; "post homing control by socket"; all four legs homed. | [`archive/05_…/final`](../archive/05_four_leg_bringup/final/), `lib/homing_controller.py` ancestors |
| **22–28 Mar** | Speed limit scaled by gear ratio; gait testing begins. | [`cur_test/`](../multiprocess_code/all_4_legs/cur_test/) |
| **18–28 Apr** | Leg commander and swing-balance scripts; first trot that "walked perfectly" (`trot_gati_node…`); IMU balance experiments. | `cur_test/`, [`archive/04_…`](../archive/04_imu_balancing/) |
| **1–3 May** | **`new_reference` rewrite**: shared config (`robot_config.py`), new analytic IK with a sit-pose reference, motor-polarity tests, manually measured `SIT_COORDS`, a clean homing → nudge → sit → zero → IK start-up, driven by a *State Manager*. | [`robot/`](../robot/) |
| **4–5 May** | Walking through the state manager; keyboard controller; MPU leveling after standing; "working good" (latest commit). | `robot/` |
| **7 Nov** | This clean-up: restructure, documentation, code comments. No logic changed. | — |

## Design decisions visible in the code

*The "why" below is inferred from the code and commit messages, not from the
authors — correct it where it is wrong.*

1. **Socket between "what to do" and "how to move"** — started as a convenience so
   test scripts could drive the motor program from another terminal, then became the
   architecture: every gait, controller and experiment is just another client.
2. **Separate process for the socket** — added during "MultiProcessing Implemented"
   so network handling can't jitter the control loop.
3. **Motor-side spring-damper (MIT mode) instead of torque/velocity control** — gives
   compliant legs and lets the Pi send only *positions* at 100 Hz.
4. **Re-zero at the sit pose** — absolute homing finds hard stops once per boot;
   zeroing at the sit pose afterwards lets all later maths be "relative to sitting".
5. **Latch faults and go limp** — (`FaultCode`, watchdogs) apparently because a
   stale CAN link on a spring-loaded joint is more dangerous than a collapse.
6. **Copy-the-folder workflow** — experiments were duplicated rather than branched
   (`main_with_ik.py` exists in three places). That is why the archive is large and why
   `docs/CODE_MAP.md` marks duplicates explicitly.

## Where did my old files go?

Every move was done with `git mv`, so `git log --follow <new path>` shows the full
history.

| Old path | New path |
|---|---|
| `multiprocess_code/all_4_legs/new_reference/` | `robot/` |
| `multiprocess_code/all_4_legs/cur_test/` | *(unchanged — see below)* |
| `mpu_6050/` | `sensors/mpu_6050/` |
| `can-setup.sh`, `can-interface.service` | `system/` |
| `extra_old_files/` | `archive/01_single_motor_experiments/` |
| `multiprocess_code/*.py`, `motor_config.json`, `motor_positions.txt` | `archive/02_multi_motor_socket_control/` |
| `multiprocess_code/ik/` | `archive/03_single_leg_ik/` |
| `multiprocess_code/main_ik/` | `archive/03_single_leg_ik/main_ik_runtime/` |
| `multiprocess_code/imu_balancing/` | `archive/04_imu_balancing/` |
| `multiprocess_code/all_4_legs/with_ik/` | `archive/05_four_leg_bringup/with_ik/` |
| `multiprocess_code/all_4_legs/final/` | `archive/05_four_leg_bringup/final/` |
| `multiprocess_code/all_4_legs/controller/` | `archive/05_four_leg_bringup/controller/` |

**Why `cur_test/` did not move:** the current-logger writes to, and
`motor_current_plotter.py` reads from, an *absolute* path inside it
(`/home/byte/ak60_motor_control/multiprocess_code/all_4_legs/cur_test/…`), the
voice assistant's `byte_env/` virtualenv (5.7 GB) has the absolute path baked into
its `activate` script and every `bin/` shebang, and the folder holds uncommitted work. Moving it is a good follow-up once those are
parameterised (see KNOWN_ISSUES H-items).

## How the clean-up was verified

* Backup: `tar` of all source before the first change.
* Moves: `git mv` only — no file was copied-and-deleted.
* Comments: after every edit, a script compared the **abstract syntax tree** of every
  `.py` file (docstrings removed) against a snapshot taken after the moves.
  Final result: **146 files checked, 0 with changed logic.**
  The only non-comment change in the whole clean-up is `system/can-setup.sh`, which
  was synced with the version actually deployed in `/usr/local/bin/` (the repo copy
  only configured `can0`).
* Numerical checks run while documenting: `IK(SIT_COORDS) − sit reference = 0` for
  all four legs; `FK(IK(p)) − p < 1e-13 cm`.
* **Not** done: nothing was run against the motors. The robot software was
  compiled/parsed and the IK exercised in isolation; hardware behaviour was not
  re-tested.
