# `cur_test/` — experiment workspace

The folder where new ideas are tried before they are (maybe) merged into
[`robot/`](../../../robot/). It is a **flat** collection of scripts that import each
other from the same directory (`from can_runtime import …`), so it can drift from
`robot/` — each file starts with a `REPO MAP` banner stating what it is and how to run it.

**Why it is still at this long path:** its loggers and plotter use absolute paths into
this folder, and `BYTE_VA/byte_env/` (a 5.7 GB virtualenv) cannot be relocated. See
`docs/KNOWN_ISSUES.md` H4 before moving it.

| Area | Files |
|---|---|
| Runtime copies | `main_with_ik.py`, `main_with_current_temp.py`, `can_runtime.py`, `homing_controller.py`, `ak60_v3_control.py`, `inverse_kinematics.py` (older/different variants of what is in `robot/`) |
| Gaits (interactive) | `sidd_gait.py` (trot), `sidd_crawl_gait.py`, `pace_gait_sidd.py`, `bound_gait_sidd.py`, `trot_gait_node_sid.py`, `daniel_ik_gait_promaxultra.py` |
| Gamepad | `kriss_trot_controller.py` (+ `controller_test.py`) |
| IMU | `imu_reader.py`, `imu_correction.py` *(new)*, `mpu_leveling.py`, `byte_balance.py` |
| Bench / test | `kriss_exp.py`, `ik_sender.py`, `single_ik.py`, `main_sender.py`, `motor_current_plotter.py` |
| `extra_files/` | Older gait / calibration / pre-test scripts |
| `BYTE_VA/` | Voice assistant: wake word → speech-to-text → "sit"/"stand" |
| `BYTE_VISION/` | RealSense vision: face recognition, YOLO detection, Gemini agent |

`extra_files/` scripts import modules that live one level up (`can_runtime`,
`homing_controller`, `imu_reader`, `main_sender`) — they were meant to be copied next to
those files (or run with `PYTHONPATH=..`).

⚠ `BYTE_VISION/integrated.py` contains a committed API key — see `docs/KNOWN_ISSUES.md` S1.
⚠ The voice scripts use the **old** coordinate protocol — see R5.
