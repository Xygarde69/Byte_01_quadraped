# `robot/` — the active BYTE-01 system

Everything needed to run the robot. **Always run commands from this directory** —
imports (`from lib…`, `from config…`) and file paths (`config/…`, `logs/…`) are relative to it.

| Path | What it is |
|---|---|
| `main_with_ik.py` | **Motor runtime.** Homing → sit/zero → 100 Hz live control from foot-position packets. Start this first. |
| `state_manager_server_v2.py` | **HTTP API** (port 8000) turning `SIT/STAND/FORWARD/…/CW/CCW` into gait packets. Current version. |
| `keyboard_controller.py` | Terminal keyboard client (WASD) — *currently incompatible with v2, see `docs/KNOWN_ISSUES.md` F1*. |
| `test_client.py` | Minimal interactive HTTP client. |
| `mpu_leveling.py` | One-shot MPU-6050 chassis levelling (used by `state_manager_server_v3.py`). |
| `state_manager_server.py` / `_v3.py` | v1 (no rotation) and an MPU-levelling variant — kept for reference. |
| `lib/ak60_v3_control.py` | CAN driver: MIT-mode frames, status decoding. |
| `lib/can_runtime.py` | Per-bus runtime: shared motor state, feedback listener, watchdogs, fault latch. |
| `lib/homing_controller.py` | Hard-stop homing algorithm and tuning. |
| `lib/inverse_kinematics.py` | Foot position → joint/motor angles. |
| `lib/trot_gait.py` | Trot and in-place rotation foot-trajectory generators. |
| `lib/trot_gait_lib.py` | Unused stateless trot engine (design reference). |
| `config/robot_config.py` | Network address, leg order, `SIT_COORDS`, `SIT_TARGETS_DEG`, default speed. |
| `config/motor_config.json` | Per-motor `kp`, `kd`, `flipped`, `gear_ratio`. |
| `logs/` | `motor_currents.csv`, `motor_temps.csv` (overwritten each run). |
| `tests/` | Bench tools: run as `python3 -m tests.ik_sender` etc. |
| `requirements.txt` | Python dependencies with tested versions. |

Start-up, step by step: [`../docs/RUNBOOK.md`](../docs/RUNBOOK.md) ·
Design: [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md).
