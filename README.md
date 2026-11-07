# BYTE-01 — Quadruped Robot Control Software

Control software for **BYTE-01**, a 12-motor quadruped robot built on
**CubeMars AK60-6 V3.0** actuators, driven from a **Raspberry Pi** over two
**CAN buses**.  It covers everything between "a foot should be *here*" and
"twelve motors receive a CAN frame": hard-stop homing, inverse kinematics, gait
generation, an HTTP command API, IMU levelling, and safety watchdogs.

```
        ┌─────────────┐  HTTP :8000   ┌──────────────────────┐  TCP :50000   ┌─────────────────┐  CAN  ┌──────────┐
 you ──▶│ keyboard /  │──────────────▶│ state_manager_server │──────────────▶│  main_with_ik   │──────▶│ 12 × AK60 │
        │ test client │  "FORWARD"    │ (gaits, state machine)│ foot offsets │ (IK, limits, MIT)│ 1 Mb/s│ motors    │
        └─────────────┘               └──────────────────────┘   (pickle)    └─────────────────┘       └──────────┘
```

> **Status:** active research robot. The clean, documented runtime is in
> [`robot/`](robot/). Everything else is either an experiment workspace or
> history. Read [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md) before running
> anything — it contains two security items and several safety notes.

---

## Where do I start?

| I want to… | Go to |
|---|---|
| Understand how the whole thing fits together | [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) |
| Power it up and make it walk | [`docs/RUNBOOK.md`](docs/RUNBOOK.md) |
| Wire / set up a fresh Pi | [`docs/HARDWARE_SETUP.md`](docs/HARDWARE_SETUP.md) |
| Find what a specific file does | [`docs/CODE_MAP.md`](docs/CODE_MAP.md) |
| See why the code looks the way it does | [`docs/PROJECT_HISTORY.md`](docs/PROJECT_HISTORY.md) |
| Know what is broken or risky | [`docs/KNOWN_ISSUES.md`](docs/KNOWN_ISSUES.md) |

## Repository layout

```
ak60_motor_control/
├── robot/                  ★ THE ACTIVE SYSTEM (run everything from here)
│   ├── main_with_ik.py         motor runtime: homing → live IK control
│   ├── state_manager_server_v2.py   HTTP API + gaits (FastAPI, port 8000)
│   ├── keyboard_controller.py  terminal keyboard client      (needs a fix, see issues)
│   ├── test_client.py          tiny interactive HTTP client
│   ├── mpu_leveling.py         one-shot IMU chassis levelling
│   ├── lib/                    driver, bus runtime, homing, IK, gaits
│   ├── config/                 robot_config.py + motor_config.json
│   ├── logs/                   motor current / temperature CSVs (overwritten each run)
│   └── tests/                  bench tools — run as `python3 -m tests.<name>`
│
├── multiprocess_code/all_4_legs/cur_test/
│                           ⚙ EXPERIMENT WORKSPACE (gaits, gamepad, IMU, voice, vision)
│                             kept in place: it has absolute paths and a virtualenv
├── sensors/mpu_6050/       MPU-6050 IMU bench scripts
├── system/                 CAN bring-up script + systemd unit
├── archive/                project history, oldest → newest (01 … 05)
└── docs/                   all documentation
```

## 60-second quick start

On the Pi, motors powered, from the repository root:

```bash
ip -br link show type can           # can0 and can1 must both be UP (see docs/HARDWARE_SETUP.md)

cd robot
python3 main_with_ik.py y           # terminal 1 — home all legs, sit, listen   (n = skip homing)
python3 state_manager_server_v2.py  # terminal 2 — HTTP API on :8000 (after "Socket receiver process started")
python3 test_client.py              # terminal 3 — type: stand, forward, cw, sit …
```

⚠ `Ctrl-C` or any fault makes every motor go **limp** — keep the robot supported.

## Hardware & software at a glance

| | |
|---|---|
| Actuators | 12 × CubeMars AK60-6 V3.0, MIT mode, 1 Mbit/s CAN, extended IDs |
| Layout | 4 legs × 3 joints (hip, thigh, knee); knee has a 1.5× gear ratio |
| Buses | `can0`: motors 1-6 (FL, BL) · `can1`: motors 7-12 (FR, BR) |
| Computer | Raspberry Pi, Ubuntu 22.04 (aarch64), Python 3.10, 2 × MCP2515 CAN |
| Sensor | MPU-6050 IMU on I²C bus 1, address `0x68` |
| Link lengths | hip offset 5.995 cm · lateral 9.094 cm · thigh 22.0 cm · shank 21.5 cm |

## Conventions used throughout the code

* **Legs:** `fl` front-left, `bl` back-left, `fr` front-right, `br` back-right.
  Motor IDs per leg are (hip, thigh, knee): fl 1-3 · bl 4-6 · fr 7-9 · br 10-12.
* **Foot coordinates:** centimetres; **+X forward, +Y right, +Z up** (feet
  below the hip have negative Z). Packets sent to the runtime are **offsets from
  the sitting pose** (`SIT_COORDS`), so `0,0,0` = sit and `dz = −14` ≈ stand.
* **Angles:** degrees in software, radians only on the CAN wire.
* **Motor zero:** the motors are re-zeroed *at the sitting pose* every start-up,
  so "motor angle 0" always means "sitting".

---

*Documentation generated and code comments added during the repository
clean-up; logic was verified unchanged by comparing syntax trees before and
after (see the note at the end of `docs/PROJECT_HISTORY.md`).*
