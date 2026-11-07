# Runbook — operating BYTE-01

Step-by-step operation, plus a troubleshooting table keyed to the **actual
messages** the software prints. For one-time Pi/CAN setup see
[`HARDWARE_SETUP.md`](HARDWARE_SETUP.md); for risks see
[`KNOWN_ISSUES.md`](KNOWN_ISSUES.md).

> ⚠ **Read first.** Stopping the program, any fault, or one unreachable target makes
> every motor **go limp**. Have the robot **sitting on its chassis** or on a stand
> before you start and before you press Ctrl-C.
> The runtime accepts unauthenticated network commands — use a trusted network.

## 1. Pre-flight checklist

- [ ] Robot sitting on its belly (or lifted on a stand for first tests).
- [ ] Motor power on (all 12 motors powered, battery charged).
- [ ] `ip -br link show type can` shows **can0** and **can1** `UP`.
- [ ] `SOCKET_HOST` in `robot/config/robot_config.py` is an address this Pi
      currently has — `ip -4 addr` — or `127.0.0.1` if everything runs on the Pi.
- [ ] You are in `robot/` (every script uses paths relative to it).
- [ ] Someone can reach the power switch.

## 2. Normal start-up

Open three terminals on the Pi (or use `tmux`).

**Terminal 1 — motor runtime**
```bash
cd ~/ak60_motor_control/robot
python3 main_with_ik.py y        # y = run homing first (after any power cycle)
```
What you will see, in order: waiting for feedback from all 12 motors → 1 s soft
hold → `Phase 1` (front-right + back-left legs search for their hard stops) →
`Phase 2` (back-right + front-left) → sit-down → "Motors successfully zeroed" →
`Socket receiver process started`. Homing takes a while — each joint creeps
into its stop slowly on purpose. Wait for the receiver line before continuing.

**Terminal 2 — behaviour server**
```bash
cd ~/ak60_motor_control/robot
python3 state_manager_server_v2.py       # HTTP API on :8000
```

**Terminal 3 — a client**
```bash
cd ~/ak60_motor_control/robot
python3 test_client.py
Enter state > stand
Enter state > forward        # 3 trot cycles
Enter state > cw             # turn in place
Enter state > sit
```
Or call the API directly:
```bash
curl -X POST http://localhost:8000/change-state \
     -H 'Content-Type: application/json' \
     -d '{"new_state":"STAND","sender_id":"me","cycles":1}'
```

States: `SIT, STAND, FORWARD, BACKWARD, LEFT, RIGHT, CW, CCW` (`JUMP` is accepted but
does nothing). One request at a time; a second one while the robot is moving gets **429**.

### Restarting without re-homing
`python3 main_with_ik.py n` skips homing. **Only** use it if the motors have *not*
lost power since the last run and the robot is sitting (see KNOWN_ISSUES R3). After
any power cycle, always use `y`.

## 3. Normal shutdown
1. Send `sit` and wait ~2 s for the robot to settle.
2. `Ctrl-C` in terminal 1 (motors go limp — now safe because it is sitting).
3. `Ctrl-C` the server and client.

## 4. Bench tools (no walking)

| Task | Command (from `robot/`) |
|---|---|
| Check the IK maths (no hardware) | `python3 -m lib.inverse_kinematics` |
| Move one leg's foot by a few cm | edit `dx,dy,dz` in `tests/ik_sender.py`, then `python3 -m tests.ik_sender` |
| Move a single motor to a raw angle | terminal A: `python3 -m tests.joint_tester` · terminal B: `python3 -m tests.joint_sender` — *instead of* `main_with_ik.py`, never together |
| Check IMU axes / signs | `python3 mpu_leveling.py` (tilt the robot, watch pitch/roll signs) |
| Plot motor currents of the last run | `python3 ../multiprocess_code/all_4_legs/cur_test/motor_current_plotter.py` (reads the *cur_test* CSV) |
| Enable IMU levelling in the server | switch to `state_manager_server_v3.py` (no rotation states) |

## 5. Telemetry
`robot/logs/motor_currents.csv` and `motor_temps.csv` — 5 Hz, one row per sample,
columns `timestamp_s, m1…m12`. **Overwritten at every start.** Copy them if you need
them. Temperature is the value each motor reports, in °C.

## 6. Troubleshooting

| You see | Meaning | What to do |
|---|---|---|
| `ModuleNotFoundError: No module named 'config'` (or `lib`) | wrong working directory | `cd` to `robot/`; for `tests/` use `python3 -m tests.<name>` |
| `OSError: [Errno 99] Cannot assign requested address` (socket process) | `SOCKET_HOST` isn't an address of this machine (DHCP changed it) | `ip -4 addr`; update `config/robot_config.py` or use `127.0.0.1` |
| `startup timeout waiting for motors [..]` | those motors never sent status frames | motor power; CAN cable/termination; `can0/can1` up at 1 Mbit/s; motor CAN IDs set to 1-12 |
| `[can0] M5: stale feedback age 0.2s` / `no RX for …s` | link dropped mid-run (watchdog latched) | check cable & power; restart with `y` |
| `motor error code N` | the motor reports a fault: 1 over-temp, 2 over-current, 3 over-volt, 4 under-volt, 5 encoder, 6 MOSFET temp, 7 locked | let it cool / check supply; power-cycle the motor |
| `search travel exceeded 220.0°` | homing never found the hard stop | check the leg moves freely toward the stop and `trigger_current_A` in `CAN_CONFIG`; check wiring/polarity |
| Homing presses against a stop but never zeroes | the contact is not reaching `trigger_current_A` (or the joint keeps moving) | tune that motor's `trigger_current_A` in `CAN_CONFIG` (`main_with_ik.py`); watch the `fcur=` value in the log lines |
| `Reachability Error: Target is physically out of bounds` then everything stops | a client asked for an unreachable foot position (KNOWN_ISSUES R2) | sit the robot, restart, and reduce the stride / check the packet |
| `Socket Error: …` from the state server | runtime isn't running or wrong host/port | start `main_with_ik.py`; compare `SOCKET_HOST/PORT` |
| HTTP 429 "Robot is busy" | previous request still executing (no queue) | wait; avoid repeated requests |
| HTTP 422 from the keyboard controller | wrong request body (KNOWN_ISSUES F1) | see the fix in F1 |
| `Connection refused` from any sender | runtime not listening yet | wait for "Socket receiver process started" |
| One CAN bus works, the other doesn't | e.g. only `can0` configured | `sudo bash system/can-setup.sh` (sets up both); `ip -d link show can1` |
| Robot sags/oscillates in a joint | kp/kd for that motor | `config/motor_config.json` (current values: kp 300-400, kd 1.5) |
| Body tilted when standing | possible causes: `SIT_COORDS` vs real pose mismatch, homing offset | re-measure `SIT_COORDS`/`SIT_TARGETS_DEG`; try IMU levelling (`mpu_leveling.py`) |
| `OSError: [Errno 121] Remote I/O error` when starting `mpu_leveling.py` / `state_manager_server_v3.py` | the MPU-6050 doesn't answer at I²C 0x68 | check wiring/power; `i2cdetect -y 1`; use `state_manager_server_v2.py`, which doesn't need the IMU |
| `keyboard` library: "you must be root" | the `keyboard` package needs root on Linux | `sudo python3 keyboard_controller.py` |
