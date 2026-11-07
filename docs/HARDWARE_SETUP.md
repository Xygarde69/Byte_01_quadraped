# Hardware & Pi Setup

What the software expects, and how this particular Raspberry Pi is configured
(read from the machine on 2026-11-07). Use it to rebuild the Pi or diagnose a
"it worked yesterday" problem.

## 1. Bill of materials (as far as the repository shows)

| Part | Detail |
|---|---|
| Computer | Raspberry Pi 4 Model B Rev 1.5 (4 cores, ≈7.6 GiB RAM) |
| OS | Ubuntu 22.04.5 LTS, aarch64, kernel 5.15 (`-raspi`), Python 3.10.12 |
| CAN interface | Waveshare CAN HAT with **two MCP2515 channels** → `can0`, `can1` |
| Actuators | 12 × **CubeMars AK60-6 V3.0**, MIT mode, 1 Mbit/s, 29-bit extended IDs |
| IMU | MPU-6050 on I²C bus 1, address `0x68` (AD0 low) |
| Optional | Intel RealSense (vision), USB microphone (voice), a gamepad ("Red Gear" per `controller_test.py`) |

## 2. Motor and bus map

```
                         FRONT
        ┌───────────────────┬───────────────────┐
        │  FL   (can0)      │  FR   (can1)      │
        │  1 hip  2 thigh   │  7 hip  8 thigh   │
        │  3 knee           │  9 knee           │
        ├───────────────────┼───────────────────┤
        │  BL   (can0)      │  BR   (can1)      │
        │  4 hip  5 thigh   │ 10 hip 11 thigh   │
        │  6 knee           │ 12 knee           │
        └───────────────────┴───────────────────┘
                         BACK
        (left legs on the left, as seen from behind the robot looking forward)
```

| Leg | Motor IDs (hip, thigh, knee) | Bus |
|---|---|---|
| FL | 1, 2, 3 | `can0` |
| BL | 4, 5, 6 | `can0` |
| FR | 7, 8, 9 | `can1` |
| BR | 10, 11, 12 | `can1` |

* **Each motor's CAN ID must be set to its number (1–12)** with CubeMars' configuration
  software, and every motor must be set to **broadcast its status frame periodically** —
  the runtime never polls; it waits for those frames (`wait_for_periodic_feedback`).
  "startup timeout waiting for motors […]" usually means one of these two settings,
  or wiring/power.
* The knee motors (3, 6, 9, 12) have an additional 1.5× reduction, expressed as
  `gear_ratio: 1.5` in `robot/config/motor_config.json`.

## 3. Enabling CAN on the Pi

`/boot/firmware/config.txt` on this Pi contains:

```ini
dtparam=spi=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=23
dtoverlay=mcp2515-can1,oscillator=16000000,interrupt=25
```

(16 MHz crystal; interrupt GPIOs 23 and 25 — as configured on this Pi.) A reboot is
needed after changing them. Afterwards `ip link` should list
`can0` and `can1`.

### Bringing the interfaces up (1 Mbit/s)

[`system/can-setup.sh`](../system/can-setup.sh) sets both channels to
`bitrate 1000000`, `txqueuelen 1000`, and brings them up. Run it by hand:

```bash
sudo bash system/can-setup.sh
```

or install it to run at every boot (this is how the Pi is currently set up — the
service is `enabled` and `active`):

```bash
sudo install -m 755 system/can-setup.sh /usr/local/bin/can-setup.sh
sudo cp system/can-interface.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now can-interface.service
```

Check:

```bash
ip -details link show can0 | grep -E "state|bitrate"
ip -br link show type can           # both UP
candump can0                        # (can-utils) live status frames, ID 0x29xx
```

> Note: the repo's copy of `can-setup.sh` used to configure only `can0`; it was
> synced with the deployed version during the clean-up.
> The software itself passes `ignore_config=True` to python-can, so it never tries to
> reconfigure the interfaces — they must be up *before* `main_with_ik.py` starts.

## 4. I²C / IMU

```bash
ls /dev/i2c-1            # present = I²C enabled
sudo apt install i2c-tools && i2cdetect -y 1     # expect 0x68
```

The MPU-6050 should be mounted flat, chip facing up, X axis forward. The sign and axis
constants (`FORWARD_AXIS`, `FORWARD_SIGN`, …) at the top of `robot/mpu_leveling.py` are
how a different mounting is accommodated; run `python3 mpu_leveling.py` and tilt the
robot to verify (instructions at the bottom of that file).
`sensors/mpu_6050/calibration.py` holds measured accelerometer/gyro bias values.

## 5. Python environment

System Python 3.10. Versions below are what is installed on the robot's Pi and known
to work (also in [`robot/requirements.txt`](../robot/requirements.txt)):

| Package | Version | Used by |
|---|---|---|
| python-can | 4.6.1 | CAN driver and runtime |
| numpy | 2.2.6 | imported by IK (not actually used) |
| fastapi / uvicorn / pydantic | 0.136.1 / 0.46.0 / 2.13.3 | state server |
| requests | 2.25.1 | `test_client.py`, `keyboard_controller.py` |
| keyboard | 0.13.5 | `keyboard_controller.py` (needs root) |
| smbus2 | 0.6.0 | MPU-6050 |

Experiment extras (not needed by `robot/`): `pygame 2.6.1` (gamepad),
`matplotlib 3.10.8`, `pandas` (current plotter), `ultralytics 8.4.33`,
`pyrealsense2`, `opencv-python`, `faster-whisper 1.2.1`, `onnxruntime`, `sounddevice`;
the voice assistant has its own 5.7 GB virtualenv (`BYTE_VA/byte_env`) plus
`BYTE_VA/requirements.txt`.

Install the core set:

```bash
python3 -m pip install -r robot/requirements.txt
```

## 6. Network

The runtime listens on `SOCKET_HOST:50000` (raw motor protocol) and the state
server on `0.0.0.0:8000` (HTTP). The Pi gets its Wi-Fi address by DHCP, so
`SOCKET_HOST` in `robot/config/robot_config.py` must be updated whenever the network
changes (KNOWN_ISSUES F8). Open ports are unauthenticated (KNOWN_ISSUES S2).

## 7. Not documented anywhere in the repository — please fill in

These are things only the builders know. Adding them here will save the next person a
lot of time:

- [ ] Battery / supply voltage and the motor power wiring & fusing
- [ ] CAN wiring diagram, where the 120 Ω termination resistors are
- [ ] How each leg is physically mounted (hip axis direction) — explains the sign
      conventions in `SIT_TARGETS_DEG`
- [ ] Mechanical hard-stop positions that homing relies on (and their safe limits)
- [ ] Which Waveshare HAT model exactly, and its jumper settings
- [ ] Procedure used to measure `SIT_COORDS` ("manually calculated" — commit `ae6da569`)
