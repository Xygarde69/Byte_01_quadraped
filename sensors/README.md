# `sensors/`

## `mpu_6050/` — IMU bench scripts (stand-alone, no robot needed)

| File | Purpose |
|---|---|
| `raw_check.py` | Smallest test: wake the MPU-6050 on I²C bus 1 (0x68) and print raw values. |
| `get_roll_pitch.py` | Roll / pitch from the accelerometer after a reference capture. |
| `check_leg_lifts.py` | Turns posture into a 0–10 "lift level" per leg. |
| `aarush_imu.py` | Interactive tool: reference, max-tilt capture (`imu_max_angles.json`), headless plot. |
| `calibration.py` / `imu_max_angles.json` | Measured sensor bias and the recorded tilt extremes (data). |

The IMU code used by the robot itself is `robot/mpu_leveling.py`.
