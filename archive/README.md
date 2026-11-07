# `archive/` — project history, oldest → newest

Nothing here is used by `robot/`. It is kept because it records *how* each idea was
reached and contains working reference code. Each numbered folder is self-contained
(its own copy of the driver, so scripts still run from inside their folder).
Every file carries a header comment saying what it is.

| Folder | Period | What it contains |
|---|---|---|
| `01_single_motor_experiments/` | Jan 2026 | First single/dual-motor tests: position steps, easing curves, S-curve trajectories, CAN diagnostic, a pixel-tracking demo. |
| `02_multi_motor_socket_control/` | Jan 2026 | Multi-motor controllers with a socket receiver/sender, multiprocessing, JSON gains, record/replay of positions. **Origin of the socket protocol.** |
| `03_single_leg_ik/` | Mar 2026 | First 3-DOF leg IK ("shift-circle"), 3D visualisation, stand-up animation; `main_ik_runtime/` is the single-leg runtime. |
| `04_imu_balancing/` | Mar–Apr 2026 | Full 12-motor runtime snapshot plus IMU balance / levelling / weight-shift experiments. Its loggers write into `cur_test/` by absolute path. |
| `05_four_leg_bringup/` | Mar 2026 | First complete 4-leg programs: `final/pre_and_post_homing.py` (ancestor of `robot/main_with_ik.py`), `with_ik/` stand-up + controller, gamepad test. |

See [`../docs/PROJECT_HISTORY.md`](../docs/PROJECT_HISTORY.md) for the timeline and the
old-path → new-path table.
