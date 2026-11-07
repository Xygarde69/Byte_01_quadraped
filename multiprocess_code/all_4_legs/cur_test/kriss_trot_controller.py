#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/kriss_trot_controller.py
# STATUS    ·  WORKSPACE · latest experiments, not yet merged into robot/
# ────────────────────────────────────────────────────────────────────────
# Gamepad (pygame) trot controller. Left stick: forward/back/strafe;
# buttons 1/2 = sit/stand, 3 = emergency stop; D-pad tilts the body
# (nose/butt/left/right) and returns to flat on release. --imu enables
# live MPU-6050 disturbance rejection via imu_correction.py.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 kriss_trot_controller.py [--imu]
# ════════════════════════════════════════════════════════════════════════
"""
kriss_trot_controller.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01 Trot Gait — Gamepad Controller

  Left stick axis 1  negative → forward
  Left stick axis 1  positive → backward
  Left stick axis 0  left/right → strafe
  Button 1           → sit
  Button 2           → stand
  Button 3           → emergency stop + exit
  DPAD UP            → tilt nose down  (fl/fr Y decreases)
  DPAD DOWN          → tilt butt down  (bl/br Y decreases)
  DPAD LEFT          → tilt left side  (fl/bl Y decreases)
  DPAD RIGHT         → tilt right side (fr/br Y decreases)
  (release DPAD)     → return to flat standing pose at same rate

Pass --imu to enable live MPU-6050 disturbance rejection:
  python3 kriss_trot_controller.py --imu
"""

import math
import pickle
import socket
import threading
import time
import sys
import pygame
from mpu_leveling import MPULeveler

HOST = "10.196.200.34"
PORT = 50000

SITTING_XYZ  = [-9.094, 11.0, 6.0]
STANDING_XYZ = [-9.094, 25.0, 6.0]

STRIDE_LENGTH  = 5.0
STRIDE_X       = 4.0
LIFT_HEIGHT    = 5.0
GROUND_PUSH    = 0.2
GAIT_FREQUENCY = 2
UPDATE_HZ      = 100.0

GAIT_SPEED_DEG_PER_S       = 400.0
TRANSITION_SPEED_DEG_PER_S = 120.0
Y_LIFT_SIGN = -1

PHASE_OFFSET = {
    "fl": 0.0,
    "br": 0.0,
    "fr": 0.5,
    "bl": 0.5,
}

ALL_LEGS = ["fl", "fr", "bl", "br"]
DEADZONE  = 0.15

TILT_RATE = 1.5    # cm per second
TILT_MAX  = 8.0    # max Y drop from standing baseline

# legs that dip for each DPAD direction
_TILT_LEGS = {
    (0,  1): ["fl", "fr"],
    (0, -1): ["bl", "br"],
    (-1, 0): ["fl", "bl"],
    (1,  0): ["fr", "br"],
}

# ── shared state ──────────────────────────────────────────────────────────────
current_pos   = {leg: list(SITTING_XYZ) for leg in ALL_LEGS}
leveler       = MPULeveler()
tilt_y        = {leg: 0.0 for leg in ALL_LEGS}
_kill_flag    = threading.Event()
_gait_active  = threading.Event()
_cmd_lock     = threading.Lock()
_cmd_axis      = 'z'
_cmd_direction = -1

# ── optional IMU correction ───────────────────────────────────────────────────
IMU_ENABLED = "--imu" in sys.argv
_corrector  = None   # assigned in main() after capture_reference()

_ZERO_CORR = {leg: 0.0 for leg in ALL_LEGS}

def _get_imu_corr() -> dict:
    """Return live per-leg Y corrections, or zeros when IMU is disabled."""
    return _corrector.get() if _corrector is not None else _ZERO_CORR

# ── socket helpers ────────────────────────────────────────────────────────────
def send_payload(payload: dict, speed: float = GAIT_SPEED_DEG_PER_S):
    payload["speed"] = speed
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)


def send_all_to(xyz: list, speed: float = TRANSITION_SPEED_DEG_PER_S):
    payload = {leg: list(xyz) for leg in ALL_LEGS}
    send_payload(payload, speed=speed)
    for leg in ALL_LEGS:
        current_pos[leg] = list(xyz)

# ── smooth transition ─────────────────────────────────────────────────────────
def smooth_transition(target_xyz: list, duration: float = 1.2):
    start    = {leg: list(current_pos[leg]) for leg in ALL_LEGS}
    n_frames = max(1, int(duration * UPDATE_HZ))
    for frame in range(n_frames + 1):
        if _kill_flag.is_set():
            break
        t    = frame / n_frames
        ease = t * t * (3.0 - 2.0 * t)
        payload = {
            leg: [
                start[leg][0] + (target_xyz[0] - start[leg][0]) * ease,
                start[leg][1] + (target_xyz[1] - start[leg][1]) * ease,
                start[leg][2] + (target_xyz[2] - start[leg][2]) * ease,
            ]
            for leg in ALL_LEGS
        }
        send_payload(payload, speed=TRANSITION_SPEED_DEG_PER_S)
        time.sleep(1.0 / UPDATE_HZ)
    for leg in ALL_LEGS:
        current_pos[leg] = list(target_xyz)

# ── foot position (cycloidal) ─────────────────────────────────────────────────
def _foot_pos_phase(leg: str, phase: float, axis: str, direction: int) -> list:
    S  = STRIDE_LENGTH if axis == 'z' else STRIDE_X
    S *= direction
    H  = LIFT_HEIGHT
    Hg = GROUND_PUSH
    x0 = STANDING_XYZ[0]
    z0 = STANDING_XYZ[2]
    y0 = STANDING_XYZ[1]

    phi = (phase + PHASE_OFFSET[leg]) % 1.0

    if phi < 0.5:
        s  = phi / 0.5
        dz = -S / 2 + S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        dy = H * (1 - math.cos(2 * math.pi * s))
        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy
    else:
        s  = (phi - 0.5) / 0.5
        dz = S / 2 - S * s
        dy = Hg * math.sin(math.pi * s)
        z  = z0 + dz
        y  = y0 + Y_LIFT_SIGN * dy

    if axis == 'x':
        x_sign = -1 if leg in ("fl", "bl") else 1
        x = x0 + x_sign * (z - z0)
        z = z0
    else:
        x = x0

    return [x, y, z]

# ── gait thread ───────────────────────────────────────────────────────────────
def gait_thread():
    """Runs one full gait cycle at a time while _gait_active is set."""
    dt         = 1.0 / UPDATE_HZ
    phase_step = GAIT_FREQUENCY / UPDATE_HZ

    while not _kill_flag.is_set():
        if not _gait_active.wait(timeout=0.05):
            continue
        if _kill_flag.is_set():
            break

        with _cmd_lock:
            axis      = _cmd_axis
            direction = _cmd_direction

        phase = 0.0
        while phase < 1.0:
            if _kill_flag.is_set():
                return
            payload  = {}
            imu_corr = _get_imu_corr()
            for leg in ALL_LEGS:
                x, y, z = _foot_pos_phase(leg, phase, axis, direction)
                # Apply IMU correction only to stance-phase legs (foot grounded).
                # Correcting a swing-phase leg mid-air would cause a jerk.
                phi = (phase + PHASE_OFFSET[leg]) % 1.0
                if phi >= 0.5:
                    y += imu_corr[leg]
                payload[leg] = [x, y, z]
            send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
            coord_str = "  ".join(
                f"{leg}=[{payload[leg][0]:6.2f},{payload[leg][1]:6.2f},{payload[leg][2]:6.2f}]"
                for leg in ALL_LEGS
            )
            print(f"\r  IK> {coord_str}", end="", flush=True)
            time.sleep(dt)
            phase += phase_step

        for leg in ALL_LEGS:
            current_pos[leg] = list(STANDING_XYZ)

# ── main ──────────────────────────────────────────────────────────────────────
def main():
    global _cmd_axis, _cmd_direction, _corrector

    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No controllers found. Please connect your controller.")
        pygame.quit()
        sys.exit(1)

    js = pygame.joystick.Joystick(0)
    js.init()
    print(f"Controller: {js.get_name()}")
    print(f"IMU correction: {'ENABLED (--imu)' if IMU_ENABLED else 'disabled'}")
    print("-" * 42)

    # ── IMU pre-flight (only when --imu is passed) ────────────────────────────
    if IMU_ENABLED:
        from imu_correction import IMUCorrector
        _corrector = IMUCorrector(leveler, ALL_LEGS, STANDING_XYZ, _kill_flag)

        if not _corrector.validate_mpu():
            print("✗ MPU health check failed — aborting.")
            pygame.quit()
            sys.exit(1)

        if not _corrector.validate_socket(HOST, PORT):
            print("✗ Motor controller not reachable — aborting.")
            pygame.quit()
            sys.exit(1)

        print("● Pre-flight checks passed.\n")

    print("● Sending SITTING pose...")
    send_all_to(SITTING_XYZ)
    leveler.capture_reference()
    time.sleep(1.5)

    print("● Transitioning to STANDING...")
    smooth_transition(STANDING_XYZ, duration=1.5)
    leveler.capture_reference()
    print("● Standing — ready!\n")
    print("  Left stick ↑      → forward")
    print("  Left stick ↓      → backward")
    print("  Left stick ←/→    → strafe")
    print("  Button 1          → SIT")
    print("  Button 2          → STAND")
    print("  Button 3          → STOP & EXIT")
    print("  DPAD ↑↓←→         → tilt (1 cm/s, max 8 cm, auto-returns)")
    print("-" * 42)

    # IMU thread must start AFTER capture_reference() so leveler has a baseline
    if IMU_ENABLED:
        _corrector.start()

    gt = threading.Thread(target=gait_thread, daemon=True)
    gt.start()

    LOOP_DT   = 0.02
    tilt_step = TILT_RATE * LOOP_DT

    try:
        while not _kill_flag.is_set():
            pygame.event.pump()

            hat = js.get_hat(0) if js.get_numhats() > 0 else (0, 0)

            if js.get_button(3):
                print("\n● Button 3 pressed — stopping.")
                _kill_flag.set()
                break

            if js.get_button(1):
                _gait_active.clear()
                print("\r  → SIT          ", flush=True)
                threading.Thread(
                    target=smooth_transition,
                    args=(SITTING_XYZ, 1.2),
                    daemon=True
                ).start()

            elif js.get_button(2):
                _gait_active.clear()
                print("\r  → STAND        ", flush=True)
                threading.Thread(
                    target=smooth_transition,
                    args=(STANDING_XYZ, 1.2),
                    daemon=True
                ).start()

            ax0 = js.get_axis(0)
            ax1 = js.get_axis(1)

            if abs(ax1) >= abs(ax0) and abs(ax1) > DEADZONE:
                with _cmd_lock:
                    _cmd_axis      = 'z'
                    _cmd_direction = -1 if ax1 < 0 else +1
                label = "FORWARD" if ax1 < 0 else "BACKWARD"
                print(f"\r  → {label:<12}", end="", flush=True)
                _gait_active.set()

            elif abs(ax0) > abs(ax1) and abs(ax0) > DEADZONE:
                with _cmd_lock:
                    _cmd_axis      = 'x'
                    _cmd_direction = +1 if ax0 > 0 else -1
                label = "STRAFE R" if ax0 > 0 else "STRAFE L"
                print(f"\r  → {label:<12}", end="", flush=True)
                _gait_active.set()

            else:
                if _gait_active.is_set():
                    print(f"\r  → IDLE        ", end="", flush=True)
                _gait_active.clear()

            # ── DPAD → tilt ───────────────────────────────────────────────────
            dip_legs     = _TILT_LEGS.get(hat, [])
            tilt_changed = False

            for leg in ALL_LEGS:
                old = tilt_y[leg]
                if leg in dip_legs:
                    tilt_y[leg] = max(-TILT_MAX, tilt_y[leg] - tilt_step)
                else:
                    if tilt_y[leg] < 0:
                        tilt_y[leg] = min(0.0, tilt_y[leg] + tilt_step)
                if abs(tilt_y[leg] - old) > 1e-6:
                    tilt_changed = True

            any_tilted = any(abs(v) > 1e-4 for v in tilt_y.values())

            imu_corr = _get_imu_corr()
            any_imu  = any(abs(v) > 1e-3 for v in imu_corr.values())

            if (tilt_changed or any_tilted or any_imu) and not _gait_active.is_set():
                tilt_payload = {
                    leg: [
                        STANDING_XYZ[0],
                        STANDING_XYZ[1] + tilt_y[leg] + imu_corr[leg],
                        STANDING_XYZ[2],
                    ]
                    for leg in ALL_LEGS
                }
                send_payload(tilt_payload, speed=TRANSITION_SPEED_DEG_PER_S)
                tilt_coord_str = "  ".join(
                    f"{leg}=[{tilt_payload[leg][0]:6.2f},{tilt_payload[leg][1]:6.2f},{tilt_payload[leg][2]:6.2f}]"
                    for leg in ALL_LEGS
                )
                print(f"\r  IK> {tilt_coord_str}", end="", flush=True)

            time.sleep(LOOP_DT)

    except KeyboardInterrupt:
        print("\nCtrl-C — exiting.")
        _kill_flag.set()

    finally:
        _gait_active.clear()
        _kill_flag.set()
        for leg in ALL_LEGS:
            tilt_y[leg] = 0.0
        gt.join(timeout=2.0)
        if IMU_ENABLED and _corrector is not None:
            _corrector.join(timeout=2.0)
        print("\n● Returning to sitting pose...")
        smooth_transition(SITTING_XYZ, duration=1.2)
        print("● Bye!")
        pygame.quit()


if __name__ == "__main__":
    main()
