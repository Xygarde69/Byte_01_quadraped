#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  robot/tests/trot_gait_node_sid.py
# STATUS    ·  SUPERSEDED · interactive trot controller (Sid)
# ────────────────────────────────────────────────────────────────────────
# Interactive terminal trot (f/ff/b/bb/r/rr/l/ll) built on a phase-based
# cycloidal gait; assumes the robot is already standing (sit/stand
# belongs to the state manager). Superseded by lib/trot_gait.py.
# ════════════════════════════════════════════════════════════════════════
"""
byte01_trot_gait.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01 Trot Gait — Interactive Terminal Controller
(Phase-based cycloidal gait, ported from trot_gait_node.py)
IK is handled externally — this file sends raw XYZ foot targets over socket.

NOTE: Sit / stand is handled by state_manager. This script assumes the robot
      is already standing when it is launched.

Commands:
  f          → trot forward  1 full cycle
  ff         → trot forward  continuously  (press y to stop)
  b          → trot backward 1 full cycle
  bb         → trot backward continuously  (press y to stop)
  r          → strafe right  1 full cycle
  rr         → strafe right  continuously  (press y to stop)
  l          → strafe left   1 full cycle
  ll         → strafe left   continuously  (press y to stop)
  y          → stop continuous gait
  q / quit   → exit

Coordinate convention:
  x = lateral shift
  y = height          (up = positive, smaller Y = foot higher on real hardware)
  z = fore-aft reach  (forward = positive)

Foot trajectory equations
──────────────────────────────────────────────────────────────────────────────
Swing phase  (local phase s ∈ [0, 1]):
  z(s) = z0 - S/2 + S * [s - (1/2π)*sin(2πs)]     ← cycloidal (centered)
  y(s) = y0 + H * [1 - cos(2πs)]                   ← lifts to y0+2H at s=0.5

Stance phase (local phase s ∈ [0, 1]):
  z(s) = z0 + S/2 - S*s                            ← linear pushback
  y(s) = y0 - Hg * sin(πs)                         ← small ground push

Trot diagonal pairing:
  Pair A  (FL + BR) : phase offset 0.0
  Pair B  (FR + BL) : phase offset 0.5
  phase ∈ [0.0, 0.5) → swing
  phase ∈ [0.5, 1.0) → stance
"""

import math
import pickle
import socket
import threading
import time
from mpu_leveling import MPULeveler

HOST = "10.176.243.34"
PORT = 50000

# ─────────────────────────────────────────────────────────────────────────────
# STANDING POSE  (gait reference — robot must already be here on launch)
# ─────────────────────────────────────────────────────────────────────────────
STANDING_XYZ = [0.0, 0.0, -14.0]

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS  (from trot_gait_node.py — tune these)
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH  = 5.0     # cm  — S,  total fore-aft foot travel per cycle
STRIDE_Y       = 4.0     # cm  — lateral stride for strafe
LIFT_HEIGHT    = 4.0     # cm  — H,  half peak lift (actual peak = y0 + 2H)
GROUND_PUSH    = 0.2     # cm  — Hg, downward stance push
GAIT_FREQUENCY = 2.0     # Hz  — full cycle rate
UPDATE_HZ      = 100.0   # Hz  — send rate

GAIT_SPEED_DEG_PER_S = 800.0   # deg/s — used during gait frames

Z_LIFT_SIGN = -1   # -1 because on real hardware smaller Y = foot higher

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS  (trot diagonal pairs — from trot_gait_node.py)
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.0,   # ─┐ Pair A
    "br": 0.0,   # ─┘
    "fr": 0.5,   # ─┐ Pair B
    "bl": 0.5,   # ─┘
}

ALL_LEGS = ["fl", "fr", "bl", "br"]

# ─────────────────────────────────────────────────────────────────────────────
# RUNTIME STATE
# ─────────────────────────────────────────────────────────────────────────────
current_pos = {leg: list(STANDING_XYZ) for leg in ALL_LEGS}
_stop_flag  = threading.Event()
leveler = MPULeveler()

# ─────────────────────────────────────────────────────────────────────────────
# COMMUNICATION
# ─────────────────────────────────────────────────────────────────────────────
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

# ─────────────────────────────────────────────────────────────────────────────
# PHASE-BASED FOOT POSITION  (core logic from trot_gait_node._foot_pos)
# ─────────────────────────────────────────────────────────────────────────────
def _foot_pos_phase(leg: str, phase: float, axis: str, direction: int) -> list:
    """
    Compute [x, y, z] foot target for a leg at a given global phase.

    axis      : 'z' → forward/backward  |  'x' → left/right strafe
    direction : +1 or -1
    """
    S  = STRIDE_LENGTH if axis == 'x' else STRIDE_Y
    S *= direction
    H  = LIFT_HEIGHT
    Hg = GROUND_PUSH
    x0 = STANDING_XYZ[0]
    z0 = STANDING_XYZ[2]
    y0 = STANDING_XYZ[1]

    phi = (phase + PHASE_OFFSET[leg]) % 1.0

    if phi < 0.5:
        # ── Swing ──────────────────────────────────────────────────────────
        s  = phi / 0.5
        dx = -S / 2 + S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        dz = H * (1 - math.cos(2 * math.pi * s))
        x  = x0 + dx
        z  = z0 + Z_LIFT_SIGN * dz
    else:
        # ── Stance ─────────────────────────────────────────────────────────
        s  = (phi - 0.5) / 0.5
        dx = S / 2 - S * s
        dz = Hg * math.sin(math.pi * s)
        x  = x0 + dx
        z  = z0 + Z_LIFT_SIGN * dz

    if axis == 'y':
        y_sign = -1 if leg in ("fl", "bl") else 1
        y = y0 + y_sign * (x - x0)
        x = x0
    else:
        y = y0

    return [x, y, z]

# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNER
# ─────────────────────────────────────────────────────────────────────────────
def _run_gait_cycle(axis: str, direction: int, cycles: int = 1):
    """
    Execute gait cycles using continuous phase advancement.
    cycles=-1 runs until _stop_flag is set.
    Always completes a full cycle before stopping — all feet land cleanly.
    """
    dt          = 1.0 / UPDATE_HZ
    phase_step  = GAIT_FREQUENCY / UPDATE_HZ
    phase       = 0.0
    cycle_count = 0

    while True:
        while phase < 1.0:
            payload = {}
            for leg in ALL_LEGS:
                x, y, z = _foot_pos_phase(leg, phase, axis, direction)
                payload[leg] = [x, y, z]
            send_payload(payload, speed=GAIT_SPEED_DEG_PER_S)
            time.sleep(dt)
            phase += phase_step

        phase = 0.0
        cycle_count += 1

        for leg in ALL_LEGS:
            current_pos[leg] = list(STANDING_XYZ)

        if cycles != -1 and cycle_count >= cycles:
            break
        if cycles == -1 and _stop_flag.is_set():
            break


def _stop_listener():
    try:
        while True:
            if input().strip().lower() == 'y':
                _stop_flag.set()
                break
    except (EOFError, KeyboardInterrupt):
        _stop_flag.set()

# ─────────────────────────────────────────────────────────────────────────────
# STEP RUNNERS
# ─────────────────────────────────────────────────────────────────────────────
def run_one_cycle(axis: str, direction: int, label: str):
    print(f"● {label} — 1 cycle")
    _run_gait_cycle(axis, direction, cycles=1)
    print("  Done.")


def run_continuous(axis: str, direction: int, label: str):
    _stop_flag.clear()
    print(f"● {label} — continuous  (press  y  to stop)")
    listener = threading.Thread(target=_stop_listener, daemon=True)
    listener.start()
    _run_gait_cycle(axis, direction, cycles=-1)
    print(f"● {label} stopped — all feet on ground.")

# ─────────────────────────────────────────────────────────────────────────────
# MAIN LOOP
# ─────────────────────────────────────────────────────────────────────────────
def main():
    print("╔══════════════════════════════════════════════════╗")
    print("║     BYTE-01 Trot Gait Controller                 ║")
    print("╠══════════════════════════════════════════════════╣")
    print(f"║  STRIDE_LENGTH (S)     = {STRIDE_LENGTH} cm                  ║")
    print(f"║  LIFT_HEIGHT   (H)     = {LIFT_HEIGHT} cm  (peak = {2*LIFT_HEIGHT} cm)  ║")
    print(f"║  GROUND_PUSH   (Hg)    = {GROUND_PUSH} cm                  ║")
    print(f"║  STRIDE_Y (strafe)     = {STRIDE_Y} cm                  ║")
    print(f"║  GAIT_FREQUENCY        = {GAIT_FREQUENCY} Hz                  ║")
    print(f"║  UPDATE_HZ             = {UPDATE_HZ:.0f} Hz                 ║")
    print(f"║  GAIT_SPEED            = {GAIT_SPEED_DEG_PER_S:.0f} deg/s             ║")
    print(f"║  Cycle duration        = {1.0/GAIT_FREQUENCY:.2f} s               ║")
    print(f"║  Trajectory            = CYCLOIDAL (centered)    ║")
    print("╠══════════════════════════════════════════════════╣")
    print("║  f   → forward  1 cycle     ff  → forward  cont. ║")
    print("║  b   → backward 1 cycle     bb  → backward cont. ║")
    print("║  r   → right    1 cycle     rr  → right    cont. ║")
    print("║  l   → left     1 cycle     ll  → left     cont. ║")
    print("║  y   → stop continuous gait                      ║")
    print("║  q / quit → exit                                 ║")
    print("╚══════════════════════════════════════════════════╝\n")

    print("● Robot assumed to be standing (launched via state_manager).")
    leveler.capture_reference()
    leveler.level_after_stand()
    print("● Ready for gait commands.\n")

    while True:
        try:
            cmd = input("> ").strip().lower()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd:
            continue

        if cmd in ("q", "quit"):
            break
        elif cmd == "f":
            run_one_cycle(axis='z', direction=-1, label="FORWARD")
        elif cmd == "ff":
            run_continuous(axis='z', direction=-1, label="FORWARD")
        elif cmd == "b":
            run_one_cycle(axis='z', direction=+1, label="BACKWARD")
        elif cmd == "bb":
            run_continuous(axis='z', direction=+1, label="BACKWARD")
        elif cmd == "r":
            run_one_cycle(axis='x', direction=+1, label="STRAFE RIGHT")
        elif cmd == "rr":
            run_continuous(axis='x', direction=+1, label="STRAFE RIGHT")
        elif cmd == "l":
            run_one_cycle(axis='x', direction=-1, label="STRAFE LEFT")
        elif cmd == "ll":
            run_continuous(axis='x', direction=-1, label="STRAFE LEFT")
        else:
            print("  Unknown command.")
            print("  Single cycle : f | b | r | l")
            print("  Continuous   : ff | bb | rr | ll  (press y to stop)")
            print("  Exit         : q")

    print("● Exiting trot gait. Use state_manager to sit the robot.")


if __name__ == "__main__":
    main()