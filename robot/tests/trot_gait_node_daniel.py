#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  robot/tests/trot_gait_node_daniel.py
# STATUS    ·  SUPERSEDED · standalone trot gait script (Daniel)
# ────────────────────────────────────────────────────────────────────────
# Phase-based trot that streams foot positions to the socket. Parameters
# at the top (stride 6/4 cm, lift 4, ground push 3, 800 deg/s).
# Superseded by lib/trot_gait.py + the state server.
# ════════════════════════════════════════════════════════════════════════
import math 
import time
from mpu_leveling import MPULeveler
from config.robot_config import LEG_ORDER

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH_X  = 6.0     # cm — Total fore-aft foot travel
STRIDE_LENGTH_Y  = 4.0     # cm — Lateral stride for strafe
LIFT_HEIGHT      = 4.0     # cm — Peak lift height (+z)
GROUND_PUSH      = 3.0     # cm — Downward stance push (-z)

GAIT_SPEED_DEG_PER_S = 800.0   

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS  (trot diagonal pairs)
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.0,   
    "br": 0.0,   
    "fr": 0.5,   
    "bl": 0.5,   
}

leveler = MPULeveler()

# ─────────────────────────────────────────────────────────────────────────────
# PHASE-BASED FOOT POSITION
# ─────────────────────────────────────────────────────────────────────────────
def _foot_pos_phase(leg: str, phase: float, axis: str, direction: int) -> list:
    S  = STRIDE_LENGTH_X if axis == 'x' else STRIDE_LENGTH_Y
    S *= direction
    H  = LIFT_HEIGHT
    Hg = GROUND_PUSH 

    phi = (phase + PHASE_OFFSET[leg]) % 1.0

    swing_stance = "Swing" if phi < 0.5 else "Stance"

    if phi < 0.5:
        # ── Swing (Leg in the air, +z) ─────────────────────────────────────
        s  = phi / 0.5 
        # X: Starts at -S/2, swings forward to +S/2
        dx = -S/2 + S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        # Z: Starts at 0, peaks at +H, returns to 0
        dz = H * (1 - math.cos(2 * math.pi * s)) / 2
        
    else:
        # ── Stance (Leg pushing ground, -z) ────────────────────────────────
        s  = (phi - 0.5) / 0.5
        # X: Starts at +S/2, pushes backward to -S/2
        dx = S/2 - S * (s - math.sin(2 * math.pi * s) / (2 * math.pi))
        # Z: Starts at 0, pushes down to -Hg, returns to 0
        dz = -Hg * (1 - math.cos(2 * math.pi * s)) / 2

    x  = dx
    z  = dz

    if axis == 'y': 
        y = x
        x = 0
    else:
        y = 0

    print(f" Leg {leg} {swing_stance}  | Phase: {phase:.2f} | x,y,z: {[x, y, z]}")
    return [x, y, z]

# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNER
# ─────────────────────────────────────────────────────────────────────────────
def _run_gait_cycle(axis: str, direction: int, cycles: int = 1):
    # Bumped update rate for smooth physical servo movement
    UPDATE_HZ      = 24.0   # Hz
    GAIT_FREQUENCY = 2.0     # Hz

    dt          = 1.0 / UPDATE_HZ
    phase_step  = GAIT_FREQUENCY / UPDATE_HZ
    phase       = 0.0 

    # Fixed loop condition to actually respect the 'cycles' argument
    while phase < cycles:
        payload = {}
        for leg in LEG_ORDER:
            x, y, z = _foot_pos_phase(leg, phase, axis, direction)
            payload[leg] = [x, y, z]
        
        # Here you would typically send `payload` to your IK solver / servos
        
        time.sleep(dt)
        phase += phase_step

def main():
    _run_gait_cycle(axis='x', direction=1, cycles=3)

if __name__ == "__main__":
    main()