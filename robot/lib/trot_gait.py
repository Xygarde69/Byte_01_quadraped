#!/usr/bin/env python3
"""
trot_gait.py — foot-trajectory generators for trot walking and in-place turning
══════════════════════════════════════════════════════════════════════════════

LAYER   Pure trajectory maths (no hardware).  Consumed by
        ``state_manager_server_v2.py`` which calls ``_foot_pos_phase`` ~100×
        per gait cycle and forwards the resulting foot positions to the robot.

WHAT A "TROT" IS
    The four legs move as two diagonal pairs that alternate:

              pair A: front-left  + back-right   (phase offset 0.0)
              pair B: front-right + back-left    (phase offset 0.5)

    While pair A is in the air (SWING), pair B is on the ground pushing the
    body along (STANCE), and vice-versa.  One gait *cycle* is phase 0 → 1; for
    each leg the first half of the cycle is swing, the second half stance
    (50 % duty).

PHASE → POSITION
    Each controller turns a global phase 0 ≤ φ < 1 into where a foot should be.
    Per leg:  φ_leg = (φ + PHASE_OFFSET[leg]) mod 1.

        φ_leg ≤ 0.5 → SWING  (foot in the air, moving forward)
            X:  cycloid       x = −S/2 + S·(s − sin(2πs)/(2π)),  s = φ_leg/0.5
                              → zero velocity at lift-off and touch-down
            Z:  raised cosine z = (H/2)·(1 − cos 2πs)       0 → H → 0
        φ_leg  > 0.5 → STANCE (foot on the ground, moving backward under the body)
            X:  linear        x = +S/2 − S·s,   s = (φ_leg − 0.5)/0.5
            Z:  small push    z = −P·sin(πs)                0 → −P → 0

    S = stride length, H = ``LIFT_HEIGHT``, P = ``PUSH_DEPTH`` (all cm).
    Because the foot moves backward in stance, the body moves forward.

OUTPUT FORMAT — INCREMENTS, NOT POSITIONS
    ``_foot_pos_phase(phase)`` returns **[dx, dy, dz]** = (new position) −
    (previous position) for that leg.  The caller *accumulates* the deltas
    into its own absolute foot position (``QuadrupedStateManager.current_positions``).
    This means the controllers must be called with monotonically increasing
    phase and never skipped, otherwise the foot drifts.  Each controller is
    single-use: create a fresh one for every gait cycle.

COORDINATES  (cm, relative to the STAND pose)
    +X forward.  Y is lateral.  Z is vertical, +Z = up.
    ⚠ The comments in this file call +Y "left", but ``SIT_COORDS`` puts the
      left legs at Y ≈ −9.09, i.e. +Y is the robot's RIGHT.  The runtime
      behaviour (state "RIGHT" = axis 'y', direction +1) is consistent with
      +Y = right; only the wording here is stale.

CONTENTS
    * gait parameters (stride, lift, speeds)       — tweak these to tune walking
    * ``TrotGaitController``   — forward / backward / strafe
    * ``RotationGaitController`` — clockwise / counter-clockwise yaw in place
    * ``_gait_loop`` + ``main`` — stand-alone demo that only *prints* the
      payloads (``# TODO: send payload to motors here``); the real send loop
      is in the state manager.
"""

import math
import time
# NOTE: imported but not used in this module (leftover from an earlier design).
# It does force `smbus2` to be installed and `mpu_leveling.py` to be importable
# (i.e. run from the robot/ directory) whenever the gait code is imported.
from mpu_leveling import MPULeveler
from config.robot_config import LEG_ORDER   # ["fl", "bl", "fr", "br"]

# ─────────────────────────────────────────────────────────────────────────────
# GAIT PARAMETERS
# ─────────────────────────────────────────────────────────────────────────────
STRIDE_LENGTH_X  = 6.0    # cm  — fore-aft foot travel per cycle
STRIDE_LENGTH_Y  = 6.0    # cm  — lateral foot travel per cycle (strafe)
GAIT_SPEED_DEG_PER_S = 400.0  # deg/s — servo speed during gait frames
                              # (not read here; the state manager sets its own 500)
LIFT_HEIGHT      = 5.0    # cm  — max foot lift during swing
PUSH_DEPTH       = 1.0    # cm  — max foot push-down during stance

# Rotation parameters
STRIDE_ARC_DEG   = 20.0   # degrees of yaw arc swept per full gait cycle

# Per-leg turning radius: distance (cm) from yaw axis to foot ground contact.
# Set to your leg Y-offset from centreline.  Your Lc = 9.094 cm → use 9.094.
LEG_RADIUS = {
    "fl": 9.094,
    "bl": 9.094,
    "fr": 9.094,
    "br": 9.094,
}

# ─────────────────────────────────────────────────────────────────────────────
# PHASE OFFSETS — trot diagonal pairs (shared by both controllers)
# ─────────────────────────────────────────────────────────────────────────────
PHASE_OFFSET = {
    "fl": 0.0,   # ─┐ Pair A — starts in swing
    "br": 0.0,   # ─┘
    "fr": 0.5,   # ─┐ Pair B — starts in stance
    "bl": 0.5,   # ─┘
}

# ─────────────────────────────────────────────────────────────────────────────
# TRANSLATION GAIT CONTROLLER  (your original, untouched)
# ─────────────────────────────────────────────────────────────────────────────
class TrotGaitController:
    """
    Computes incremental [dx, dy, dz] foot-position deltas for one leg
    during translational motion (forward / backward / strafe).

    axis      : 'x' (fore-aft)  |  'y' (lateral / strafe)
    direction : +1 forward/left  |  -1 backward/right
                (+1 = toward +X or +Y; see the Y-direction note above)

    Create one instance per leg per gait cycle and call ``_foot_pos_phase``
    with phase values that increase from 0 toward 1.
    """

    def __init__(self, leg: str, axis: str, direction: int):
        self.leg       = leg
        self.axis      = axis
        self.direction = direction

        # Scratch outputs of the path functions below (cm along the stride axis
        # and cm of height).
        self.x_com = 0.0
        self.z_com = 0.0

        self.phase_off    = PHASE_OFFSET[leg]
        # Signed stride: a negative direction mirrors the whole trajectory.
        self.STRIDE_LENGTH = (STRIDE_LENGTH_X if axis == 'x' else STRIDE_LENGTH_Y) * direction

        # Remember where the foot was on the previous call, to produce deltas.
        # Initialise to the trajectory position at phase 0 so the first delta
        # is ~0 (swing legs start at the back of the stroke, stance legs at
        # the front — the point where their swing ends / stance begins).
        self.last_pos = [0.0, 0.0, 0.0]
        self.x_start  = 0.0

        if self.phase_off == 0.0:   # starts in Swing
            self.x_start     = -self.STRIDE_LENGTH / 2
            self.last_pos[0] = self.x_start
        else:                        # starts in Stance
            self.x_start     =  self.STRIDE_LENGTH / 2
            self.last_pos[0] = self.x_start

    def calculate_cycloidal_path(self, phi_swing):
        """
        SWING trajectory.  phi_swing: 0.0 → 1.0
        X: −S/2 → +S/2 (cycloid: zero speed at both ends)  |  Z: 0 → LIFT_HEIGHT → 0
        Writes the result to ``self.x_com`` / ``self.z_com``.
        """
        x_start_swing = -self.STRIDE_LENGTH / 2
        total_dist    =  self.STRIDE_LENGTH
        self.x_com = x_start_swing + total_dist * (
            phi_swing - (1 / (2 * math.pi)) * math.sin(2 * math.pi * phi_swing)
        )
        self.z_com = (LIFT_HEIGHT / 2) * (1 - math.cos(2 * math.pi * phi_swing))

    def calculate_sinusoidal_path(self, phi_stance):
        """
        STANCE trajectory.  phi_stance: 0.0 → 1.0
        X: +S/2 → −S/2 (constant-speed drag backward)  |  Z: 0 → −PUSH_DEPTH → 0
        Writes the result to ``self.x_com`` / ``self.z_com``.
        """
        x_start_stance = self.STRIDE_LENGTH / 2
        total_dist     = -self.STRIDE_LENGTH
        self.x_com = x_start_stance + total_dist * phi_stance
        self.z_com = -PUSH_DEPTH * math.sin(math.pi * phi_stance)

    def _foot_pos_phase(self, phase: float) -> list:
        """
        Advance to global ``phase`` (0..1) and return the foot movement since
        the previous call as ``[dx, dy, dz]`` (cm).  For a strafe ('y') the
        along-stride motion is routed to Y instead of X.
        """
        phi = (phase + self.phase_off) % 1.0

        self.x_com = 0.0
        self.z_com = 0.0

        if phi <= 0.5:
            phi_swing = phi / 0.5
            self.calculate_cycloidal_path(phi_swing)
            # swing_stance = "Swing "
        else:
            phi_stance = (phi - 0.5) / 0.5
            self.calculate_sinusoidal_path(phi_stance)
            # swing_stance = "Stance"

        npx = self.x_com
        npz = self.z_com

        # Strafe: re-use the same stride curve but along Y.
        if self.axis == 'y':
            npy = npx
            npx = 0
        else:
            npy = 0

        dx = npx - self.last_pos[0]
        dy = npy - self.last_pos[1]
        dz = npz - self.last_pos[2]

        # print(f"  Leg {self.leg} {swing_stance} | Phase: {phase:.2f} | "
        #       f"x,y,z: {[self.x_com, self.z_com]} | delta: {[dx,dy,dz]}")

        self.last_pos = [npx, npy, npz]
        return [dx, dy, dz]


# ─────────────────────────────────────────────────────────────────────────────
# ROTATION GAIT CONTROLLER  (new)
# ─────────────────────────────────────────────────────────────────────────────

# Sign of tangential stride per leg:
#   CW  (+1) → left legs stride +X, right legs stride −X
#   CCW (−1) → reversed
_LATERAL_SIGN = {
    "fl": +1,
    "bl": +1,
    "fr": -1,
    "br": -1,
}


class RotationGaitController:
    """
    Computes incremental [dx, dy, dz] foot-position deltas for one leg
    during in-place yaw rotation.

    The foot traces a proper 2-D arc in the X-Y plane whose radius equals
    LEG_RADIUS[leg], keeping it tangential to the turning circle throughout
    the stance phase.

    direction : +1 → CW (viewed from above)   |   -1 → CCW

    Same interface and timing contract as ``TrotGaitController`` (monotonic
    phase, returns deltas, one instance per cycle).  Left and right legs
    stride in opposite fore-aft directions (``_LATERAL_SIGN``) so the body
    pivots about its centre instead of translating.
    """

    def __init__(self, leg: str, direction: int):
        self.leg       = leg
        self.direction = direction
        self.phase_off = PHASE_OFFSET[leg]

        self.R        = LEG_RADIUS[leg]
        self.arc_half = math.radians(STRIDE_ARC_DEG) / 2.0
        self.lat_sign = _LATERAL_SIGN[leg] * direction

        if self.phase_off == 0.0:   # starts in Swing — foot at back of arc
            init_angle = -self.arc_half
        else:                        # starts in Stance — foot at front of arc
            init_angle =  self.arc_half

        x0, y0 = self._arc_to_xy(init_angle)
        self.last_pos = [x0, y0, 0.0]

    def _arc_to_xy(self, angle: float):
        """
        Map an arc angle to (x, y) in the foot's local frame.
            x =  R · sin(angle) · lat_sign
            y = −R · (1 − cos(angle)) · lat_sign
        (A point on a circle of radius R, measured from its nearest point;
        y is the small sideways correction that keeps the foot on the circle
        rather than on a straight line.)
        """
        x = self.R * math.sin(angle) * self.lat_sign
        y = -self.R * (1.0 - math.cos(angle)) * self.lat_sign
        return x, y

    def _swing(self, phi: float):
        """
        phi : 0 → 1
        X-Y : cycloidal advance along the arc (back → front)
        Z   : cosine lift  0 → LIFT_HEIGHT → 0
        """
        arc_angle = -self.arc_half + 2 * self.arc_half * (
            phi - (1.0 / (2 * math.pi)) * math.sin(2 * math.pi * phi)
        )
        x, y = self._arc_to_xy(arc_angle)
        z    = (LIFT_HEIGHT / 2.0) * (1.0 - math.cos(2 * math.pi * phi))
        return x, y, z

    def _stance(self, phi: float):
        """
        phi : 0 → 1
        X-Y : linear sweep along the arc (front → back)
        Z   : sinusoidal push  0 → −PUSH_DEPTH → 0
        """
        arc_angle = self.arc_half - 2 * self.arc_half * phi
        x, y = self._arc_to_xy(arc_angle)
        z    = -PUSH_DEPTH * math.sin(math.pi * phi)
        return x, y, z

    def _foot_pos_phase(self, phase: float) -> list:
        """Returns [dx, dy, dz] incremental delta for this update step."""
        phi = (phase + self.phase_off) % 1.0

        if phi <= 0.5:
            phi_n = phi / 0.5
            x, y, z = self._swing(phi_n)
        else:
            phi_n = (phi - 0.5) / 0.5
            x, y, z = self._stance(phi_n)

        dx = x - self.last_pos[0]
        dy = y - self.last_pos[1]
        dz = z - self.last_pos[2]

        # print(f"  Leg {self.leg} | phase={phase:.3f} | "
        #       f"pos=({x:.2f},{y:.2f},{z:.2f}) | delta=({dx:.2f},{dy:.2f},{dz:.2f})")

        self.last_pos = [x, y, z]
        return [dx, dy, dz]


# ─────────────────────────────────────────────────────────────────────────────
# CORE GAIT RUNNERS
# ─────────────────────────────────────────────────────────────────────────────
# These two runners are a stand-alone dry-run: they generate the payloads and
# print them but do not talk to the robot.  Production code path:
#   state_manager_server_v2.py → _run_trot_gait_cycle / _run_rotation_gait_cycle

def _gait_loop(controllers: dict):
    """
    Shared timing loop used by both translate and rotate runners.
    Runs exactly one full gait cycle then returns.
    (0.2 s per cycle here vs 0.4 s in the state manager.)
    """
    cycle_time = 0.2
    dt         = 0.01
    n_steps    = int(cycle_time / dt)
    phase_step = 1.0 / n_steps
    phase      = 0.0

    while phase < 1.0:
        payload = {}
        for leg in LEG_ORDER:
            dx, dy, dz = controllers[leg]._foot_pos_phase(phase)
            payload[leg] = [dx, dy, dz]

        # TODO: send payload to motors here
        print(payload)

        time.sleep(dt)
        phase += phase_step


def _run_trot_gait_cycle(axis: str, direction: int):
    """
    Translational trot cycle.
    axis      : 'x' → fore-aft   |   'y' → lateral (strafe)
    direction : +1 forward/left   |   -1 backward/right
    """
    controllers = {leg: TrotGaitController(leg, axis, direction) for leg in LEG_ORDER}
    _gait_loop(controllers)


def _run_rotation_gait_cycle(direction: int):
    """
    In-place yaw rotation cycle.
    direction : +1 → CW (viewed from above)   |   -1 → CCW
    """
    controllers = {leg: RotationGaitController(leg, direction) for leg in LEG_ORDER}
    _gait_loop(controllers)


# ─────────────────────────────────────────────────────────────────────────────
# ENTRY POINT
# ─────────────────────────────────────────────────────────────────────────────

def main():
    """Dry-run one forward trot cycle (prints payloads; moves nothing)."""
    _run_trot_gait_cycle(axis='x', direction=1)

if __name__ == "__main__":
    main()
