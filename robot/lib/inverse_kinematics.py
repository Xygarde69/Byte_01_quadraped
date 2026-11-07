#!/usr/bin/env python3
"""
inverse_kinematics.py — foot position (cm) → motor angles (degrees)
══════════════════════════════════════════════════════════════════════════════

LAYER   Pure maths; no hardware access.  Called by ``main_with_ik.py`` for every
        incoming foot-position packet.

LEG MODEL  (identical for all four legs; mirrored by sign flags in the IK)
    Three revolute joints per leg, origin at the hip:

        joint 1  HIP     θ1  rotates the whole leg about the fore-aft (X) axis
                         (abduction — swings the leg out to the side)
        joint 2  THIGH   θ2  swings the thigh in the leg's own plane
        joint 3  KNEE    θ3  bends the shank relative to the thigh

    Link lengths (cm):
        L1 = 5.995   fore-aft offset from hip axis to the thigh plane
        Lc = 9.094   lateral offset of the thigh plane from the hip axis
                     ("Lc" = *c*ylinder radius: when θ1 sweeps, the thigh
                     plane rotates on a cylinder of this radius)
        L2 = 22.0    thigh        L3 = 21.5   shank

    Forward kinematics (``RoboticLeg.forward_kinematics``), all lengths in cm::

        Le = L2·cos θ2 + L3·cos(θ2+θ3)         reach inside the thigh plane
        Lp = L2·sin θ2 + L3·sin(θ2+θ3)         "drop" inside the thigh plane
        X  = −L1 − Le
        Y  =  Lc·cos θ1 + Lp·sin θ1
        Z  = −Lc·sin θ1 + Lp·cos θ1

    BODY FRAME of the foot target (matches the whole project)
        +X forward      +Y toward the robot's RIGHT
        +Z up           standing feet therefore have Z < 0 relative to the hip

    ⚠ Left/right legs share these equations but mirror the Y offset: the IK
      uses a ``shoulder_dir``/``knee_dir`` sign pair per side (see the method).
      ``SIT_COORDS`` in ``config/robot_config.py`` shows it: left legs have
      Y ≈ −9.09, right legs Y ≈ +9.09, hence +Y = right.  (``mpu_leveling.py``
      agrees; the header of ``lib/trot_gait.py`` says "+Y left", which
      contradicts the data — see docs/KNOWN_ISSUES.md.)

IK STRATEGY  (``RoboticLeg.inverse_kinematics``)
    1. Le = −X − L1                                     (X fixes the reach)
    2. In the Y-Z plane the foot must lie at distance R ≥ Lc from the hip axis.
       θ1 follows from the tangent geometry:
            α  = atan2(−Z, Y)
            θ1 = α − shoulder_dir · acos(Lc / R)        (R = √(Y²+Z²))
       then Lp is the foot's component perpendicular to the Lc offset.
    3. Classic planar two-link solution for the thigh/knee:
            cos θ3 = (Le²+Lp² − L2² − L3²) / (2·L2·L3)
            θ3 = knee_dir · acos(cos θ3)               (picks knee-bend side)
            θ2 = atan2(Lp, Le) − atan2(L3·sin θ3, L2 + L3·cos θ3)
    Errors: ``ValueError`` if the target is inside the Lc cylinder or outside
    the reachable annulus (|L2−L3| … L2+L3).

FROM IK ANGLES TO MOTOR COMMANDS
    IK angles are *absolute* joint angles.  The motors, however, are zeroed
    **at the sitting pose** (see ``run_post_homing_live_control``), so a motor
    command is the IK angle *relative to the IK angle of the sit pose*:

        motor_deg = ik_deg(target) − ik_deg(SIT_COORDS[leg])

    ``ik_to_motor_deg`` does that subtraction.  Motor polarity (``flipped``)
    and the gearbox on the knee (``gear_ratio``) are applied afterwards in
    ``main_with_ik.convert_coords_to_motor_targets`` using ``motor_config.json``.

SIDE EFFECTS TO BE AWARE OF
    * The sit-pose angles are computed ONCE at import time (module level) and
      the import prints four IK lines.
    * ``inverse_kinematics`` and ``ik_to_motor_deg`` ``print`` every call.  At
      gait rates that is a lot of console output (and a little CPU).
"""
import math
import numpy as np
# IMPORT SHARED CONFIG
from config.robot_config import SIT_COORDS, LEG_ORDER


# ─── Kinematics Class ────────────────────────────────────────────────────────
class RoboticLeg:
    """
    Geometry of one leg plus its forward/inverse kinematics.  All four legs
    use the same instance (``leg_solver`` below); the leg's identity is passed
    to ``inverse_kinematics`` only to choose the left/right sign convention.
    """
    def __init__(self):
        # Updated specific link lengths
        self.L1 = 5.995     # cm — fore-aft hip offset
        self.Lc = 9.094     # cm — lateral thigh-plane offset ("cylinder radius")
        self.L2 = 22.0      # cm — thigh length
        self.L3 = 21.5      # cm — shank length

    def forward_kinematics(self, t1, t2, t3):
        """
        Foot position (X, Y, Z) in cm for joint angles ``t1, t2, t3`` given in
        **radians**.  Not used at runtime — handy for checking the IK
        (``inverse_kinematics`` followed by this should return the input).
        """
        # t1, t2, t3 in radians
        Le = self.L2 * math.cos(t2) + self.L3 * math.cos(t2 + t3)
        Lp = self.L2 * math.sin(t2) + self.L3 * math.sin(t2 + t3)

        X = -self.L1 - Le
        Y = self.Lc * math.cos(t1) + Lp * math.sin(t1)
        Z = -self.Lc * math.sin(t1) + Lp * math.cos(t1)

        return X, Y, Z

    def inverse_kinematics(self, X, Y, Z, leg_id):
        """
        Joint angles (θ1, θ2, θ3) **in degrees** that put the foot of leg
        ``leg_id`` at (X, Y, Z) cm in the body frame.

        Parameters
        ----------
        X, Y, Z : float   target foot position, cm (see module docstring).
        leg_id  : str     ``"fl" | "bl"`` (left legs) or ``"fr" | "br"``
                          (right legs).  Selects the mirrored sign convention.

        Raises
        ------
        ValueError  for an unknown ``leg_id`` or an unreachable target
                    (inside the Lc cylinder, or farther/closer than the two
                    links can span).  A small tolerance (1.0001) absorbs
                    floating-point error at full stretch.
        """
        # "fl" and "bl" are left legs. "fr" and "br" are right legs.
        if leg_id in ["fl", "bl"]:
            shoulder_dir = -1
            knee_dir = 1
        elif leg_id in ["fr", "br"]:
            shoulder_dir = 1
            knee_dir = -1
        else:
            raise ValueError(f"Invalid leg_id passed to IK solver: {leg_id}")

        # 1. Calculate Backward Extension (Le)
        Le = -X - self.L1

        # 2. Check Cylinder Intersection & Calculate Theta 1
        R_sq = Y**2 + Z**2
        if R_sq < self.Lc**2:
            raise ValueError("Reachability Error: Target is inside the Lc offset cylinder.")

        R = math.sqrt(R_sq)
        alpha = math.atan2(-Z, Y)

        # Uses the auto-detected shoulder_dir
        t1 = alpha - shoulder_dir * math.acos(self.Lc / R)
        Lp = Y * math.sin(t1) + Z * math.cos(t1)

        # 3. Check Workspace Distance Error
        D_sq = Le**2 + Lp**2
        cos_t3 = (D_sq - self.L2**2 - self.L3**2) / (2 * self.L2 * self.L3)

        if cos_t3 < -1.0001 or cos_t3 > 1.0001:
            raise ValueError("Reachability Error: Target is physically out of bounds.")
        cos_t3 = max(-1.0, min(1.0, cos_t3))

        # 4. Calculate Theta 3 and Theta 2
        # Uses the auto-detected knee_dir
        t3 = knee_dir * math.acos(cos_t3)
        t2 = math.atan2(Lp, Le) - math.atan2(self.L3 * math.sin(t3), self.L2 + self.L3 * math.cos(t3))

        print("For leg {}, Co-ordinates (cm) = ({:.2f}, {:.2f}, {:.2f}), IK angles (degrees) = ({:.2f}, {:.2f}, {:.2f})".format(leg_id.upper(), X, Y, Z, math.degrees(t1), math.degrees(t2), math.degrees(t3)))
        return math.degrees(t1), math.degrees(t2), math.degrees(t3)


# Instantiate the solver globally for the script to use
leg_solver = RoboticLeg()

# --- Pre-compute the sit coordinates ONCE ---
# For every leg: IK angles of its sitting pose.  These are the reference that
# the motors are zeroed at, so every later command is "angle minus this".
PRECOMPUTED_SIT_ANGLES = {}
for _leg in LEG_ORDER:
    # This calculates the (c1, c2, c3) baseline angles for each leg's sitting position
    PRECOMPUTED_SIT_ANGLES[_leg] = leg_solver.inverse_kinematics(*SIT_COORDS[_leg], _leg)

# Flip and gear_ratio are handled downstream via motor_config.json — not here.
# Right legs and left legs can differ due to mechanical assembly variations.

# Leg IDs: front-left, back-left, front-right, back-right
# (Duplicate of config.robot_config.LEG_ORDER; not referenced in this file.)
legs = ["fl", "bl", "fr", "br"]


# ─── IK to Motor Angles Conversion ─────────────────────────────────────────────
def ik_to_motor_deg(t1, t2, t3, leg):
    """
    Convert IK angles to relative motor angles in degrees, applying per-leg sit co-ords offset.

    Args:
        t1, t2, t3: target angles of foot with respect to hip in DEGREES.
        leg: which leg ("fl", "bl", "fr", "br")

    Returns:
        (m1, m2, m3) relative motor angles in degrees

    Raises:
        ValueError if leg is invalid
    """
    if leg not in LEG_ORDER:
        raise ValueError(f"Invalid leg '{leg}'. Must be one of {LEG_ORDER}.")

    # Get the "zero position" angles for this specific leg's sitting posture
    # c1, c2, c3 = leg_solver.inverse_kinematics(*SIT_COORDS[leg])
    c1, c2, c3 = PRECOMPUTED_SIT_ANGLES[leg]  # use pre-computed sit angles for efficiency

    # Calculate the delta (t1, c1, etc. are already in degrees from the class)
    m1 = t1 - c1
    m2 = t2 - c2
    m3 = t3 - c3

    print(f"Leg {leg.upper()}: Sit IK angles (degrees) = ({c1:.2f}, {c2:.2f}, {c3:.2f}), Relative Motor Angles (degrees) = ({m1:.2f}, {m2:.2f}, {m3:.2f})")

    return m1, m2, m3


# ─── Main IK Function for 1 Leg ─────────────────────────────────────────────
def calculate_each_motor_angles(x, y, z, leg):
    """
    Given a target foot position (x, y, z) in cm and leg ID, compute the motor angles to get there.
    Returns:
        Tuple of (m1, m2, m3) relative motor angles in degrees for the leg.
    """
    # compute target angles for foot position (returns degrees)
    t1, t2, t3 = leg_solver.inverse_kinematics(x, y, z, leg)

    # convert to relative motor angles
    m1, m2, m3 = ik_to_motor_deg(t1, t2, t3, leg)

    return (m1, m2, m3)


# ─── IK for All Legs ─────────────────────────────────────────────────────────
def calculate_motor_angles(coords_dict):
    """
    Parameters:
    coords_dict: dict of leg_id -> (x, y, z) target foot positions in cm, where leg_id is one of "fl", "bl", "fr", "br".

    Given a dict of all 4 legs such that leg_id -> (x, y, z) target foot positions in cm, compute the motor angles to get there.

    Sample dict:
    coords_dict = {
        "fl": (-9.094, 5.0, -30.0),
        "bl": (-9.094, -5.0, -30.0),
        "fr": (-9.094, 5.0, -30.0),
        "br": (-9.094, -5.0, -30.0)
    }

    Returns:
        Dict of all 4 legs such that leg_id -> (m1, m2, m3) tuple of relative motor angles in degrees.
    """
    motor_angles = {}
    for leg, coords in coords_dict.items():
        motor_angles[leg] = calculate_each_motor_angles(*coords, leg)
    return motor_angles


# ─── Entry Point (testing only) ──────────────────────────────────────────────
if __name__ == "__main__":
    # First test: compute motor angles for target foot position for front-left leg only
    # Note: Passing the exact SIT_COORDS should yield exactly (0.0, 0.0, 0.0)
    target_coords = SIT_COORDS["fl"]
    motor_angles = calculate_each_motor_angles(*target_coords, leg="fl")
    print(f"Front-left leg motor angles (degrees) = {motor_angles}")

    # Second test: compute motor angles for target foot position for all 4 legs
    target_dict = {
        "fl": (-9.094, 5.0, -30.0),
        "bl": (-9.094, -5.0, -30.0),
        "fr": (-9.094, 5.0, -30.0),
        "br": (-9.094, -5.0, -30.0)
    }

    print("\nCalculating for all legs:")
    all_motor_angles = calculate_motor_angles(target_dict)
    for leg, angles in all_motor_angles.items():
        print(f"Leg {leg.upper()}: Motor Angles (degrees) = ({angles[0]:.2f}, {angles[1]:.2f}, {angles[2]:.2f})")
