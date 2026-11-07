"""
robot_config.py — the single source of truth for robot-wide constants
══════════════════════════════════════════════════════════════════════════════

Imported (as ``config.robot_config``) by the motor runtime, the IK, the state
manager, the gait code and the clients, so that all of them agree.  Per-motor
tuning (kp/kd/flip/gear ratio) lives next to this file in ``motor_config.json``.

⚠ Run every script from the ``robot/`` directory — the imports and the
  relative file paths (``config/…``, ``logs/…``) assume it.

CONTENTS
    MAX_LIVE_DEG_PER_S   default joint speed limit when a packet has no "speed"
    SOCKET_HOST / PORT   where the motor runtime LISTENS and everyone else SENDS
    LEG_ORDER            canonical leg ordering used for every dict/list
    SIT_COORDS           foot (x, y, z) in cm at the sitting pose  ← the IK origin
    SIT_TARGETS_DEG      raw motor angles of the sitting pose      ← used by startup

THE TWO "SIT" TABLES — how they relate
    The robot is homed, driven to ``SIT_TARGETS_DEG`` (raw motor degrees), and
    *then every motor is re-zeroed there*.  From that moment "motor angle 0"
    means "sitting".  ``SIT_COORDS`` is the same pose expressed as foot
    positions; the IK subtracts the IK angles of ``SIT_COORDS`` from every
    target so that SIT_COORDS ↔ motor angle 0.  The two tables therefore MUST
    describe the same physical pose — if you change one, re-derive the other
    (git history: commit "manually calculated SIT_COORDS").

COORDINATE FRAME FOR (x, y, z)  — centimetres, origin at each leg's hip
    +X forward, +Y toward the robot's right (left legs are negative Y),
    +Z up (feet below the hip have negative Z).
"""
# robot_config.py


# Default safety speed limit — used when the sender does NOT include a "speed" key.
# tapping_gait.py sends "speed": 900.0 to override this for fast tap movements.
# test_sender.py and any other client that omits "speed" will use this value.
# Units: degrees per second of *joint* motion (multiplied by the motor's gear
# ratio inside the runtime, so the knee — gear 1.5 — is allowed to move faster
# in motor degrees).
MAX_LIVE_DEG_PER_S = 120

# ─── Network Settings ────────────────────────────────────────────────────────
# The motor runtime (main_with_ik.py) BINDS this address; the state manager,
# keyboard controller and MPU leveler connect to it.  It is this Pi's IP on the
# current Wi-Fi/LAN, so it must be updated when the network changes
# (127.0.0.1 works when everything runs on the Pi itself).  If the address does
# not exist on the machine, the socket process fails to bind at start-up.
SOCKET_HOST = "127.0.0.1" # was "10.203.242.34" (old Wi-Fi IP); use this Pi's LAN IP only if a sender runs on another machine
SOCKET_PORT = 50000

# ─── Leg Identifiers ─────────────────────────────────────────────────────────
# fl = front-left, bl = back-left, fr = front-right, br = back-right.
# Motor IDs: fl 1-3, bl 4-6, fr 7-9, br 10-12 (hip, thigh, knee in each trio).
LEG_ORDER = ("fl", "bl", "fr", "br")

# ─── Sitting Cartesian Coordinates (X, Y, Z in cm) ───────────────────────────
# Used by Inverse Kinematics and as the baseline for live socket tracking.
# Every packet sent to the robot is an OFFSET added to these (see
# main_with_ik.run_post_homing_live_control): target = SIT_COORDS[leg] + (dx, dy, dz).
# The four legs differ slightly because of assembly tolerances.
SIT_COORDS = {
    "fl": (-13.60, -9.09, -15.21),
    "bl": (-13.03, -9.09, -14.00),
    "fr": (-12.76, 9.09, -14.05),
    "br": (-14.46, 9.09, -16.71)
}

# ─── Physical Sitting Motor Angles (Degrees) ─────────────────────────────────
# The absolute hardware angles the motors must reach BEFORE zeroing.
# will not be transformed as per motor_config.json
# Keys are motor IDs, values are raw angles in the motors' homed frame
# (hip, thigh, knee per leg).  After the motors reach these angles the runtime
# issues a temporary "set origin" so this pose becomes angle 0.

SIT_TARGETS_DEG = {
    1: -85.0,  2: 114.0, 3: -9.0,    # fl - flipped and gear ratio applied
    4: -85.0,  5: 114.0, 6: -3.0,    # bl - flipped and gear ratio applied
    7: 85.0,  8: -114.0, 9: 6.0,    # fr
    10: 87.0, 11: -116.0, 12: 9.0   # br
}
