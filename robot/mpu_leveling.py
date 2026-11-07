#!/usr/bin/env python3
"""
mpu_leveling.py
════════════════════════════════════════════════════════════════════════════════
BYTE-01  MPU-6050  One-Shot Chassis Leveling
────────────────────────────────────────────────────────────────────────────────

PURPOSE
-------
After every homing + stand transition the chassis can tilt because the homing
offset shifts slightly each run.  This module reads the MPU-6050 once (averaged)
after the bot stands up, computes how much each leg Z must change to bring the
chassis back to horizontal, and sends a corrected XYZ offset payload through
the existing socket to main_with_ik.py — exactly like state_manager_server.py
does for the STAND command.

ROBOT COORDINATE FRAME (your IK convention)
--------------------------------------------
    X  = forward / backward    (forward = positive)
    Y  = left / right          (right   = positive)
    Z  = vertical height       (UP      = positive,  i.e. Z becomes MORE NEGATIVE
                                          as the leg extends down — standing is dz=-14
                                          relative to sitting)

    SIT_COORDS  (per leg, from robot_config): e.g. approx (-9.094, ±Y_offset, 0.0)
    STAND offset sent as payload:  dx=0, dy=0, dz=-14.0
    → Absolute standing Z  ≈  SIT_COORDS[leg][2] + (-14.0)

    main_with_ik.py applies payload as:
        absolute[leg] = SIT_COORDS[leg] + payload[leg]   (element-wise)

MPU-6050 MOUNTING ASSUMPTION
------------------------------
    Board is flat on top of chassis, chip facing up.
    Default axis mapping:
        FORWARD_AXIS = 0   (MPU ax → robot forward / X direction)
        LATERAL_AXIS = 1   (MPU ay → robot lateral / Y direction, right +ve)

    If corrections act backwards after a physical tilt test, flip the sign:
        FORWARD_SIGN = -1   (reverses pitch correction direction)
        LATERAL_SIGN = -1   (reverses roll  correction direction)

    If the board is rotated 90° on the chassis, swap the axis indices:
        FORWARD_AXIS = 1
        LATERAL_AXIS = 0

SIGN CONVENTION FOR CORRECTIONS
---------------------------------
    delta_pitch > 0  →  front of chassis tilted DOWN
                        → front legs need MORE Z extension (more negative dz)
                          to push the front chassis back UP
                        → rear  legs need LESS  Z extension (less negative dz)

    delta_roll  > 0  →  right side tilted DOWN
                        → right legs need MORE Z extension (more negative dz)
                        → left  legs need LESS  Z extension

    Per-leg dZ correction formula (relative to nominal stand dz = -14.0):
        FL :  STAND_DZ − dz_pitch + dz_roll      (front, left)
        FR :  STAND_DZ − dz_pitch − dz_roll      (front, right)
        BL :  STAND_DZ + dz_pitch + dz_roll      (back,  left)
        BR :  STAND_DZ + dz_pitch − dz_roll      (back,  right)

    where dz_pitch and dz_roll are POSITIVE magnitudes of extension needed.
    Subtracting makes dz more negative = leg extends further down = pushes
    that corner of the chassis UP.

ROBOT GEOMETRY
--------------
    fore_aft distance (front hip to rear hip)  ≈ 42 cm  →  half = 21 cm
    lateral  distance (left hip to right hip)  ≈ 28 cm  →  half = 14 cm

SOCKET
------
    Sends to HOST:PORT = SOCKET_HOST:SOCKET_PORT from config/robot_config.py
    (same destination as state_manager_server_v2.py).  [Older revisions of this
    docstring said 127.0.0.1:50000; the code now reads the shared config.]
    Payload format  : pickle'd dict
                      {"fl":[dx,dy,dz], "fr":..., "bl":..., "br":..., "speed": float}
    The values are OFFSETS from SIT_COORDS — main_with_ik.py adds them on top.
    Speed used      : LEVEL_SPEED_DEG_PER_S = 120 deg/s  (slow, safe correction)
"""

import math
import pickle
import socket
import time
import smbus2
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, MAX_LIVE_DEG_PER_S

# ─── Socket (must match main_with_ik.py / robot_config) ─────────────────────
HOST = SOCKET_HOST
PORT = SOCKET_PORT

# ─── Speed for the leveling correction ───────────────────────────────────────
LEVEL_SPEED_DEG_PER_S = MAX_LIVE_DEG_PER_S

# ─── MPU-6050 I²C ────────────────────────────────────────────────────────────
MPU_I2C_BUS  = 1       # Raspberry Pi default I²C bus
MPU_ADDR     = 0x68    # AD0 pulled LOW (default)
PWR_MGMT_1   = 0x6B
ACCEL_XOUT_H = 0x3B    # first of 6 bytes: AX_H AX_L AY_H AY_L AZ_H AZ_L

# ─── Axis mapping ─────────────────────────────────────────────────────────────
# Raw accel tuple index:  0 = ax,  1 = ay,  2 = az
FORWARD_AXIS = 0    # which raw accel index carries the robot-forward tilt signal
LATERAL_AXIS = 1    # which raw accel index carries the robot-lateral tilt signal
FORWARD_SIGN = +1   # flip to -1 if pitch correction acts backwards on real hardware
LATERAL_SIGN = +1   # flip to -1 if roll  correction acts backwards on real hardware

# ─── Robot geometry ──────────────────────────────────────────────────────────
FORE_AFT_HALF = 42.0 / 2.0    # cm — distance from chassis centre to front/rear hips
LATERAL_HALF  = 28.0 / 2.0    # cm — distance from chassis centre to left/right hips

# ─── Stand offset (must match state_manager_server.py STAND state) ───────────
# This is the dz offset that puts the bot in standing position relative to sit.
# X and Y offsets are 0 — leveling only adjusts Z per leg.
# ⚠ MISMATCH: state_manager_server_v2.py's STAND uses dx = 1.5, but this file
#   uses STAND_DX = 4.0.  If level_after_stand() is enabled, the feet will
#   shift fore-aft by 2.5 cm when the leveling packet arrives.  Make the two
#   agree (ideally by defining STAND_* once in config/robot_config.py).
STAND_DX = 4.0
STAND_DY = 0.0
STAND_DZ = -14.0   # negative = leg extends downward in your Z convention

# ─── Z safety clamp ──────────────────────────────────────────────────────────
# These are dz OFFSET limits (relative to SIT_COORDS), not absolute Z values.
# A correction should never push a leg above sit level or too far below stand.
DZ_MIN = STAND_DZ - 6.0   # max safe extension beyond nominal stand  → -20.0 cm offset
DZ_MAX = 0.0               # can't retract past sitting height         →   0.0 cm offset

# ─── Averaging samples ────────────────────────────────────────────────────────
N_SAMPLES_REF   = 50    # samples for reference capture (once, while sitting)
N_SAMPLES_LEVEL = 30    # samples for post-stand leveling read


# ─────────────────────────────────────────────────────────────────────────────
class MPULeveler:
    """
    Reads the MPU-6050 accelerometer to compute chassis tilt and send a
    one-shot corrected foot-position offset payload via socket.

    Your IK: X = forward, Y = lateral, Z = height (more negative = lower).
    Corrections are computed as per-leg dZ adjustments on top of the nominal
    STAND_DZ offset, sent as payload offsets that main_with_ik.py applies
    on top of SIT_COORDS.

    Usage
    -----
        leveler = MPULeveler()

        # while sitting flat on the ground:
        leveler.capture_reference()

        # after sending the STAND command and waiting for the bot to rise:
        leveler.level_after_stand()
    """

    def __init__(self):
        self._bus = smbus2.SMBus(MPU_I2C_BUS)
        self._wake_mpu()
        self._ref_pitch    = 0.0
        self._ref_roll     = 0.0
        self._ref_captured = False
        print("[MPU] Initialized — I²C bus 1, address 0x68")

    # ── Hardware ──────────────────────────────────────────────────────────────

    def _wake_mpu(self):
        """Take MPU out of sleep mode and allow 150 ms to stabilise."""
        self._bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0x00)
        time.sleep(0.15)

    def _read_raw_accel(self) -> tuple:
        """
        Read one raw accelerometer sample.
        Returns (ax, ay, az) as signed 16-bit integers (±32767).
        Default ±2 g full-scale → 1 g ≈ 16384 LSB.
        """
        data = self._bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 6)

        def to_signed(hi_byte, lo_byte):
            v = (hi_byte << 8) | lo_byte
            return v - 65536 if v > 32767 else v

        ax = to_signed(data[0], data[1])
        ay = to_signed(data[2], data[3])
        az = to_signed(data[4], data[5])
        return ax, ay, az

    def _read_averaged(self, n: int) -> tuple:
        """
        Average n accelerometer readings with 5 ms spacing between each.
        Returns (ax_avg, ay_avg, az_avg) as floats.
        """
        total = [0.0, 0.0, 0.0]
        for _ in range(n):
            raw = self._read_raw_accel()
            for i in range(3):
                total[i] += raw[i]
            time.sleep(0.005)
        return tuple(t / n for t in total)

    # ── Tilt computation ──────────────────────────────────────────────────────

    def _accel_to_tilt(self, ax: float, ay: float, az: float) -> tuple:
        """
        Convert raw accelerometer values to chassis tilt angles in degrees.

        pitch > 0  →  front of chassis tilted DOWN
        roll  > 0  →  right side of chassis tilted DOWN

        Uses the full gravity magnitude for the vertical component so that
        readings stay stable even with combined pitch + roll.
        """
        accel = [ax, ay, az]

        a_forward = FORWARD_SIGN * accel[FORWARD_AXIS]   # gravity along robot forward
        a_lateral = LATERAL_SIGN * accel[LATERAL_AXIS]   # gravity along robot right

        g_sq      = ax**2 + ay**2 + az**2
        a_vert_sq = max(g_sq - a_forward**2 - a_lateral**2, 0.0)
        a_vert    = math.sqrt(a_vert_sq)

        pitch = math.degrees(math.atan2(a_forward, a_vert))   # forward tilt
        roll  = math.degrees(math.atan2(a_lateral, a_vert))   # lateral tilt
        return pitch, roll

    # ── Socket send ───────────────────────────────────────────────────────────

    def _send_payload(self, payload: dict):
        """
        Send the corrected XYZ offset payload through the socket.
        Format must match what main_with_ik.py expects:
            pickle'd dict  {"fl":[dx,dy,dz], "fr":..., "bl":..., "br":..., "speed": float}
        """
        payload["speed"] = LEVEL_SPEED_DEG_PER_S
        data = pickle.dumps(payload)
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.5)
                s.connect((HOST, PORT))
                s.sendall(data)
        except Exception as e:
            print(f"[MPU] Socket error: {e}")

    # ── Public API ────────────────────────────────────────────────────────────

    def capture_reference(self):
        """
        Call this ONCE while the bot is in sitting pose resting on its chassis
        (chassis guaranteed horizontal by physical contact with the ground).

        Records the MPU tilt at this known-flat state so that post-stand tilt
        is computed as a delta, removing any MPU mounting offset error.
        """
        print("[MPU] Capturing reference — keep bot sitting flat...")
        ax, ay, az = self._read_averaged(N_SAMPLES_REF)
        self._ref_pitch, self._ref_roll = self._accel_to_tilt(ax, ay, az)
        self._ref_captured = True
        print(f"[MPU] Reference locked — "
              f"pitch_ref={self._ref_pitch:+.3f}°  roll_ref={self._ref_roll:+.3f}°")

    def validate_health(self, n: int = 50) -> bool:
        """
        Perform n consecutive raw reads and return True only if every read
        succeeds with plausible values.

        Use as a startup gate to confirm the MPU is alive on I²C before
        relying on it for live corrections.
        """
        print(f"[MPU] Health check — performing {n} consecutive reads...")
        failures = 0
        for i in range(n):
            try:
                raw = self._read_raw_accel()
                if raw == (0, 0, 0):
                    failures += 1
                    print(f"[MPU] Read {i+1}: all-zero readout (sensor unresponsive)")
                elif any(abs(v) > 32767 for v in raw):
                    failures += 1
                    print(f"[MPU] Read {i+1}: out-of-range {raw}")
            except Exception as e:
                failures += 1
                print(f"[MPU] Read {i+1} FAILED: {e}")
            time.sleep(0.005)

        if failures == 0:
            print(f"[MPU] Health check passed — {n}/{n} reads OK.")
            return True
        print(f"[MPU] Health check FAILED — {failures}/{n} reads errored.")
        return False

    def compute_level_targets(self, n_samples: int = N_SAMPLES_LEVEL,
                              verbose: bool = True) -> dict:
        """
        Read current chassis tilt, compute per-leg dZ corrections and return
        a payload dict of offsets (from SIT_COORDS) ready to send to main_with_ik.py.

        Args
        ----
        n_samples : how many MPU samples to average for this read.
        verbose   : print per-call diagnostics.

        Returns
        -------
        dict  {"fl": [dx, dy, dz],  "fr": [dx, dy, dz],
               "bl": [dx, dy, dz],  "br": [dx, dy, dz]}

        dx and dy are always 0.0 — leveling only adjusts dz per leg.
        dz values are offsets from SIT_COORDS (same convention as STAND_DZ).

        Raises
        ------
        RuntimeError  if capture_reference() has not been called first.
        """
        if not self._ref_captured:
            raise RuntimeError(
                "[MPU] capture_reference() must be called before compute_level_targets()."
            )

        # ── Read current tilt ─────────────────────────────────────────────────
        ax, ay, az  = self._read_averaged(n_samples)
        pitch, roll = self._accel_to_tilt(ax, ay, az)

        delta_pitch = pitch - self._ref_pitch   # + → front tilted down
        delta_roll  = roll  - self._ref_roll  
        
        PITCH_DEADZONE_DEG = 0.5   # tune this on your hardware
        ROLL_DEADZONE_DEG  = 0.5

        if abs(delta_pitch) < PITCH_DEADZONE_DEG:
            print(f"[MPU] Pitch {delta_pitch:+.3f}° within deadzone — no correction.")
            delta_pitch = 0.0

        if abs(delta_roll) < ROLL_DEADZONE_DEG:
            print(f"[MPU] Roll {delta_roll:+.3f}° within deadzone — no correction.")
            delta_roll = 0.0

        # early exit — if both are zero, just send the flat STAND payload
        if delta_pitch == 0.0 and delta_roll == 0.0:
            print("[MPU] Chassis within deadzone — sending nominal STAND targets.")
            return {
                leg: [STAND_DX, STAND_DY, STAND_DZ]
                for leg in ("fl", "fr", "bl", "br")
            }  # + → right side tilted down

        # ── Convert tilt angle to cm of Z correction at each hip ──────────────
        # tan(angle) × half-distance = how much that side needs to move vertically.
        # These are POSITIVE magnitudes — the sign is applied per leg below.
        dz_pitch = math.tan(math.radians(delta_pitch)) * FORE_AFT_HALF
        dz_roll  = math.tan(math.radians(delta_roll))  * LATERAL_HALF

        # ── Build per-leg dZ offsets ───────────────────────────────────────────
        # In your Z convention, more negative dz = leg extends further down =
        # that corner of the chassis gets pushed UP.
        #
        # Front tilted down (delta_pitch > 0):
        #   → front legs must extend more  → subtract dz_pitch from STAND_DZ
        #   → rear  legs must extend less  → add    dz_pitch to   STAND_DZ
        #
        # Right side tilted down (delta_roll > 0):
        #   → right legs must extend more  → subtract dz_roll from STAND_DZ
        #   → left  legs must extend less  → add    dz_roll to   STAND_DZ
        #
        #   FL (front, left) :  STAND_DZ − dz_pitch + dz_roll
        #   FR (front, right):  STAND_DZ − dz_pitch − dz_roll
        #   BL (back,  left) :  STAND_DZ + dz_pitch + dz_roll
        #   BR (back,  right):  STAND_DZ + dz_pitch − dz_roll
        raw_dz = {
            "fl": STAND_DZ - dz_pitch + dz_roll,
            "fr": STAND_DZ - dz_pitch - dz_roll,
            "bl": STAND_DZ + dz_pitch + dz_roll,
            "br": STAND_DZ + dz_pitch - dz_roll,
        }

        # ── Safety clamp ──────────────────────────────────────────────────────
        clamped_dz = {leg: max(DZ_MIN, min(DZ_MAX, dz)) for leg, dz in raw_dz.items()}

        # ── Build final payload (dx=0, dy=0, dz=corrected per leg) ───────────
        payload = {
            leg: [STAND_DX, STAND_DY, clamped_dz[leg]]
            for leg in ("fl", "fr", "bl", "br")
        }

        # ── Diagnostics ───────────────────────────────────────────────────────
        if verbose:
            print(f"\n[MPU] Tilt — "
                  f"pitch: {delta_pitch:+.3f}°   roll: {delta_roll:+.3f}°")
            print(f"[MPU] Height deltas — "
                  f"dz_pitch: {dz_pitch:+.4f} cm   dz_roll: {dz_roll:+.4f} cm")
            print(f"  {'Leg':<4}  {'dZ offset':>10}  {'Δ from stand':>14}  {'Clamped?':>8}")
            print("  " + "─" * 44)
            for leg in ("fl", "fr", "bl", "br"):
                dz      = clamped_dz[leg]
                dz_raw  = raw_dz[leg]
                delta   = dz - STAND_DZ
                clamp_note = " ← CLAMPED" if abs(dz - dz_raw) > 1e-6 else ""
                print(f"  {leg.upper():<4}  {dz:>10.4f}  {delta:>+14.4f}{clamp_note}")
            print()

        return payload

    def level_after_stand(self):
        """
        One-shot leveling call — run this immediately after every STAND transition.

        Internally:
          1. Reads MPU (averaged over N_SAMPLES_LEVEL readings)
          2. Computes delta pitch and roll vs the sitting reference
          3. Derives per-leg dZ correction offset (keeping dx, dy at stand values)
          4. Sends the corrected payload once via socket to main_with_ik.py
        """
        print("[MPU] Computing post-stand level correction...")
        payload = self.compute_level_targets()
        self._send_payload(payload)
        print("[MPU] Level correction sent — one packet dispatched.")


# ─────────────────────────────────────────────────────────────────────────────
# Integration guide
# ─────────────────────────────────────────────────────────────────────────────
#
#   from mpu_leveling import MPULeveler
#
#   leveler = MPULeveler()           # ← once, at startup
#
#   # After homing, while bot is sitting flat on the ground:
#   leveler.capture_reference()
#
#   # After your STAND command completes (state_manager sends dz=-14):
#   time.sleep(2.0)                  # wait for physical stand transition
#   leveler.level_after_stand()      # read MPU, compute per-leg dZ, send once
#
#   # Inside the 'stand' command branch — same two lines work every time.
#
# ─────────────────────────────────────────────────────────────────────────────
# Axis troubleshooting
# ─────────────────────────────────────────────────────────────────────────────
# Step 1 — run standalone:  python3 mpu_leveling.py
#           Tilt the bot forward. Check that delta_pitch is positive.
#           If negative, set FORWARD_SIGN = -1.
#           Tilt right. Check that delta_roll is positive.
#           If negative, set LATERAL_SIGN = -1.
#
# Step 2 — if corrections are on the wrong axis entirely
#           (e.g. rolling triggers a pitch correction), swap FORWARD_AXIS and
#           LATERAL_AXIS.
# ─────────────────────────────────────────────────────────────────────────────


# ─── Standalone test (no socket, no robot needed) ────────────────────────────
if __name__ == "__main__":
    import sys

    print("=" * 60)
    print("  MPU-6050 Leveling — Axis Verification Mode")
    print("  X=forward  Y=lateral  Z=height (negative=down)")
    print("  Keep the bot sitting flat, then tilt manually")
    print("  to verify pitch/roll sign conventions.")
    print("  Ctrl+C to exit.")
    print("=" * 60)

    try:
        leveler = MPULeveler()
    except Exception as e:
        print(f"\n[ERROR] Could not initialise MPU: {e}")
        print("  Check: I²C enabled on Pi?  smbus2 installed?  AD0 wiring?")
        sys.exit(1)

    print("\n[Step 1] Capturing reference (keep chassis flat)...")
    leveler.capture_reference()

    print("\n[Step 2] Live tilt readout — tilt the bot to verify signs:")
    print(f"  {'pitch':>10}  {'roll':>10}  {'dz_pitch':>12}  {'dz_roll':>12}")
    print("  " + "─" * 50)

    try:
        while True:
            ax, ay, az = leveler._read_averaged(10)
            pitch, roll = leveler._accel_to_tilt(ax, ay, az)
            dp = pitch - leveler._ref_pitch
            dr = roll  - leveler._ref_roll
            dz_p = math.tan(math.radians(dp)) * FORE_AFT_HALF
            dz_r = math.tan(math.radians(dr)) * LATERAL_HALF
            print(f"  {dp:>+10.3f}°  {dr:>+10.3f}°  {dz_p:>+12.4f}cm  {dz_r:>+12.4f}cm",
                  end="\r", flush=True)
            time.sleep(0.1)
    except KeyboardInterrupt:
        print("\n\n[Done]")