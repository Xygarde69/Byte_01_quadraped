#!/usr/bin/env python3
"""
ak60_v3_control.py — low-level CAN driver for the CubeMars AK60-6 V3.0 motor
══════════════════════════════════════════════════════════════════════════════

LAYER   Hardware driver (the lowest layer of the BYTE-01 software stack).
        Used by ``lib/can_runtime.py``; nothing above that should touch CAN.

WHAT IT DOES
    * Builds and sends **MIT-mode** control frames (position + velocity +
      stiffness + damping + feed-forward torque in one 8-byte frame).
    * Sends the small set of housekeeping commands we need: set zero /
      origin, and "current brake" (used here as the *disable* command).
    * Decodes the motor's **periodic status frames** (position, speed,
      current, temperature, error code).

WHAT IT DELIBERATELY DOES NOT DO
    It does not own the receive path in a multi-motor system.  Twelve motors
    share two buses, so a single background listener
    (``MotorFeedbackListener`` in ``can_runtime.py``) reads every frame and
    dispatches by motor ID.  ``AK60V3Motor.read_feedback`` is kept only for
    quick single-motor bench tests.

CAN PROTOCOL CHEAT-SHEET  (CubeMars "V3.0" servo protocol)
    Bus            : 1 Mbit/s, 29-bit *extended* IDs.
    Arbitration ID : ``(control_mode << 8) | motor_id``
                      └─ bits 28..8: control mode,  bits 7..0: motor CAN ID

    Control modes (``MODE_*`` below)
        0x00 duty cycle      0x03 velocity        0x06 position + velocity
        0x01 current         0x04 position        0x08 **MIT** (what we use)
        0x02 current brake   0x05 set origin
        0x29 is *not* a command: it is the ID the motor uses for its
        periodic status/feedback frame (``FEEDBACK_MODE_ID``).

    MIT command payload (8 bytes, big-endian bit packing)
        byte0         kp[11:4]
        byte1  [7:4]  kp[3:0]        [3:0] kd[11:8]
        byte2         kd[7:0]
        byte3         position[15:8]
        byte4         position[7:0]
        byte5         velocity[11:4]
        byte6  [7:4]  velocity[3:0]  [3:0] torque[11:8]
        byte7         torque[7:0]
    Each field is a float linearly mapped onto an unsigned integer of the
    given width over its [MIN, MAX] range (``_float_to_uint``).

    The motor then runs, at its own internal rate::

        torque_out = kp * (p_des - p_meas) + kd * (v_des - v_meas) + t_ff

    so ``kp`` is a spring stiffness, ``kd`` a damper, and ``torque`` a
    feed-forward.  Setting kp = kd = 0 gives pure torque control; setting
    velocity = 0 with kd > 0 gives a "hold position, resist motion" behaviour,
    which is how the rest of this project uses it.

    Status frame payload (ID mode 0x29, 8 bytes, big-endian signed)
        bytes 0-1  int16  position   × 0.1  → degrees
        bytes 2-3  int16  speed      (raw; 10 ERPM per count, left unscaled)
        bytes 4-5  int16  current    × 0.01 → amps
        byte  6    int8   temperature reported by the motor, °C
        byte  7    uint8  error code:
                          0 No Error            4 Under Voltage
                          1 Motor Over Temp     5 Encoder Error
                          2 Over Current        6 MOSFET Over Temp
                          3 Over Voltage        7 Motor Lock-up
                   Any non-zero value latches a fault in ``can_runtime.py``.

UNITS USED BY THIS MODULE
    position   radians on the wire (MIT command), degrees in decoded feedback
    kp         N·m / rad       kd   N·m·s / rad       torque   N·m
"""

import math
import struct
from typing import Optional, Dict, Any

import can


class AK60V3Motor:
    """
    One AK60-6 V3.0 motor on a CAN bus, driven in MIT mode.

    The object is a thin, mostly *transmit-only* wrapper: it knows its motor
    ID, how to pack frames for that ID, and how to push them onto a
    ``python-can`` bus.  It can either create its own bus or (the normal case
    in this project) share a bus that ``BusRuntime`` already opened, so that
    six motors can use one ``can0``/``can1`` channel without fighting over
    the socket.

    Attributes
    ----------
    motor_id : int
        CAN ID configured in the motor (1-12 on BYTE-01; see
        ``LEG_TO_MOTOR_IDS`` in ``main_with_ik.py`` for the leg mapping).
    position, velocity, current, torque, temperature, error
        Last values stored by ``read_feedback`` (single-motor bench use only).
    owns_bus : bool
        True if this object created the bus and therefore must shut it down
        in ``close()``.  False when the bus was injected.
    """

    # ── Control-mode IDs (the ``mode`` half of the arbitration ID) ───────────
    MODE_DUTY_CYCLE = 0x00
    MODE_CURRENT = 0x01
    MODE_CURRENT_BRAKE = 0x02
    MODE_VELOCITY = 0x03
    MODE_POSITION = 0x04
    MODE_SET_ORIGIN = 0x05
    MODE_POSITION_VELOCITY = 0x06
    MODE_MIT = 0x08

    # ID the motor stamps on its periodic status frames (not a command).
    FEEDBACK_MODE_ID = 0x29

    # ── MIT-mode value ranges (fixed by the motor firmware) ──────────────────
    # Out-of-range inputs are clamped (``_constrain``) before being quantised.
    P_MIN = -12.56      # rad  (≈ ±2 turns of the output shaft)
    P_MAX = 12.56
    V_MIN = -60.0       # rad/s
    V_MAX = 60.0
    KP_MIN = 0.0        # N·m/rad
    KP_MAX = 500.0
    KD_MIN = 0.0        # N·m·s/rad
    KD_MAX = 5.0
    T_MIN = -12.0       # N·m (feed-forward torque)
    T_MAX = 12.0

    def __init__(self, motor_id, can_interface='can0', bitrate=1000000, bus=None):
        """
        Parameters
        ----------
        motor_id : int
            The motor's CAN ID (low byte of every arbitration ID).
        can_interface : str
            SocketCAN channel name, e.g. ``"can0"``.  Only used when ``bus``
            is None.
        bitrate : int
            Bus bit rate in bit/s (the motors are fixed at 1 Mbit/s).
        bus : can.BusABC, optional
            An already-open, thread-safe bus to share.  If omitted a private
            ``can.ThreadSafeBus`` is created and closed again in ``close()``.
        """
        self.motor_id = int(motor_id)
        self.can_interface = can_interface
        self.bitrate = bitrate

        # Latest decoded feedback (only filled by ``read_feedback``).
        self.position = 0.0
        self.velocity = 0.0
        self.current = 0.0
        self.torque = 0.0
        self.temperature = 0
        self.error = 0

        if bus is None:
            # ignore_config=True: the interface is brought up by
            # system/can-setup.sh, python-can must not try to reconfigure it.
            self.bus = can.ThreadSafeBus(
                channel=can_interface,
                interface='socketcan',
                bitrate=bitrate,
                ignore_config=True,
            )
            self.owns_bus = True
        else:
            self.bus = bus
            self.owns_bus = False

    # ── Arbitration-ID helpers ───────────────────────────────────────────────

    def _construct_can_id(self, control_mode: int) -> int:
        """Return the 29-bit arbitration ID for a command: ``mode<<8 | id``."""
        return (control_mode << 8) | self.motor_id

    @staticmethod
    def rx_motor_id(msg: can.Message) -> int:
        """Extract the sender's motor ID (low byte) from a received frame."""
        return msg.arbitration_id & 0xFF

    @staticmethod
    def rx_control_mode(msg: can.Message) -> int:
        """Extract the mode field (bits 28..8) from a received frame."""
        return (msg.arbitration_id >> 8) & 0x1FFFFF

    @staticmethod
    def is_feedback_frame(msg: can.Message) -> bool:
        """
        True only for a well-formed periodic status frame:
        extended ID, exactly 8 data bytes, and mode == ``FEEDBACK_MODE_ID``.
        Anything else on the bus (our own echoes, other traffic) is ignored.
        """
        if msg is None:
            return False
        if not getattr(msg, "is_extended_id", False):
            return False
        if len(msg.data) != 8:
            return False
        if AK60V3Motor.rx_control_mode(msg) != AK60V3Motor.FEEDBACK_MODE_ID:
            return False
        return True

    @staticmethod
    def decode_feedback_message(msg: can.Message) -> Optional[Dict[str, Any]]:
        """
        Decode a status frame into engineering units.

        Returns ``None`` if ``msg`` is not a status frame, otherwise a dict:

        ==============  ==========================================================
        position_deg    output-shaft angle, degrees (raw int16 × 0.1)
        position_rad    same, radians
        velocity_raw    speed in the motor's native unit (10 ERPM / count);
                        used only as a "is it moving?" signal, so not rescaled
        current_A       phase/q-axis current in amps (raw int16 × 0.01)
        torque          alias of ``current_A`` — the frame carries no torque,
                        torque ∝ current for a BLDC, so we reuse the number
        temperature_C   temperature reported by the motor, °C
        error_code      0 = OK, otherwise see the error table in the module docstring
        ==============  ==========================================================
        """
        if not AK60V3Motor.is_feedback_frame(msg):
            return None

        # '>h' = big-endian signed 16-bit, 'b' = signed 8-bit.
        pos_int = struct.unpack('>h', bytes(msg.data[0:2]))[0]
        vel_int = struct.unpack('>h', bytes(msg.data[2:4]))[0]
        cur_int = struct.unpack('>h', bytes(msg.data[4:6]))[0]
        temp = struct.unpack('b', bytes(msg.data[6:7]))[0]
        err = int(msg.data[7])

        position_deg = pos_int * 0.1
        position_rad = math.radians(position_deg)
        velocity_raw = float(vel_int)
        current_A = round(cur_int * 0.01, 3)

        return {
            "position_rad": position_rad,
            "position_deg": position_deg,
            "velocity_raw": velocity_raw,
            "current_A": current_A,
            "torque": current_A,
            "temperature_C": temp,
            "error_code": err,
        }

    # ── Quantisation helpers ─────────────────────────────────────────────────

    def _constrain(self, value, min_val, max_val):
        """Clamp ``value`` into ``[min_val, max_val]``."""
        return max(min_val, min(max_val, value))

    def _float_to_uint(self, x, x_min, x_max, bits):
        """
        Map a float in ``[x_min, x_max]`` linearly onto ``0 … 2**bits - 1``.
        Truncates (does not round), matching the reference MIT-mode code.
        e.g. position ∈ [-12.56, 12.56] rad → 16-bit integer.
        """
        span = x_max - x_min
        offset = x - x_min
        return int((offset * ((1 << bits) - 1)) / span)

    # ── Origin (zero) handling ───────────────────────────────────────────────

    def set_temporary_origin(self):
        """
        Declare the *current* shaft position to be 0° until the next power
        cycle (payload byte 0x00).  This is what homing uses: nothing is
        written to the motor's flash, so a mistake is undone by rebooting.
        """
        can_id = self._construct_can_id(self.MODE_SET_ORIGIN)
        msg = can.Message(
            arbitration_id=can_id,
            data=[0x00],
            is_extended_id=True
        )
        self.bus.send(msg)

    def set_permanent_origin(self):
        """
        Store the current shaft position as 0° in the motor's non-volatile
        memory (payload byte 0x01).  Survives power cycles — use sparingly.
        """
        can_id = self._construct_can_id(self.MODE_SET_ORIGIN)
        msg = can.Message(
            arbitration_id=can_id,
            data=[0x01],
            is_extended_id=True
        )
        self.bus.send(msg)

    def set_zero_position(self, permanent=False):
        """Convenience switch between the temporary and permanent origin."""
        if permanent:
            self.set_permanent_origin()
        else:
            self.set_temporary_origin()

    # ── Commands ─────────────────────────────────────────────────────────────

    def send_mit_command(self, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
        """
        Send one MIT-mode frame.  The motor will drive toward ``position``
        like a spring/damper:  τ = kp·(position − θ) + kd·(velocity − θ̇) + torque.

        Parameters
        ----------
        position : float  — target angle, **radians** (clamped to ±12.56)
        velocity : float  — target speed, rad/s (clamped to ±60); we use 0
        kp       : float  — stiffness, N·m/rad (0 … 500)
        kd       : float  — damping,   N·m·s/rad (0 … 5)
        torque   : float  — feed-forward torque, N·m (±12)

        The call is fire-and-forget: it returns as soon as the frame is queued
        on the bus.  ``python-can`` raises ``CanError`` if the queue is full or
        the interface is down; callers (``BusRuntime.send_position_deg``) turn
        that into a latched fault.
        """
        # 1. Clamp every field into the range the firmware understands.
        position = self._constrain(position, self.P_MIN, self.P_MAX)
        velocity = self._constrain(velocity, self.V_MIN, self.V_MAX)
        kp = self._constrain(kp, self.KP_MIN, self.KP_MAX)
        kd = self._constrain(kd, self.KD_MIN, self.KD_MAX)
        torque = self._constrain(torque, self.T_MIN, self.T_MAX)

        # 2. Quantise to the field widths (16 / 12 / 12 / 12 / 12 bits).
        p_int = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        v_int = self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 12)
        kp_int = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 12)

        # 3. Pack into the 8-byte payload (layout in the module docstring).
        data = [
            (kp_int >> 4) & 0xFF,
            ((kp_int & 0x0F) << 4) | ((kd_int >> 8) & 0x0F),
            kd_int & 0xFF,
            (p_int >> 8) & 0xFF,
            p_int & 0xFF,
            (v_int >> 4) & 0xFF,
            ((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),
            t_int & 0xFF,
        ]

        can_id = self._construct_can_id(self.MODE_MIT)
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)

    def send_current_brake(self, current):
        """
        Current-brake mode: the motor shorts its windings with a braking
        current of ``current`` amps (clamped 0-60 A).  Payload is a big-endian
        int32 in milliamps.  With ``current = 0`` the motor is effectively
        released — see ``disable``.
        """
        current = self._constrain(current, 0.0, 60.0)
        data = struct.pack('>i', int(current * 1000.0))

        can_id = self._construct_can_id(self.MODE_CURRENT_BRAKE)
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)

    def disable(self):
        """
        Make the motor go limp (zero-current brake).  The AK60 has no
        dedicated "disable" frame in this protocol; this is the safe shutdown
        used on exit and on faults.  The leg will fall under gravity —
        support the robot before calling it.
        """
        self.send_current_brake(0.0)

    # ── Receive helpers (single-motor use only) ──────────────────────────────

    def read_feedback(self, timeout=0.1):
        """
        Block up to ``timeout`` s for one frame and, if it is this motor's
        status frame, store it in the ``position/velocity/...`` attributes.

        Returns True on success.  **Do not use on a shared bus**: this steals
        frames from other motors.  The multi-motor runtime uses the listener
        in ``can_runtime.py`` instead.
        """
        msg = self.bus.recv(timeout=timeout)
        if msg is None:
            return False
        if self.rx_motor_id(msg) != self.motor_id:
            return False

        decoded = self.decode_feedback_message(msg)
        if decoded is None:
            return False

        self.position = decoded["position_rad"]
        self.velocity = decoded["velocity_raw"]
        self.current = decoded["current_A"]
        self.torque = decoded["torque"]
        self.temperature = decoded["temperature_C"]
        self.error = decoded["error_code"]
        return True

    def get_feedback(self):
        """Return the values last stored by ``read_feedback`` as a dict."""
        return {
            "position": self.position,
            "velocity": self.velocity,
            "torque": self.torque,
            "current": self.current,
            "temperature": self.temperature,
            "error": self.error,
        }

    def close(self):
        """Shut the bus down — but only if this object created it."""
        if self.owns_bus:
            self.bus.shutdown()
