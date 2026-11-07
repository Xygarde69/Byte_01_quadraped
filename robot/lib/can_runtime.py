#!/usr/bin/env python3
"""
can_runtime.py — per-bus runtime: shared state, feedback listener, watchdogs
══════════════════════════════════════════════════════════════════════════════

LAYER   Sits directly above ``ak60_v3_control.py`` (the raw CAN driver) and
        below ``homing_controller.py`` / ``main_with_ik.py``.

WHY IT EXISTS
    BYTE-01 has 12 motors on **two** CAN buses (``can0``: motors 1-6,
    ``can1``: motors 7-12).  Several threads touch each bus at once:

        ┌──────────────────────────┐  writes   ┌─────────────────────────┐
        │ python-can Notifier thread│ ───────▶ │  MotorState (per motor) │
        │ (MotorFeedbackListener)   │          │  guarded by one RLock   │
        └──────────────────────────┘          └───────────▲─────────────┘
                                                          │ deep-copy reads
        control / homing / logger threads ────────────────┘
        control threads ──send_position_deg()──▶ AK60V3Motor ──▶ CAN bus

    Rather than let every thread read the bus, **one** listener thread per bus
    decodes every status frame and stores it in a ``MotorState``.  Everybody
    else reads *snapshots* of that state.  ``BusRuntime`` packages the bus,
    the listener, the per-motor state, the fault latch and the watchdogs.

SAFETY MODEL ("fail loudly, latch, stop")
    Faults are *latched*: once ``BusRuntime.faulted`` is set it never clears
    itself.  Control loops call ``refresh_watchdogs()`` each tick and then
    ``raise RuntimeError(get_fault_summary())`` if ``faulted``; ``main`` catches
    that and disables every motor.  Conditions that latch a fault:

        * a motor reports a non-zero error code            → MOTOR_ERROR
        * a motor that was streaming stops for too long    → NO_FEEDBACK
        * the whole bus is silent for too long             → bus fault
        * the SocketCAN controller enters BUS_OFF          → bus fault
        * the listener thread itself crashes               → bus fault
        * a CAN send raises an exception                   → SEND_FAIL

    (``FaultCode`` has more members than are raised today; see its docstring.)

THREADING
    ``self.lock`` is a re-entrant lock shared by the listener and all readers.
    Public getters return **deep copies** so callers can use a snapshot without
    holding the lock.
"""

import copy
import math
import threading
import time
from dataclasses import dataclass
from enum import Enum, auto
from typing import Dict, List, Optional

import can

from lib.ak60_v3_control import AK60V3Motor


class FaultCode(Enum):
    """
    Reason a motor/bus was latched into the faulted state.

    Currently *raised* by this module: ``NO_FEEDBACK`` (stale/missing
    status frames), ``SEND_FAIL`` (an exception while transmitting) and
    ``MOTOR_ERROR`` (the motor's own error byte is non-zero).  The remaining
    members are reserved names for conditions that the homing code or future
    code may want to report; nothing sets them yet.
    """
    NONE = auto()
    NO_FEEDBACK = auto()
    SEND_FAIL = auto()
    BUS_ERROR = auto()
    MOTOR_ERROR = auto()
    STARTUP_TIMEOUT = auto()
    ZERO_TIMEOUT = auto()
    TRIGGER_TIMEOUT = auto()
    TRAVEL_LIMIT = auto()
    THREAD_DIED = auto()
    BARRIER_BROKEN = auto()


@dataclass
class MotorState:
    """
    Everything the runtime knows about one motor.  Written by the feedback
    listener (and a few ``BusRuntime`` helpers); read via ``get_state_copy``.

    Telemetry (straight from the last status frame)
        position_rad / position_deg   output-shaft angle
        velocity_raw                  motor-native speed units (10 ERPM/count)
        current_A / torque            measured current (torque is an alias)
        temperature_C, error_code     see ``ak60_v3_control`` for the table
        filtered_current_A            exponential moving average of current
                                      (alpha = 0.25) — used for hard-stop
                                      detection so a single noisy sample can't
                                      trigger homing

    Freshness bookkeeping
        last_feedback_time   wall-clock time of last status frame
        feedback_age         seconds since then (updated by the watchdog)
        feedback_count       number of frames received (a monotonic counter —
                             handy to detect "a *new* frame has arrived")
        missed_deadline_count  how often the watchdog saw stale data
        state_valid / startup_ready   True once the first frame has arrived

    Homing / hold targets
        homed                 True after this motor finished its homing nudge
        boot_hold_deg         position captured at power-up; used as the hold
                              target before a motor has been homed
        final_hold_deg        hold target after homing (the nudge position)
        last_command_time     when we last sent this motor a command

    Fault info
        faulted, fault_code, fault_msg
    """
    motor_id: int

    position_rad: float = 0.0
    position_deg: float = 0.0
    velocity_raw: float = 0.0
    current_A: float = 0.0
    torque: float = 0.0
    temperature_C: int = 0
    error_code: int = 0

    filtered_current_A: float = 0.0

    last_feedback_time: float = 0.0
    feedback_age: float = float("inf")
    feedback_count: int = 0
    missed_deadline_count: int = 0

    state_valid: bool = False
    startup_ready: bool = False
    homed: bool = False

    boot_hold_deg: float = 0.0
    boot_hold_captured: bool = False

    final_hold_deg: float = 0.0
    last_command_time: float = 0.0

    faulted: bool = False
    fault_code: FaultCode = FaultCode.NONE
    fault_msg: str = ""


class MotorFeedbackListener(can.Listener):
    """
    ``python-can`` listener: runs on the Notifier thread for **every** frame
    that passes the bus filter, decodes it and updates the matching
    ``MotorState``.  This is the *only* place feedback is received.

    It must stay fast and must never raise (an exception here is reported
    through ``on_error`` and ends up as a bus-level fault).
    """

    def __init__(self, states: Dict[int, MotorState], lock: threading.RLock, rx_info: dict):
        self.states = states      # shared dict owned by BusRuntime
        self.lock = lock          # shared RLock owned by BusRuntime
        self.rx_info = rx_info    # shared bus-level counters (last_rx_time, ...)

    def on_message_received(self, msg):
        # Ignore anything that isn't a motor status frame.
        if not AK60V3Motor.is_feedback_frame(msg):
            return

        now = time.time()
        motor_id = AK60V3Motor.rx_motor_id(msg)

        with self.lock:
            # Bus-level liveness: ANY valid status frame proves the bus works.
            self.rx_info["last_rx_time"] = now
            self.rx_info["rx_count"] += 1

            # A frame from a motor we weren't told about (e.g. the other bus's
            # ID range) is counted above but otherwise dropped.
            if motor_id not in self.states:
                return

            decoded = AK60V3Motor.decode_feedback_message(msg)
            if decoded is None:
                return

            st = self.states[motor_id]
            st.position_rad = decoded["position_rad"]
            st.position_deg = decoded["position_deg"]
            st.velocity_raw = decoded["velocity_raw"]
            st.current_A = decoded["current_A"]
            st.torque = decoded["torque"]
            st.temperature_C = decoded["temperature_C"]
            st.error_code = decoded["error_code"]

            # Exponential moving average of current:
            #   filtered = (1 - alpha) * filtered + alpha * new
            # Seeded with the first sample so it doesn't ramp up from zero.
            alpha = 0.25
            if st.feedback_count == 0:
                st.filtered_current_A = st.current_A
            else:
                st.filtered_current_A = (1.0 - alpha) * st.filtered_current_A + alpha * st.current_A

            st.last_feedback_time = now
            st.feedback_age = 0.0
            st.feedback_count += 1
            st.state_valid = True
            st.startup_ready = True

            # Remember where the motor was when we first heard from it; this
            # is the position we hold before homing so nothing jumps.
            if not st.boot_hold_captured:
                st.boot_hold_deg = st.position_deg
                st.boot_hold_captured = True

            # The motor itself reports a problem → latch immediately.
            if st.error_code != 0:
                st.faulted = True
                st.fault_code = FaultCode.MOTOR_ERROR
                st.fault_msg = f"motor error code {st.error_code}"

    def on_error(self, exc):
        """Called by python-can if the Notifier thread hits an exception."""
        with self.lock:
            self.rx_info["listener_failed"] = True
            self.rx_info["listener_error"] = exc

    def stop(self):
        """Required by the Listener interface; nothing to clean up."""
        return


class BusRuntime:
    """
    Everything needed to run the motors on **one** CAN channel.

    Owns: the ``ThreadSafeBus``, one ``AK60V3Motor`` per ID (sharing that
    bus), a ``MotorState`` per ID, the feedback listener/Notifier and the
    bus-level fault latch.  ``main_with_ik.py`` creates two of these
    (``rt0`` for ``can0``, ``rt1`` for ``can1``).

    Typical lifecycle::

        rt = BusRuntime("can0", [1, 2, 3, 4, 5, 6])
        rt.wait_for_periodic_feedback(...)       # wait until all motors talk
        loop:
            rt.refresh_watchdogs(...)            # then check rt.faulted
            rt.send_position_deg(id, deg, kp, kd)
        rt.disable_all(); rt.stop()
    """

    def __init__(self, channel: str, motor_ids: List[int], bitrate: int = 1_000_000):
        """
        Parameters
        ----------
        channel   : SocketCAN interface, ``"can0"`` or ``"can1"`` (must
                    already be up — see ``system/can-setup.sh``).
        motor_ids : the motor CAN IDs that live on this bus.
        bitrate   : 1 Mbit/s for the AK60-6 V3.
        """
        self.channel = channel
        self.motor_ids = sorted(set(int(mid) for mid in motor_ids))
        self.lock = threading.RLock()

        # Bus-level fault latch (separate from per-motor ``faulted``).
        self.faulted = False
        self.fault_msg = ""

        self.states: Dict[int, MotorState] = {
            mid: MotorState(motor_id=mid) for mid in self.motor_ids
        }
        self.motors: Dict[int, AK60V3Motor] = {}

        # Shared with the listener: bus-wide receive statistics.
        self.rx_info = {
            "last_rx_time": 0.0,
            "rx_count": 0,
            "listener_failed": False,
            "listener_error": None,
        }

        self.bus = can.ThreadSafeBus(
            channel=channel,
            interface="socketcan",
            bitrate=bitrate,
            ignore_config=True,
        )

        # Hardware/kernel-level filter: only let *status frames from our own
        # motors* through (exact 29-bit ID match), so the listener isn't
        # woken for unrelated traffic.  Failure to set filters is non-fatal —
        # the listener re-checks every frame anyway.
        filters = [
            {
                "can_id": (AK60V3Motor.FEEDBACK_MODE_ID << 8) | mid,
                "can_mask": 0x1FFFFFFF,
                "extended": True,
            }
            for mid in self.motor_ids
        ]
        try:
            self.bus.set_filters(filters)
        except Exception:
            pass

        # One motor object per ID, all injected with the shared bus.
        for mid in self.motor_ids:
            self.motors[mid] = AK60V3Motor(
                motor_id=mid,
                can_interface=channel,
                bitrate=bitrate,
                bus=self.bus,
            )

        # Start receiving.  From here on, MotorState fills itself.
        self.listener = MotorFeedbackListener(self.states, self.lock, self.rx_info)
        self.notifier = can.Notifier(self.bus, [self.listener], timeout=0.01)

    def stop(self):
        """Stop the listener thread and release the CAN socket."""
        try:
            if self.notifier is not None:
                self.notifier.stop(timeout=1.0)
        finally:
            self.bus.shutdown()

    # ── Fault latching ───────────────────────────────────────────────────────

    def latch_bus_fault(self, msg: str):
        """Mark the whole bus as faulted (sticky) with an explanatory message."""
        with self.lock:
            self.faulted = True
            self.fault_msg = f"[{self.channel}] {msg}"

    def latch_motor_fault(self, motor_id: int, code: FaultCode, msg: str):
        """Mark one motor *and* the bus as faulted (sticky)."""
        with self.lock:
            st = self.states[motor_id]
            st.faulted = True
            st.state_valid = False
            st.fault_code = code
            st.fault_msg = msg
            self.faulted = True
            self.fault_msg = f"[{self.channel}] M{motor_id}: {msg}"

    # ── State access (always returns copies) ─────────────────────────────────

    def get_state_copy(self, motor_id: int) -> MotorState:
        """Snapshot of one motor's state; safe to use without the lock."""
        with self.lock:
            return copy.deepcopy(self.states[motor_id])

    def get_all_state_copies(self) -> Dict[int, MotorState]:
        """Snapshot of every motor on this bus (one consistent moment)."""
        with self.lock:
            return copy.deepcopy(self.states)

    # ── Hold-target bookkeeping ──────────────────────────────────────────────

    def capture_boot_hold_positions(self):
        """
        Record each ready motor's current position as its "boot hold" target
        if that hasn't happened yet.  (The listener normally does this on the
        first frame; this is a belt-and-braces call after startup.)
        """
        with self.lock:
            for st in self.states.values():
                if st.startup_ready and not st.boot_hold_captured:
                    st.boot_hold_deg = st.position_deg
                    st.boot_hold_captured = True

    def set_final_hold_deg(self, motor_id: int, hold_deg: float):
        """Override the position this motor is held at after homing."""
        with self.lock:
            self.states[motor_id].final_hold_deg = hold_deg

    def mark_homed(self, motor_id: int, hold_deg: float):
        """Flag a motor as homed and make ``hold_deg`` its permanent hold target."""
        with self.lock:
            st = self.states[motor_id]
            st.homed = True
            st.final_hold_deg = hold_deg

    # ── Actuation ────────────────────────────────────────────────────────────

    def zero_motor(self, motor_id: int, permanent: bool = False):
        """
        Make the motor's current position the new 0°.  ``permanent=False``
        (the default and what the robot uses) only lasts until power-off.
        A send error latches ``SEND_FAIL`` and is re-raised.
        """
        try:
            self.motors[motor_id].set_zero_position(permanent=permanent)
        except Exception as exc:
            self.latch_motor_fault(motor_id, FaultCode.SEND_FAIL, f"zero failed: {exc}")
            raise

    def disable_motor(self, motor_id: int):
        """Best-effort "go limp" for one motor; never raises (used in cleanup)."""
        try:
            self.motors[motor_id].disable()
        except Exception:
            pass

    def disable_all(self):
        """Go limp on every motor of this bus.  Called on shutdown/fault."""
        for mid in self.motor_ids:
            self.disable_motor(mid)

    def send_position_deg(self, motor_id: int, position_deg: float, kp: float, kd: float, torque: float = 0.0):
        """
        Command one motor to ``position_deg`` (degrees, in the motor's current
        zero frame) with stiffness ``kp`` and damping ``kd``.

        Converts degrees → radians for the MIT frame and always requests zero
        velocity, so the motor behaves as a PD position controller.  On a send
        failure the motor is fault-latched and the exception re-raised.
        """
        try:
            self.motors[motor_id].send_mit_command(
                position=math.radians(position_deg),
                velocity=0.0,
                kp=kp,
                kd=kd,
                torque=torque,
            )
            with self.lock:
                self.states[motor_id].last_command_time = time.time()
        except Exception as exc:
            self.latch_motor_fault(motor_id, FaultCode.SEND_FAIL, f"MIT send failed: {exc}")
            raise

    def command_positions_deg(self, targets_deg: Dict[int, float], kp: float, kd: float, torque: float = 0.0):
        """Send ``send_position_deg`` to every motor on this bus with the same gains."""
        for mid in self.motor_ids:
            target_deg = targets_deg[mid]
            self.send_position_deg(mid, target_deg, kp=kp, kd=kd, torque=torque)

    # ── Health queries ───────────────────────────────────────────────────────

    def all_startup_ready(self, ids: Optional[List[int]] = None) -> bool:
        """True once every (or every listed) motor has delivered a status frame."""
        ids = self.motor_ids if ids is None else ids
        with self.lock:
            return all(self.states[mid].startup_ready for mid in ids)

    def get_idle_hold_targets(self) -> Dict[int, float]:
        """
        Where each motor should be held when it isn't being actively moved:
        the homed nudge position if homing finished, else the position it had
        at boot.  Used to keep not-yet-homed motors perfectly still while
        another motor is homing.
        """
        with self.lock:
            out = {}
            for mid in self.motor_ids:
                st = self.states[mid]
                out[mid] = st.final_hold_deg if st.homed else st.boot_hold_deg
            return out

    def get_fault_summary(self) -> str:
        """
        One-line explanation of why we are faulted: the bus message if set,
        otherwise the first faulted motor's message.
        """
        with self.lock:
            if self.fault_msg:
                return self.fault_msg
            for mid in self.motor_ids:
                st = self.states[mid]
                if st.faulted:
                    return f"[{self.channel}] M{mid}: {st.fault_msg}"
            return f"[{self.channel}] unknown fault"

    def refresh_watchdogs(
        self,
        feedback_timeout_s: float,
        bus_silence_timeout_s: float,
        fault_on_missing_feedback: bool = True,
    ):
        """
        Evaluate all health conditions and latch faults.  **Call once per
        control tick**, then check ``self.faulted``.

        Parameters
        ----------
        feedback_timeout_s     : a streaming motor with no frame for longer than
                                 this is stale (default tuning: 0.15 s).
        bus_silence_timeout_s  : no frame at all from any motor for this long
                                 means the bus/cable/power is gone (0.40 s).
        fault_on_missing_feedback : if False, stale motors are counted
                                 (``missed_deadline_count``) but not latched.
                                 Used during start-up, before every motor has
                                 spoken for the first time.

        Note the bus-silence check only arms after the first frame has been
        received (``last_rx > 0``), so a bus that never came up is handled by
        ``wait_for_periodic_feedback``'s start-up timeout instead.
        """
        now = time.time()

        with self.lock:
            # 1. The listener thread died.
            if self.rx_info["listener_failed"]:
                self.faulted = True
                self.fault_msg = f"[{self.channel}] listener failed: {self.rx_info['listener_error']}"

            # 2. Whole-bus silence.
            last_rx = self.rx_info["last_rx_time"]
            if last_rx > 0.0 and (now - last_rx) > bus_silence_timeout_s:
                self.faulted = True
                self.fault_msg = f"[{self.channel}] no RX for {now - last_rx:.3f}s"

            # 3. CAN controller went BUS_OFF (too many errors on the wire).
            try:
                bus_state = getattr(self.bus, "state", None)
                if bus_state is not None:
                    state_text = str(bus_state).upper()
                    if "BUS_OFF" in state_text:
                        self.faulted = True
                        self.fault_msg = f"[{self.channel}] bus state {bus_state}"
            except Exception:
                pass

            # 4. Per-motor checks.
            for mid in self.motor_ids:
                st = self.states[mid]

                if st.last_feedback_time > 0.0:
                    st.feedback_age = now - st.last_feedback_time
                else:
                    st.feedback_age = float("inf")

                # Motor-reported error (re-checked here as well as in the
                # listener so a fault is never missed between ticks).
                if st.error_code != 0:
                    st.faulted = True
                    st.fault_code = FaultCode.MOTOR_ERROR
                    st.fault_msg = f"motor error code {st.error_code}"
                    self.faulted = True
                    self.fault_msg = f"[{self.channel}] M{mid}: {st.fault_msg}"

                # Stale feedback from a motor that had been streaming.
                if st.startup_ready and st.feedback_age > feedback_timeout_s:
                    st.missed_deadline_count += 1
                    if fault_on_missing_feedback:
                        st.faulted = True
                        st.fault_code = FaultCode.NO_FEEDBACK
                        st.fault_msg = f"stale feedback age {st.feedback_age:.3f}s"
                        self.faulted = True
                        self.fault_msg = f"[{self.channel}] M{mid}: {st.fault_msg}"

    def wait_for_periodic_feedback(
        self,
        timeout_s: float,
        poll_period_s: float,
        feedback_timeout_s: float,
        bus_silence_timeout_s: float,
    ) -> bool:
        """
        Block until **every** motor on this bus has sent at least one status
        frame, or ``timeout_s`` elapses.  This is the "are all motors alive
        and in periodic-feedback mode?" start-up gate.

        Returns True on success (and records boot hold positions).  Returns
        False — with the bus latched as faulted — on a watchdog fault or if
        some motors never answered (the message lists the missing IDs).

        If it times out, check: motor power, CAN wiring/termination, that the
        interface is up at 1 Mbit/s, and that each motor's CAN ID is set
        correctly (see ``docs/HARDWARE_SETUP.md``).
        """
        print(f"[{self.channel}] waiting for periodic feedback from motors {self.motor_ids} ...")
        start = time.time()
        announced = set()

        while (time.time() - start) < timeout_s:
            self.refresh_watchdogs(
                feedback_timeout_s=feedback_timeout_s,
                bus_silence_timeout_s=bus_silence_timeout_s,
                fault_on_missing_feedback=False,
            )
            if self.faulted:
                return False

            with self.lock:
                for mid in self.motor_ids:
                    st = self.states[mid]
                    if st.startup_ready and mid not in announced:
                        print(f"  [{self.channel}] M{mid} ready at {st.position_deg:+.1f}°")
                        announced.add(mid)

                ready = all(self.states[mid].startup_ready for mid in self.motor_ids)

            if ready:
                self.capture_boot_hold_positions()
                print(f"[{self.channel}] startup feedback verified.")
                return True

            time.sleep(poll_period_s)

        # Timed out: report exactly which motors were silent.
        missing = []
        with self.lock:
            for mid in self.motor_ids:
                if not self.states[mid].startup_ready:
                    missing.append(mid)

        self.faulted = True
        self.fault_msg = f"[{self.channel}] startup timeout waiting for motors {missing}"
        return False
