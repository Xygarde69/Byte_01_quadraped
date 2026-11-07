#!/usr/bin/env python3
"""
homing_controller.py — hard-stop homing of each leg's three joints
══════════════════════════════════════════════════════════════════════════════

LAYER   Above ``can_runtime.BusRuntime``.  One ``BusHomingController`` drives
        one CAN bus; ``main_with_ik.py`` runs the two buses in parallel threads.

THE PROBLEM
    The AK60 reports its angle relative to wherever it was powered on (or
    last zeroed).  After a power cycle the robot has no idea where its joints
    are.  Homing finds an absolute reference by driving each joint *into a
    mechanical hard stop*, detecting the stop from the motor current, and
    declaring that spot to be 0°.

THE ALGORITHM  (``_run_phase`` — repeated for each motor in a phase, in order)

    1. SEARCH     Move toward ``start + target_deg`` along a velocity- and
                  acceleration-limited profile (``_profile_step``).  All other
                  motors on the bus are held in place (``_send_mixed_targets``).
    2. CREEP      Once the target is reached and the joint has settled, keep
                  nudging the target further in ``search_direction_deg`` at a
                  slow, constant speed (``continuous_search_vel_deg_s``) so the
                  joint gently walks into the stop.
    3. CONTACT    When the *filtered* current reaches ``trigger_current_A`` we
                  **latch** contact: freeze the target just past the current
                  position (``contact_hold_offset_deg``) so the motor presses
                  with bounded, steady force instead of pushing harder.
    4. CONFIRM    Contact counts as a real hard stop if EITHER
                    * current stays ≥ threshold for ``trigger_confirm_s``, or
                    * the joint is stalled (|speed| ≤ ``contact_stall_vel_max``)
                      for ``trigger_confirm_s``.
                  If the joint instead moves ≥ ``contact_release_move_deg`` away
                  and the current drops, the latch is released (it was a
                  transient, e.g. a bump or friction spike) and the search goes on.
    5. ZERO       ``zero_motor`` (temporary origin) → wait until the motor
                  reports a position near 0 (``_wait_for_zero_feedback``).
    6. NUDGE      Back off from the stop to ``nudge_back_deg`` (a known-safe
                  pose a few degrees to tens of degrees away from the stop) and
                  record it as the hold target (``mark_homed``).

    SAFETY LIMITS: the search aborts with ``RuntimeError`` if the joint travels
    more than ``max_search_travel_deg`` from where it started, if the stop flag
    is set, or if ``BusRuntime`` latches any fault.

PHASES
    A phase is an ordered list of motors on a bus.  The robot homes two legs at
    a time (one per bus) so the other two legs can keep supporting it:

        Phase 1:  can1 → front-right leg   |  can0 → back-left leg
        Phase 2:  can1 → back-right leg    |  can0 → front-left leg

    Within a leg the joints are homed thigh → knee → hip (e.g. motors 5 → 6 → 4
    for the back-left leg; per leg the IDs are hip/thigh/knee = 4/5/6), as set
    by ``CAN_CONFIG`` in ``main_with_ik.py`` where all the numbers live.

UNITS: everything is degrees / seconds / amps; "raw velocity" is the motor's
native 10-ERPM unit and is only compared against thresholds.
"""

import math
import threading
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set

from lib.can_runtime import BusRuntime


@dataclass(frozen=True)
class HomingMotorConfig:
    """
    Homing recipe for ONE motor.  Defined in ``CAN_CONFIG`` (main_with_ik.py).

    motor_id             physical CAN ID of the motor.
    target_deg           initial *relative* move (degrees from the starting
                         position) toward the stop — a generous guess that
                         should land at or near the stop.
    search_direction_deg only the **sign** matters: +1 = keep creeping in the
                         positive direction, -1 = negative.
    trigger_current_A    filtered motor current (A) that means "pressing on a
                         hard stop".  Must exceed free-motion current but stay
                         within what the joint/gearbox can safely take.
    nudge_back_deg       raw encoder angle to retreat to *after* zeroing at the
                         stop; this becomes the held, homed pose.  Sign differs
                         per joint because stops are at different ends.
    """
    motor_id: int
    target_deg: float
    search_direction_deg: float
    trigger_current_A: float
    nudge_back_deg: float


@dataclass
class ControlTuning:
    """
    All control gains, thresholds and timeouts for homing and for the watchdogs.
    One instance is built in ``main_with_ik.main`` and shared by both buses.

    ── Gains (MIT-mode kp [N·m/rad], kd [N·m·s/rad]) ─────────────────────────
    active_kp/kd         the motor currently moving (searching or nudging).
    hold_kp/kd           motors waiting to be homed — soft, just stay put.
    homed_hold_kp/kd     motors already homed — stiffer.
    safe_idle_kp/kd      the very gentle hold used right after boot, before any
                         motion, so nothing lurches.

    ── Loop timing ────────────────────────────────────────────────────────────
    loop_hz              control rate (100 Hz); ``loop_dt`` is derived.
    min_move_time_s      a nudge may not be declared "done" sooner than this.
    status_period_s      how often the progress line is printed.
    wait_log_period_s    how often a verbose "waiting…" line is printed.

    ── Contact detection ──────────────────────────────────────────────────────
    trigger_confirm_s            how long contact must persist (see step 4).
    trigger_velocity_raw_max     "essentially stopped" threshold, raw units.
    contact_hold_offset_deg      how far past the contact point to aim (step 3).
    contact_release_move_deg     back-off distance that cancels a false latch.
    contact_stall_vel_max        |raw speed| below which the joint counts as stalled.
    max_search_travel_deg        hard abort: search moved this far without contact.

    ── Motion profile ─────────────────────────────────────────────────────────
    search_max_vel_deg_s / search_max_acc_deg_s2     fast approach.
    continuous_search_vel_deg_s                      slow creep into the stop.
    nudge_max_vel_deg_s / nudge_max_acc_deg_s2       back-off move.
    advance_target_tolerance_deg, advance_settle_time_s
        define "arrived at the initial target" before creeping starts.
    nudge_position_tolerance_deg, nudge_settle_time_s
        define "arrived at the nudge pose".

    ── Watchdogs / start-up ───────────────────────────────────────────────────
    feedback_timeout_s       per-motor stale-frame limit.
    bus_silence_timeout_s    whole-bus silence limit.
    startup_timeout_s, startup_poll_s    wait for the first frames after boot.
    zero_position_tolerance_deg          |angle| considered "zeroed".

    ── Defined but currently NOT read by any code ─────────────────────────────
    seconds_per_deg, max_search_time_s, zero_feedback_timeout_s,
    stall_progress_epsilon_deg, stall_timeout_s
        (left over from earlier versions; changing them has no effect.)
    """
    active_kp: float
    active_kd: float
    hold_kp: float
    hold_kd: float
    homed_hold_kp: float
    homed_hold_kd: float
    loop_hz: float

    min_move_time_s: float
    seconds_per_deg: float

    trigger_confirm_s: float
    trigger_velocity_raw_max: float = 5.0

    feedback_timeout_s: float = 0.15
    bus_silence_timeout_s: float = 0.40

    startup_timeout_s: float = 5.0
    startup_poll_s: float = 0.01
    zero_feedback_timeout_s: float = 1.5
    zero_position_tolerance_deg: float = 5.0

    max_search_time_s: float = 10.0
    max_search_travel_deg: float = 180.0

    status_period_s: float = 0.5

    safe_idle_kp: float = 48.0
    safe_idle_kd: float = 0.8

    nudge_position_tolerance_deg: float = 2.0
    nudge_settle_time_s: float = 0.12

    search_max_vel_deg_s: float = 80.0
    search_max_acc_deg_s2: float = 120.0
    continuous_search_vel_deg_s: float = 6.0

    nudge_max_vel_deg_s: float = 60.0
    nudge_max_acc_deg_s2: float = 168.0

    advance_target_tolerance_deg: float = 2.0
    advance_settle_time_s: float = 0.20

    stall_progress_epsilon_deg: float = 0.25
    stall_timeout_s: float = 0.50

    wait_log_period_s: float = 1.0

    contact_hold_offset_deg: float = 0.5
    contact_release_move_deg: float = 3.0
    contact_stall_vel_max: float = 2.0


    def __post_init__(self):
        # Derived: seconds per control tick.
        self.loop_dt = 1.0 / self.loop_hz


@dataclass
class MotionProfileState:
    """Commanded position and velocity of the motion profile (not measured!)."""
    cmd_deg: float
    vel_deg_s: float = 0.0


class BusHomingController:
    """
    Runs the homing sequence for the motors of ONE ``BusRuntime``.

    ``main_with_ik.controller_thread_entry`` calls the private ``_run_phase``
    directly from one thread per bus so that the two buses home in lock-step
    (see ``hold_until_both_phase1_done``).  ``run_all_phases`` is the
    single-bus convenience wrapper that does the whole thing sequentially.

    Parameters
    ----------
    runtime       : the bus to drive.
    phase1_config : ordered ``HomingMotorConfig`` list for phase 1 on this bus.
    phase2_config : ditto for phase 2.
    tuning        : shared ``ControlTuning``.
    print_lock    : serialises console output between the two bus threads.
    """

    def __init__(
        self,
        runtime: BusRuntime,
        phase1_config: List[HomingMotorConfig],
        phase2_config: List[HomingMotorConfig],
        tuning: ControlTuning,
        print_lock: Optional[threading.Lock] = None,
    ):
        self.runtime = runtime
        self.phase1_config = list(phase1_config)
        self.phase2_config = list(phase2_config)
        self.tuning = tuning
        self.print_lock = print_lock or threading.Lock()

        # Quick lookup: motor_id → its homing recipe.
        self.cfg_by_id: Dict[int, HomingMotorConfig] = {}
        for cfg in self.phase1_config + self.phase2_config:
            self.cfg_by_id[cfg.motor_id] = cfg

    def _log(self, msg: str):
        """Thread-safe print (two buses log concurrently)."""
        with self.print_lock:
            print(msg, flush=True)

    # ── Start-up and holding ─────────────────────────────────────────────────

    def verify_periodic_feedback_and_capture_boot_holds(self):
        """
        Gate: wait for every motor on this bus to start streaming status
        frames, remember their boot positions.  Raises ``RuntimeError`` (with
        the fault summary) on timeout or fault.
        """
        ok = self.runtime.wait_for_periodic_feedback(
            timeout_s=self.tuning.startup_timeout_s,
            poll_period_s=self.tuning.startup_poll_s,
            feedback_timeout_s=self.tuning.feedback_timeout_s,
            bus_silence_timeout_s=self.tuning.bus_silence_timeout_s,
        )
        if not ok:
            raise RuntimeError(self.runtime.get_fault_summary())
        self.runtime.capture_boot_hold_positions()

    def send_idle_hold_once(self):
        """
        Send ONE frame to every motor: hold the boot/homed target with the
        very soft ``safe_idle`` gains.  Called for ~1 s after start-up.
        """
        targets = self.runtime.get_idle_hold_targets()
        self.runtime.command_positions_deg(
            targets,
            kp=self.tuning.safe_idle_kp,
            kd=self.tuning.safe_idle_kd,
            torque=0.0,
        )

    def hold_all_once(self):
        """
        Send ONE hold frame to every motor: homed motors get the stiffer
        ``homed_hold`` gains, un-homed ones the softer ``hold`` gains.
        Used while this bus is idle waiting for the other bus / next phase.
        """
        targets = self.runtime.get_idle_hold_targets()
        states = self.runtime.get_all_state_copies()
        for mid in self.runtime.motor_ids:
            st = states[mid]
            kp = self.tuning.homed_hold_kp if st.homed else self.tuning.hold_kp
            kd = self.tuning.homed_hold_kd if st.homed else self.tuning.hold_kd
            self.runtime.send_position_deg(mid, targets[mid], kp=kp, kd=kd, torque=0.0)

    def run_all_phases(self, stop_event: Optional[threading.Event] = None):
        """
        Stand-alone single-bus homing: verify feedback, soft-hold for 1 s, then
        phase 1 followed by phase 2.  (``main_with_ik.py`` does NOT call this;
        it coordinates two buses itself.)
        """
        if stop_event is None:
            stop_event = threading.Event()

        self.verify_periodic_feedback_and_capture_boot_holds()

        # One second of gentle hold (loop_hz frames at loop_dt spacing).
        for _ in range(int(self.tuning.loop_hz)):
            self.send_idle_hold_once()
            time.sleep(self.tuning.loop_dt)

        if self.phase1_config:
            self._run_phase(
                sequence_ids=[cfg.motor_id for cfg in self.phase1_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE1",
                stop_event=stop_event,
            )

        if self.phase2_config:
            self._run_phase(
                sequence_ids=[cfg.motor_id for cfg in self.phase2_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE2",
                stop_event=stop_event,
            )

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _status_line(self, sequence_ids: List[int], active_id: int, active_symbol: str) -> str:
        """
        One-line progress summary, e.g. ``[can0]  ✅M5:65.0°|+0.10A  🚀M6:…``.
        ✅ homed, ⏸️ waiting, and ``active_symbol`` for the motor in motion.
        """
        states = self.runtime.get_all_state_copies()
        parts = []
        for mid in sequence_ids:
            st = states[mid]
            if mid == active_id:
                symbol = active_symbol
            elif st.homed:
                symbol = "✅"
            else:
                symbol = "⏸️"
            parts.append(f"{symbol}M{mid}:{st.position_deg:.1f}°|{st.current_A:+.2f}A")
        return "  [" + self.runtime.channel + "] " + "  ".join(parts)

    def _refresh_or_raise(self):
        """Run the watchdogs and abort the homing sequence on any fault."""
        self.runtime.refresh_watchdogs(
            feedback_timeout_s=self.tuning.feedback_timeout_s,
            bus_silence_timeout_s=self.tuning.bus_silence_timeout_s,
            fault_on_missing_feedback=True,
        )
        if self.runtime.faulted:
            raise RuntimeError(self.runtime.get_fault_summary())

    def _wait_for_zero_feedback(self, motor_id: int, pre_zero_pos_deg: float):
        """
        After sending "set origin", block until the motor has reported a
        *fresh* position close to 0° (so we know the new zero took effect
        before we start the nudge move relative to it).

        Accepts the zero if a new frame shows |pos| ≤ ``zero_position_tolerance_deg``,
        or — for the case where the joint was already within 5° of 0 before
        zeroing — a slightly looser tolerance (+2°).  Raises if no new frame
        arrives for ``feedback_timeout_s``.
        """
        st = self.runtime.get_state_copy(motor_id)
        last_count = st.feedback_count
        last_new_feedback_time = time.time()
        last_log = time.time()

        while True:
            self._refresh_or_raise()
            st = self.runtime.get_state_copy(motor_id)
            now = time.time()

            # Only trust frames that arrived AFTER the zero command.
            if st.feedback_count > last_count:
                last_count = st.feedback_count
                last_new_feedback_time = now

                if abs(st.position_deg) <= self.tuning.zero_position_tolerance_deg:
                    return

                if abs(pre_zero_pos_deg) <= 5.0 and abs(st.position_deg) <= (self.tuning.zero_position_tolerance_deg + 2.0):
                    return

            if (now - last_new_feedback_time) > self.tuning.feedback_timeout_s:
                raise RuntimeError(
                    f"[{self.runtime.channel}] M{motor_id}: zero feedback stale for "
                    f"{now - last_new_feedback_time:.3f}s"
                )

            if (now - last_log) >= self.tuning.wait_log_period_s:
                last_log = now
                self._log(
                    f"  [{self.runtime.channel}] M{motor_id} zero wait: "
                    f"pos={st.position_deg:+.1f}° age={st.feedback_age*1000:.1f}ms "
                    f"count={st.feedback_count}"
                )

            time.sleep(self.tuning.loop_dt)

    @staticmethod
    def _clamp(x: float, lo: float, hi: float) -> float:
        """Clamp ``x`` into ``[lo, hi]``."""
        return max(lo, min(hi, x))

    def _profile_step(
        self,
        profile: MotionProfileState,
        goal_deg: float,
        vmax_deg_s: float,
        amax_deg_s2: float,
    ) -> MotionProfileState:
        """
        Advance the motion profile by one control tick and return the new one.

        A "bang-bang with braking curve" generator: the desired speed toward
        the goal is the lesser of ``vmax`` and the speed from which we can
        still stop in the remaining distance at ``amax`` (v = √(2·a·d)).  The
        actual commanded speed slews toward that by at most ``amax·dt`` per
        tick.  Result: smooth ramp-up, cruise, ramp-down — no jerk at the
        target, and a sudden new goal is followed gracefully.

        When the next step would cross the goal, snap to the goal and stop.
        """
        dt = self.tuning.loop_dt
        err = goal_deg - profile.cmd_deg

        # Already there and stopped.
        if abs(err) < 1e-6 and abs(profile.vel_deg_s) < 1e-6:
            return MotionProfileState(goal_deg, 0.0)

        direction = 1.0 if err >= 0.0 else -1.0
        brake_speed = math.sqrt(max(0.0, 2.0 * amax_deg_s2 * abs(err)))
        desired_vel = direction * min(vmax_deg_s, brake_speed)

        dv_max = amax_deg_s2 * dt
        new_vel = profile.vel_deg_s + self._clamp(desired_vel - profile.vel_deg_s, -dv_max, dv_max)
        new_cmd = profile.cmd_deg + new_vel * dt

        # Overshoot guard: if goal lies between the old and new command, land on it.
        if (goal_deg - profile.cmd_deg) * (goal_deg - new_cmd) <= 0.0:
            new_cmd = goal_deg
            new_vel = 0.0

        return MotionProfileState(new_cmd, new_vel)

    def _send_mixed_targets(self, active_id: int, active_cmd_deg: float):
        """
        One tick of output for the whole bus: the ``active_id`` motor gets the
        profile command with the strong ``active`` gains; every other motor is
        held at its idle target with the (homed or un-homed) hold gains.
        """
        hold_targets = self.runtime.get_idle_hold_targets()
        states = self.runtime.get_all_state_copies()

        for mid in self.runtime.motor_ids:
            if mid == active_id:
                self.runtime.send_position_deg(
                    mid,
                    active_cmd_deg,
                    kp=self.tuning.active_kp,
                    kd=self.tuning.active_kd,
                    torque=0.0,
                )
            else:
                st = states[mid]
                kp = self.tuning.homed_hold_kp if st.homed else self.tuning.hold_kp
                kd = self.tuning.homed_hold_kd if st.homed else self.tuning.hold_kd
                self.runtime.send_position_deg(
                    mid,
                    hold_targets[mid],
                    kp=kp,
                    kd=kd,
                    torque=0.0,
                )

    # ── Step 6: back off from the stop ───────────────────────────────────────

    def _run_nudge(
        self,
        cfg: HomingMotorConfig,
        sequence_ids: List[int],
        phase_name: str,
        stop_event: threading.Event,
    ):
        """
        Move from the freshly zeroed hard stop to ``cfg.nudge_back_deg`` using
        the gentler nudge profile, wait until the joint is there and still for
        ``nudge_settle_time_s`` (and at least ``min_move_time_s`` has passed),
        then ``mark_homed`` so this pose becomes the motor's hold target.
        """
        _ = phase_name   # kept for a uniform signature / future logging
        motor_id = cfg.motor_id
        nudge_target_deg = cfg.nudge_back_deg

        st0 = self.runtime.get_state_copy(motor_id)
        profile = MotionProfileState(cmd_deg=st0.position_deg, vel_deg_s=0.0)

        self._log(f"  ↩️ [{self.runtime.channel}] M{motor_id} nudge: 0.0° -> {nudge_target_deg:+.1f}°")

        start = time.time()
        last_status = 0.0
        at_target_since = None
        last_wait_log = time.time()

        while True:
            if stop_event.is_set():
                raise RuntimeError(f"[{self.runtime.channel}] stop requested")

            self._refresh_or_raise()

            profile = self._profile_step(
                profile,
                goal_deg=nudge_target_deg,
                vmax_deg_s=self.tuning.nudge_max_vel_deg_s,
                amax_deg_s2=self.tuning.nudge_max_acc_deg_s2,
            )
            self._send_mixed_targets(motor_id, profile.cmd_deg)

            now = time.time()
            st = self.runtime.get_state_copy(motor_id)

            if (now - last_status) >= self.tuning.status_period_s:
                last_status = now
                self._log(self._status_line(sequence_ids, motor_id, "↩️"))

            # "Done" = close to target AND slow, held for nudge_settle_time_s.
            pos_err = abs(st.position_deg - nudge_target_deg)
            vel_ok = abs(st.velocity_raw) <= self.tuning.trigger_velocity_raw_max
            time_ok = (now - start) >= self.tuning.min_move_time_s

            if pos_err <= self.tuning.nudge_position_tolerance_deg and vel_ok:
                if at_target_since is None:
                    at_target_since = now
                elif time_ok and (now - at_target_since) >= self.tuning.nudge_settle_time_s:
                    self.runtime.mark_homed(motor_id, nudge_target_deg)
                    self._log(f"  ↩️ [{self.runtime.channel}] M{motor_id} nudge done -> holding at {nudge_target_deg:+.1f}°")
                    return
            else:
                at_target_since = None

            if (now - last_wait_log) >= self.tuning.wait_log_period_s:
                last_wait_log = now
                self._log(
                    f"  [{self.runtime.channel}] M{motor_id} nudge waiting: "
                    f"pos={st.position_deg:+.1f}° target={nudge_target_deg:+.1f}° "
                    f"err={pos_err:.2f}° cur={st.current_A:+.2f}A "
                    f"fcur={st.filtered_current_A:+.2f}A vel={st.velocity_raw:+.2f}"
                )

            time.sleep(self.tuning.loop_dt)

    # ── Steps 1-5: find the hard stop, then zero + nudge ─────────────────────

    def _run_phase(
        self,
        sequence_ids: List[int],
        hold_at_nudge_ids: Set[int],
        phase_name: str,
        stop_event: threading.Event,
    ):
        """
        Home every motor in ``sequence_ids`` one after the other (the module
        docstring explains the per-motor algorithm).  Blocks until all are
        homed; raises ``RuntimeError`` on fault, stop request, or excessive
        travel.  ``hold_at_nudge_ids`` is accepted for compatibility and ignored.
        """
        _ = hold_at_nudge_ids

        for motor_id in sequence_ids:
            cfg = self.cfg_by_id[motor_id]
            st0 = self.runtime.get_state_copy(motor_id)

            # The search target is RELATIVE to wherever the motor is now.
            phase_start_deg = st0.position_deg
            active_target_deg = phase_start_deg + cfg.target_deg
            profile = MotionProfileState(cmd_deg=phase_start_deg, vel_deg_s=0.0)

            self._log(
                f"\n[{self.runtime.channel}] {phase_name} ▶ M{motor_id} "
                f"rel {cfg.target_deg:+.1f}° from {phase_start_deg:+.1f}° -> {active_target_deg:+.1f}°"
            )

            # ── per-motor search state ──
            last_status = 0.0
            last_wait_log = time.time()
            target_settled_since = None     # when we first sat at the initial target
            current_above_since = None      # when current first exceeded threshold
            stall_since = None              # when the joint first became stationary

            continuous_search_started = False   # step 2 active?
            contact_latched = False             # step 3 active?
            latched_contact_pos_deg = None      # where contact was first seen
            latched_target_deg = None           # the frozen pressing target

            # Direction of creep: +1 / -1 / 0 (from the sign of the config value).
            search_sign = 0.0
            if cfg.search_direction_deg > 0.0:
                search_sign = 1.0
            elif cfg.search_direction_deg < 0.0:
                search_sign = -1.0

            while True:
                if stop_event.is_set():
                    raise RuntimeError(f"[{self.runtime.channel}] stop requested")

                self._refresh_or_raise()
                now = time.time()

                # Advance the profile toward the (possibly moving) target and
                # transmit it (other motors on the bus are held still).
                profile = self._profile_step(
                    profile,
                    goal_deg=active_target_deg,
                    vmax_deg_s=self.tuning.search_max_vel_deg_s,
                    amax_deg_s2=self.tuning.search_max_acc_deg_s2,
                )
                self._send_mixed_targets(motor_id, profile.cmd_deg)

                st = self.runtime.get_state_copy(motor_id)

                if (now - last_status) >= self.tuning.status_period_s:
                    last_status = now
                    self._log(self._status_line(sequence_ids, motor_id, "🚀"))

                # Safety: never wander further than max_search_travel_deg.
                travel_deg = abs(st.position_deg - phase_start_deg)
                if travel_deg > self.tuning.max_search_travel_deg:
                    raise RuntimeError(
                        f"[{self.runtime.channel}] M{motor_id}: search travel exceeded "
                        f"{self.tuning.max_search_travel_deg:.1f}°"
                    )

                # Signals used to detect the stop.
                current_abs = abs(st.filtered_current_A)
                current_ok = current_abs >= cfg.trigger_current_A       # pressing hard
                vel_abs = abs(st.velocity_raw)
                velocity_ok = vel_abs <= self.tuning.trigger_velocity_raw_max   # slow enough to be "arrived"
                stalled_ok = vel_abs <= self.tuning.contact_stall_vel_max       # essentially not moving

                if not contact_latched and current_ok:
                    # STEP 3 — first sign of contact: freeze the target just
                    # beyond here and reset the profile to the real position
                    # so the motor holds a steady push instead of ramping.
                    contact_latched = True
                    latched_contact_pos_deg = st.position_deg
                    latched_target_deg = st.position_deg + search_sign * self.tuning.contact_hold_offset_deg
                    active_target_deg = latched_target_deg
                    profile = MotionProfileState(cmd_deg=st.position_deg, vel_deg_s=0.0)
                    current_above_since = now
                    stall_since = now if stalled_ok else None
                    self._log(
                        f"  [{self.runtime.channel}] M{motor_id} contact latched: "
                        f"freeze target at {active_target_deg:+.1f}° "
                        f"(fcur={st.filtered_current_A:+.2f}A vel={st.velocity_raw:+.2f})"
                    )

                elif contact_latched:
                    # STEP 4 — decide whether this is a real hard stop.
                    active_target_deg = latched_target_deg

                    # Track how long each confirming condition has held.
                    if current_ok:
                        if current_above_since is None:
                            current_above_since = now
                    else:
                        current_above_since = None

                    if stalled_ok:
                        if stall_since is None:
                            stall_since = now
                    else:
                        stall_since = None

                    # Confirmed by sustained high current  → ZERO + NUDGE (steps 5-6)
                    if current_above_since is not None and (now - current_above_since) >= self.tuning.trigger_confirm_s:
                        self._log(
                            f"  ✅ [{self.runtime.channel}] M{motor_id}: "
                            f"{abs(st.filtered_current_A):.2f}A >= {cfg.trigger_current_A:.2f}A "
                            f"-> zero at {st.position_deg:+.1f}°"
                        )
                        pre_zero_pos_deg = st.position_deg
                        self.runtime.zero_motor(motor_id, permanent=False)
                        self._wait_for_zero_feedback(motor_id, pre_zero_pos_deg)
                        self._run_nudge(cfg, sequence_ids, phase_name, stop_event)
                        break

                    # Confirmed by sustained stall (low speed)  → ZERO + NUDGE
                    if stall_since is not None and (now - stall_since) >= self.tuning.trigger_confirm_s:
                        self._log(
                            f"  ✅ [{self.runtime.channel}] M{motor_id}: "
                            f"stalled at hard stop (vel={st.velocity_raw:+.2f}) "
                            f"-> zero at {st.position_deg:+.1f}°"
                        )
                        pre_zero_pos_deg = st.position_deg
                        self.runtime.zero_motor(motor_id, permanent=False)
                        self._wait_for_zero_feedback(motor_id, pre_zero_pos_deg)
                        self._run_nudge(cfg, sequence_ids, phase_name, stop_event)
                        break

                    # False alarm: the joint moved away AND current fell →
                    # release the latch and resume searching.
                    if (
                        latched_contact_pos_deg is not None and
                        abs(st.position_deg - latched_contact_pos_deg) >= self.tuning.contact_release_move_deg and
                        not current_ok
                    ):
                        contact_latched = False
                        latched_contact_pos_deg = None
                        latched_target_deg = None
                        current_above_since = None
                        stall_since = None

                # STEP 2 — decide when to start creeping, and creep.
                if not continuous_search_started:
                    near_target = abs(st.position_deg - active_target_deg) <= self.tuning.advance_target_tolerance_deg
                    if near_target and velocity_ok:
                        if target_settled_since is None:
                            target_settled_since = now
                        elif (now - target_settled_since) >= self.tuning.advance_settle_time_s:
                            continuous_search_started = True
                            target_settled_since = None
                    else:
                        target_settled_since = None
                else:
                    if not contact_latched:
                        # Slide the target onward at constant speed.
                        active_target_deg += (
                            search_sign
                            * self.tuning.continuous_search_vel_deg_s
                            * self.tuning.loop_dt
                        )

                if (now - last_wait_log) >= self.tuning.wait_log_period_s:
                    last_wait_log = now
                    self._log(
                        f"  [{self.runtime.channel}] M{motor_id} search waiting: "
                        f"pos={st.position_deg:+.1f}° target={active_target_deg:+.1f}° "
                        f"travel={travel_deg:.1f}° cur={st.current_A:+.2f}A "
                        f"fcur={st.filtered_current_A:+.2f}A vel={st.velocity_raw:+.2f} "
                        f"latched={contact_latched}"
                    )

                time.sleep(self.tuning.loop_dt)
