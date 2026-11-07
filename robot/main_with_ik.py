#!/usr/bin/env python3
"""
main_with_ik.py — BYTE-01 motor runtime  (THE program that drives the robot)
══════════════════════════════════════════════════════════════════════════════

Run this on the Raspberry Pi, from the ``robot/`` directory, with both CAN
buses up (``system/can-setup.sh``):

    python3 main_with_ik.py y      # home all 12 motors first, then go live
    python3 main_with_ik.py n      # skip homing, go live (see warning below)
    python3 main_with_ik.py        # same as "n"

WHAT IT DOES, IN ORDER
    ┌────────────────────────────────────────────────────────────────────────┐
    │ 1. open can0 (motors 1-6) and can1 (motors 7-12) → two ``BusRuntime``  │
    │ 2. wait until all 12 motors stream status frames; remember boot pose   │
    │ 3. hold everything gently for 1 s                                      │
    │ 4. HOMING (only with "y"): two threads, one per bus, run 2 phases in   │
    │    lock-step — drive each joint into its hard stop, zero it, back off  │
    │ 5. load per-motor settings from config/motor_config.json               │
    │ 6. start CSV loggers (motor current and temperature, 5 Hz each)        │
    │ 7. start a SEPARATE PROCESS that listens on a TCP socket for targets   │
    │ 8. LIVE CONTROL loop (below) until Ctrl-C or a fault                   │
    │ 9. shut down: every motor is told to go limp ("disable")              │
    └────────────────────────────────────────────────────────────────────────┘

ARCHITECTURE (processes / threads)

      other machine / process                     this program
    ┌────────────────────────┐   TCP :50000   ┌──────────────────────────────┐
    │ state_manager_server   │ ─────────────▶ │ socket_listener_process      │ (separate
    │ mpu_leveling, tests…   │  pickled dict  │   validate → mp.Queue(1)     │  PROCESS)
    └────────────────────────┘                └──────────────┬───────────────┘
                                                             │ latest packet only
                                              ┌──────────────▼───────────────┐
                                              │ run_post_homing_live_control │ (main
                                              │  100 Hz loop: IK → limit →   │  thread)
                                              │  MIT frame to each motor     │
                                              └───────┬─────────────┬────────┘
                                                      │             │
                                      BusRuntime can0 ┘             └ BusRuntime can1
                          (+ listener thread each)        (+ listener thread each)
                          + CurrentLogger thread, TempLogger thread

    The socket reader lives in its own process so network I/O and un-pickling
    can never delay the 100 Hz motor loop.

THE PACKET PROTOCOL (what senders must transmit)
    One TCP connection per packet: connect, send, close.  The payload is a
    ``pickle.dumps`` of::

        {"fl": [dx, dy, dz], "bl": [...], "fr": [...], "br": [...],
         "speed": 400.0}                 # optional, deg/s override

    * Values are foot-position OFFSETS in cm from ``SIT_COORDS`` (so all zeros
      = sitting, ``dz = -14`` ≈ standing).
    * All four legs are required; ``speed`` is optional (default
      ``MAX_LIVE_DEG_PER_S``).
    * Only the newest packet matters: the queue has length 1 and stale packets
      are discarded, so a slow loop can't build up a backlog.

LIVE CONTROL PIPELINE (per 10 ms tick)
    offsets + SIT_COORDS ─▶ IK (angles relative to sit pose) ─▶ flip sign
    (``flipped``) ─▶ × gear ratio ─▶ per-motor rate limiter (deg/s) ─▶
    MIT position command with the motor's kp/kd.

⚠ SAFETY NOTES — READ BEFORE RUNNING
    * SHUTDOWN = LIMP.  On exit or any fault every motor is "disabled" (zero
      current brake).  A standing robot will collapse; support it or sit it
      first.
    * ``n`` (skip homing) assumes the motors have NOT been power-cycled since
      the last homing and that the robot is physically sitting: the runtime
      starts from the motors' current angles and eases them to its idea of
      0° (= sit).  After a power cycle that idea is wrong.
    * The socket accepts **pickle** from the network with no authentication.
      Anyone who can reach port 50000 can run arbitrary code as this process.
      Keep it on a trusted network (see docs/KNOWN_ISSUES.md).
    * ``SOCKET_HOST`` (config/robot_config.py) is a *bind* address; if the Pi's
      IP changed, binding fails.
    * Current/temperature log files are overwritten every run.
"""
# temperature_C for temperature
import sys
import json
import multiprocessing as mp
import pickle
import queue
import socket
import threading
import time
from multiprocessing.queues import Queue as MpQueue
from multiprocessing.synchronize import Event as MpEvent
from typing import Any, Dict, List, Optional, Tuple

from lib.can_runtime import BusRuntime
from lib.homing_controller import BusHomingController, ControlTuning, HomingMotorConfig
from lib.inverse_kinematics import calculate_each_motor_angles
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER, SIT_COORDS, SIT_TARGETS_DEG, MAX_LIVE_DEG_PER_S

# Size of the hand-off queue between the socket process and the control loop.
# 1 = "mailbox": only the most recent target is ever kept.
LIVE_QUEUE_MAXSIZE = 1
# Relative path: the program must be started from the robot/ directory.
MOTOR_CONFIG_PATH = "config/motor_config.json"



# Type aliases used in signatures below.
LegPayload = Dict[str, List[float]]                     # {"fl": [x, y, z], ...}
MotorCommandConfig = Dict[int, Dict[str, float | bool]] # {motor_id: {kp, kd, flipped, gear_ratio}}

# Which three motors belong to which leg: [hip, thigh, knee].
LEG_TO_MOTOR_IDS: Dict[str, List[int]] = {
    "fl": [1,  2,  3],
    "bl": [4,  5,  6],
    "fr": [7,  8,  9],
    "br": [10, 11, 12],
}


# CSV telemetry logs (relative paths → written inside robot/logs/).
CURRENT_LOG_PATH = "logs/motor_currents.csv"
CURRENT_LOG_HZ = 5.0
CURRENT_LOG_DT = 1.0 / CURRENT_LOG_HZ

TEMP_LOG_PATH = "logs/motor_temps.csv"
TEMP_LOG_HZ = 5.0
TEMP_LOG_DT = 1.0 / TEMP_LOG_HZ

# ─── CLI ──────────────────────────────────────────────────────────────────────

def parse_calibration_flag() -> bool:
    """
    Read the single optional command-line argument.

    ``y`` → run homing (returns True);  ``n`` / missing → skip (False).
    Any other value prints a warning and is treated as ``n``.
    """
    if len(sys.argv) < 2:
        return False
    arg = sys.argv[1].strip().lower()
    if arg == "y":
        return True
    elif arg == "n":
        return False
    else:
        print(f"Unknown argument '{arg}'. Use 'y' or 'n'. Defaulting to False.")
        return False


# ─── HOMING TABLE ─────────────────────────────────────────────────────────────
# Which motors are homed, in which order, on which bus.  Two phases per bus
# so that two legs (one on each bus) home at the same time while the other
# two legs stay put.  See lib/homing_controller.py for what each number means.
#
#   Phase 1:  can1 → FR leg (8 → 9 → 7)  ‖  can0 → BL leg (5 → 6 → 4)
#   Phase 2:  can1 → BR leg (11→12→10)   ‖  can0 → FL leg (2 → 3 → 1)
#
# The signs/angles differ per joint and side because each joint hits its
# mechanical stop at a different end of its travel.
#
# Homing Uses Raw Hardware Values - No transformation to these angles is done downstream
CAN_CONFIG: Dict[str, Dict[str, List[HomingMotorConfig]]] = {
    "can0": {
        # HomingMotorConfig(motor_id, target_deg, search_direction_deg, trigger_current_A, nudge_back_deg)
        # ------------------------------------------------------------------------------------------------
        # motor_id             : The physical CAN ID of the motor.
        # target_deg           : The initial relative angle to travel towards to find the hard stop.
        # search_direction_deg : The direction to keep creeping if the stop isn't found immediately (-1.0 = negative, 1.0 = positive).
        # trigger_current_A    : The current spike threshold (Amps) that tells the script it has physically hit the hard stop.
        # nudge_back_deg       : The raw encoder degrees to back away from the hard stop before locking in the final 0.0 position.
        "phase1": [
            HomingMotorConfig(5, -60.0, -1.0, 4.5, 65.0),
            HomingMotorConfig(6, 10.0, 1.0, 3.5, -38.0),
            HomingMotorConfig(4, 60.0, 1.0, 4.5, -90.0),
        ],
        "phase2": [
            HomingMotorConfig(2, -60.0, -1.0, 4.5, 65.0),
            HomingMotorConfig(3, 10.0, 1.0, 3.5, -38.0),
            HomingMotorConfig(1, 60.0, 1.0, 4.5, -85.0),
        ],
    },
    "can1": {
        "phase1": [
            HomingMotorConfig(8, 60.0, 1.0, 4.5, -65.0),
            HomingMotorConfig(9, -10.0, -1.0, 3.5, 38.0),
            HomingMotorConfig(7, -60.0, -1.0, 4.5, 80.0),
        ],
        "phase2": [
            HomingMotorConfig(11, 60.0, 1.0, 4.5, -65.0),
            HomingMotorConfig(12, -10.0, -1.0, 4.0, 38.0),
            HomingMotorConfig(10, -60.0, -1.0, 4.5, 90.0),
        ],
    },
}


def collect_motor_ids(bus_cfg: Dict[str, List[HomingMotorConfig]]) -> List[int]:
    """Sorted, de-duplicated list of every motor ID in a bus's phase1+phase2 tables."""
    ids = []
    for phase_name in ("phase1", "phase2"):
        ids.extend(cfg.motor_id for cfg in bus_cfg[phase_name])
    return sorted(set(ids))


# ─── Homing coordination between the two bus threads ─────────────────────────

def hold_until_both_phase1_done(
    controller: BusHomingController,
    my_done_event: threading.Event,
    other_done_event: threading.Event,
    stop_event: threading.Event,
):
    """
    A two-party barrier between the bus threads, used between phase 1 and 2.

    Marks *this* bus as finished with phase 1, then keeps its motors held in
    place (sending hold frames, so the watchdogs stay happy) until the *other*
    bus has finished too.  Returns True to continue to phase 2, or False if a
    stop was requested while waiting.
    """
    my_done_event.set()

    while not stop_event.is_set():
        if other_done_event.is_set():
            return True
        controller.hold_all_once()
        time.sleep(controller.tuning.loop_dt)

    return False


def hold_until_takeover(
    controller: BusHomingController,
    takeover_event: threading.Event,
    stop_event: threading.Event,
):
    """
    After a bus finished BOTH phases, keep holding its motors until the main
    thread signals that it will take over control (``takeover_event``).
    This avoids a gap in command frames, which would trip the motors' timeout.
    """
    while not stop_event.is_set() and not takeover_event.is_set():
        controller.hold_all_once()
        time.sleep(controller.tuning.loop_dt)


def controller_thread_entry(
    controller: BusHomingController,
    my_phase1_done: threading.Event,
    other_phase1_done: threading.Event,
    my_phase2_done: threading.Event,
    takeover_event: threading.Event,
    stop_event: threading.Event,
    errors: List[str],
):
    """
    Thread body: run the whole homing sequence for ONE bus.

        phase 1  →  barrier (wait for other bus)  →  phase 2  →  hold until
        main thread takes over.

    Any exception is recorded in ``errors`` and *every* event is set so the
    other thread and the main thread are released instead of hanging forever.
    """
    channel = controller.runtime.channel
    try:
        if controller.phase1_config:
            controller._run_phase(
                sequence_ids=[cfg.motor_id for cfg in controller.phase1_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE1",
                stop_event=stop_event,
            )

        if not hold_until_both_phase1_done(
            controller=controller,
            my_done_event=my_phase1_done,
            other_done_event=other_phase1_done,
            stop_event=stop_event,
        ):
            return

        if stop_event.is_set():
            return

        if controller.phase2_config:
            controller._run_phase(
                sequence_ids=[cfg.motor_id for cfg in controller.phase2_config],
                hold_at_nudge_ids=set(),
                phase_name="PHASE2",
                stop_event=stop_event,
            )

        my_phase2_done.set()
        hold_until_takeover(
            controller=controller,
            takeover_event=takeover_event,
            stop_event=stop_event,
        )

    except Exception as exc:
        # Fail loudly and release everyone who might be waiting on us.
        errors.append(f"{channel}: {exc}")
        stop_event.set()
        my_phase1_done.set()
        other_phase1_done.set()
        my_phase2_done.set()
        takeover_event.set()


# ─── Per-motor command settings ───────────────────────────────────────────────

def load_motor_command_config(path: str) -> MotorCommandConfig:
    """
    Load and validate ``config/motor_config.json``.

    Expected shape: ``{"motors": {"1": {"kp", "kd", "flipped", "gear_ratio"}, … "12": …}}``
        kp / kd      MIT stiffness / damping used in live control
        flipped      True if the motor is mounted so positive IK = negative motor angle
        gear_ratio   output-to-motor multiplier (1.0 direct; 1.5 on the knees,
                     whose motors drive the joint through an extra reduction)
    Raises ``ValueError`` if the file lacks any of motors 1..12.
    """
    with open(path, "r", encoding="utf-8") as fh:
        raw = json.load(fh)

    motors = raw.get("motors")
    if not isinstance(motors, dict):
        raise ValueError("motor_config.json must contain a top-level 'motors' object")

    config: MotorCommandConfig = {}
    for motor_id in range(1, 13):
        entry = motors.get(str(motor_id))
        if not isinstance(entry, dict):
            raise ValueError(f"motor_config.json missing config for motor {motor_id}")

        config[motor_id] = {
            "kp":         float(entry["kp"]),
            "kd":         float(entry["kd"]),
            "flipped":    bool(entry["flipped"]),
            "gear_ratio": float(entry["gear_ratio"]),
        }

    return config


# ─── Packet handling ─────────────────────────────────────────────────────────

def validate_leg_payload(payload: Any) -> Tuple[LegPayload, Optional[float]]:
    """
    Validates the incoming socket payload.

    Returns:
        (normalized_leg_payload, speed_override)

    speed_override is None if the sender did not include a "speed" key,
    in which case run_post_homing_live_control falls back to MAX_LIVE_DEG_PER_S.
    tapping_gait.py sends "speed": 900.0 to unlock fast tap movements.

    Raises ``ValueError`` (caught by the socket process, which drops the
    packet) if the payload isn't a dict, a leg is missing, or a leg doesn't
    contain exactly three numbers.  Values are coerced to ``float``.
    """
    if not isinstance(payload, dict):
        raise ValueError("payload must be a dict with keys: fl, bl, fr, br")

    normalized: LegPayload = {}
    for leg in LEG_ORDER:
        if leg not in payload:
            raise ValueError(f"missing leg '{leg}' in payload")

        values = payload[leg]
        if not isinstance(values, (list, tuple)):
            raise ValueError(f"payload['{leg}'] must be a list or tuple")

        if len(values) != 3:
            raise ValueError(f"payload['{leg}'] must contain exactly 3 values")

        normalized[leg] = [float(v) for v in values]

    # Optional per-packet speed override — absent means use the global default
    speed_override: Optional[float] = None
    if "speed" in payload:
        speed_override = float(payload["speed"])

    return normalized, speed_override


def flatten_leg_payload(payload: LegPayload) -> List[float]:
    """Concatenate the four legs' (x, y, z) into one 12-element list in LEG_ORDER. (Currently unused.)"""
    flat: List[float] = []
    for leg in LEG_ORDER:
        flat.extend(payload[leg])
    return flat


# def transform_live_angles(
#     raw_angles_deg: List[float],
#     motor_config: MotorCommandConfig,
# ) -> Dict[int, float]:
#     if len(raw_angles_deg) != 12:
#         raise ValueError(f"expected 12 motor angles, received {len(raw_angles_deg)}")

#     transformed: Dict[int, float] = {}
#     for motor_id, raw_angle_deg in enumerate(raw_angles_deg, start=1):
#         cfg = motor_config[motor_id]
#         angle_deg = float(raw_angle_deg)
#         # if bool(cfg["flipped"]):
#         #     angle_deg = -angle_deg
#         angle_deg *= float(cfg["gear_ratio"])
#         transformed[motor_id] = angle_deg

#     return transformed



def convert_coords_to_motor_targets(
    leg_payload: LegPayload,
    motor_config: MotorCommandConfig,
) -> Dict[int, float]:
    """
    Converts foot coordinates (x, y, z) per leg into final motor angles.
    Pipeline: IK (relative to sit) → flip → gear_ratio

    ``leg_payload`` holds ABSOLUTE foot positions (SIT_COORDS already added).
    Returns ``{motor_id: target_deg}`` for all 12 motors, in each motor's own
    zeroed frame, ready to be rate-limited and sent.
    May raise ``ValueError`` from the IK if a target is unreachable.
    """
    transformed: Dict[int, float] = {}

    for leg in LEG_ORDER:
        x, y, z = leg_payload[leg]
        # Use the new IK function directly
        m1, m2, m3 = calculate_each_motor_angles(x, y, z, leg)
        raw_angles = [m1, m2, m3]

        for i, motor_id in enumerate(LEG_TO_MOTOR_IDS[leg]):
            cfg = motor_config[motor_id]
            angle = raw_angles[i]
            if bool(cfg["flipped"]):
                angle = -angle
            angle *= float(cfg["gear_ratio"])
            transformed[motor_id] = angle

    return transformed


def runtime_for_motor_id(
    motor_id: int,
    rt0: BusRuntime,
    rt1: BusRuntime,
) -> BusRuntime:
    """Return the bus that owns ``motor_id``: 1-6 → can0 (rt0), 7-12 → can1 (rt1)."""
    if 1 <= motor_id <= 6:
        return rt0
    if 7 <= motor_id <= 12:
        return rt1
    raise ValueError(f"invalid motor_id {motor_id}")


def send_live_targets(
    rt0: BusRuntime,
    rt1: BusRuntime,
    targets_deg: Dict[int, float],
    motor_config: MotorCommandConfig,
):
    """Send one MIT position frame to each of the 12 motors using its configured kp/kd."""
    for motor_id in range(1, 13):
        cfg = motor_config[motor_id]
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        runtime.send_position_deg(
            motor_id,
            targets_deg[motor_id],
            kp=float(cfg["kp"]),
            kd=float(cfg["kd"]),
            torque=0.0,
        )


def drain_latest_packet(
    live_queue: MpQueue,
) -> Optional[Tuple[LegPayload, Optional[float]]]:
    """
    Drains the queue and returns the most recent (leg_payload, speed_override) tuple,
    or None if the queue was empty.
    """
    latest = None
    while True:
        try:
            latest = live_queue.get_nowait()
        except queue.Empty:
            break
    return latest


def limit_target_step(
    current_cmd_deg: float,
    requested_deg: float,
    max_deg_per_s: float,
    dt: float,
) -> float:
    """
    Slew-rate limiter: move the *commanded* angle toward ``requested_deg`` by at
    most ``max_deg_per_s * dt`` this tick.

    This is the main safety valve for live control: no matter how far away or
    how abruptly a sender asks for a target, no joint is ever commanded to move
    faster than the speed limit.
    """
    max_step = max_deg_per_s * dt
    delta = requested_deg - current_cmd_deg

    if delta > max_step:
        return current_cmd_deg + max_step
    if delta < -max_step:
        return current_cmd_deg - max_step
    return requested_deg


def initialize_live_command_state(
    rt0: BusRuntime,
    rt1: BusRuntime,
) -> Dict[int, float]:
    """
    Seed the rate limiter with where each motor physically is right now, so the
    first command is a small step from reality rather than a jump.
    """
    live_cmd_deg: Dict[int, float] = {}

    for motor_id in range(1, 13):
        runtime = runtime_for_motor_id(motor_id, rt0, rt1)
        st = runtime.get_state_copy(motor_id)
        live_cmd_deg[motor_id] = st.position_deg

    return live_cmd_deg


# ─── Telemetry loggers (background threads) ──────────────────────────────────

def current_logger_thread_entry(
    rt0: BusRuntime,
    rt1: BusRuntime,
    stop_event: threading.Event,
    log_path: str = CURRENT_LOG_PATH,
    log_hz: float = CURRENT_LOG_HZ,
) -> None:
    """
    Reads current (Amps) from all 12 motors at `log_hz` Hz and appends
    every sample as a CSV row to `log_path`.

    CSV columns:
        timestamp_s, m1_a, m2_a, ..., m12_a

    NOTE: despite "appends", the file is opened with mode "w", so each program
    start TRUNCATES the previous log (the header-if-empty check below is
    therefore always true).  Copy a log elsewhere if you want to keep it.
    ``cur_test/motor_current_plotter.py`` can plot these CSVs.
    """
    log_dt = 1.0 / log_hz
    motor_ids = list(range(1, 13))

    try:
        with open(log_path, "w", buffering=1, encoding="utf-8") as fh:
            fh.seek(0, 2)
            if fh.tell() == 0:
                header = "timestamp_s," + ",".join(f"m{i}_a" for i in motor_ids)
                fh.write(header + "\n")

            print(
                f"[CurrentLogger] Logging {log_hz:.0f} Hz motor currents -> {log_path}",
                flush=True,
            )

            while not stop_event.is_set():
                t_start = time.monotonic()
                ts = time.time()

                currents: List[float] = []
                for motor_id in motor_ids:
                    runtime = runtime_for_motor_id(motor_id, rt0, rt1)
                    try:
                        state = runtime.get_state_copy(motor_id)
                        currents.append(round(state.current_A, 4))
                    except Exception:
                        currents.append(float("nan"))

                row = f"{ts:.4f}," + ",".join(str(c) for c in currents)
                fh.write(row + "\n")

                # Sleep only for the remainder of the period → steady rate.
                elapsed = time.monotonic() - t_start
                sleep_for = log_dt - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

    except Exception as exc:
        print(f"[CurrentLogger] Fatal error: {exc}", flush=True)


def temp_logger_thread_entry(
    rt0: BusRuntime,
    rt1: BusRuntime,
    stop_event: threading.Event,
    log_path: str = TEMP_LOG_PATH,
    log_hz: float = TEMP_LOG_HZ,
) -> None:
    """
    Reads temperature (°C) from all 12 motors at `log_hz` Hz and appends
    every sample as a CSV row to `log_path`.

    CSV columns:
        timestamp_s, m1_c, m2_c, ..., m12_c

    (Same "w"-mode truncation caveat as the current logger.)
    """
    log_dt = 1.0 / log_hz
    motor_ids = list(range(1, 13))

    try:
        with open(log_path, "w", buffering=1, encoding="utf-8") as fh:
            fh.seek(0, 2)
            if fh.tell() == 0:
                header = "timestamp_s," + ",".join(f"m{i}_c" for i in motor_ids)
                fh.write(header + "\n")

            print(
                f"[TempLogger] Logging {log_hz:.0f} Hz motor temperatures -> {log_path}",
                flush=True,
            )

            while not stop_event.is_set():
                t_start = time.monotonic()
                ts = time.time()

                temps: List[float] = []
                for motor_id in motor_ids:
                    runtime = runtime_for_motor_id(motor_id, rt0, rt1)
                    try:
                        state = runtime.get_state_copy(motor_id)
                        temps.append(round(state.temperature_C, 4))
                    except Exception:
                        temps.append(float("nan"))

                row = f"{ts:.4f}," + ",".join(str(t) for t in temps)
                fh.write(row + "\n")

                elapsed = time.monotonic() - t_start
                sleep_for = log_dt - elapsed
                if sleep_for > 0:
                    time.sleep(sleep_for)

    except Exception as exc:
        print(f"[TempLogger] Fatal error: {exc}", flush=True)


# ─── Network receiver (runs in its own process) ──────────────────────────────

def socket_listener_process(
    dest_queue: MpQueue,
    stop_event: MpEvent,
    port: int = SOCKET_PORT,
    host: str = SOCKET_HOST,
):
    """
    Process body: accept TCP connections, decode one packet per connection and
    publish the newest valid one to ``dest_queue``.

    Per connection: read until the client closes → ``pickle.loads`` →
    ``validate_leg_payload`` → clear the queue → put (packet, speed_override).
    Malformed packets are logged and dropped; the server keeps running.

    ⚠ ``pickle.loads`` on network data is unsafe against untrusted senders
    (arbitrary code execution).  Only run on a network you trust.

    Short socket timeouts (0.2 s) let the loop notice ``stop_event`` promptly.
    """
    print(f"[Socket Process {mp.current_process().pid}] Starting on {host}:{port}", flush=True)

    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.settimeout(0.2)

    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Socket Process {mp.current_process().pid}] Listening", flush=True)

        while not stop_event.is_set():
            try:
                conn, addr = server.accept()
            except socket.timeout:
                continue
            except OSError:
                if stop_event.is_set():
                    break
                raise

            # Receive the whole message (client closes the socket when done).
            with conn:
                conn.settimeout(0.2)
                data = bytearray()

                while not stop_event.is_set():
                    try:
                        chunk = conn.recv(4096)
                    except socket.timeout:
                        continue

                    if not chunk:
                        break
                    data.extend(chunk)

            if not data:
                continue

            try:
                payload = pickle.loads(bytes(data))
                # validate_leg_payload now returns (leg_payload, speed_override)
                packet, speed_override = validate_leg_payload(payload)
            except Exception as exc:
                print(f"[Socket Process] Dropping invalid packet from {addr}: {exc}", flush=True)
                continue

            # Drain stale items so only the latest command is in the queue
            while True:
                try:
                    dest_queue.get_nowait()
                except queue.Empty:
                    break
                except Exception:
                    break

            try:
                # Queue carries (leg_payload, speed_override) together
                dest_queue.put_nowait((packet, speed_override))
            except Exception:
                pass

    except Exception as exc:
        print(f"[Socket Process] Server error: {exc}", flush=True)
        raise
    finally:
        server.close()
        print(f"[Socket Process {mp.current_process().pid}] Shutdown", flush=True)

# ─── Live control loop ───────────────────────────────────────────────────────

def run_post_homing_live_control(
    ctrl0: BusHomingController,
    ctrl1: BusHomingController,
    rt0: BusRuntime,
    rt1: BusRuntime,
    tuning: ControlTuning,
    stop_event: threading.Event,
    live_queue: MpQueue,
    motor_config: MotorCommandConfig,
    did_homing: bool = True     # <-- NEW ARGUMENT
):
    """
    The 100 Hz heart of the robot.  Runs on the main thread until ``stop_event``
    is set or a fault raises ``RuntimeError``.

    If ``did_homing`` is True it first performs the **sit-down + re-zero**
    sequence (steps 1-3 below); otherwise it skips straight to step 4.

      1. HOLD     keep the homed "nudge" poses for 1 s (let things settle).
      2. SIT      slew every motor to ``SIT_TARGETS_DEG`` (raw hardware angles)
                  at ``MAX_LIVE_DEG_PER_S``; done when all are within 0.001°.
      3. ZERO     send a *temporary* set-origin to all 12 motors: the sitting
                  pose is now angle 0 for every joint.  (Lost on power cycle.)
      4. LIVE     each tick: refresh watchdogs → take the newest socket packet
                  (if any) → IK → rate-limit → send.  With no packet yet, the
                  targets are 0 = hold the sit pose.

    Every iteration raises ``RuntimeError`` if either bus latched a fault.
    """
    first_live_packet_seen = False

    # Grabs the current physical positions
    live_cmd_deg: Dict[int, float] = initialize_live_command_state(rt0, rt1)
    current_speed_limit: float = MAX_LIVE_DEG_PER_S

    if did_homing:
        print("\n" + "=" * 70)
        print("🎯 POST-HOMING LIVE CONTROL READY")
        print("   1. Holding at homed nudge positions for 1 second...")
        print("   2. Smoothly transitioning to default sitting posture...")
        print("   3. Zeroing all motors at the sit posture (New 0.0)...")
        print(f"   4. Waiting for grouped socket targets on {SOCKET_HOST}:{SOCKET_PORT}")
        print("=" * 70)

        # --- PHASE 1: HOLD AT NUDGE FOR 1 SECOND ---
        hold_start = time.time()
        while (time.time() - hold_start) < 1.0 and not stop_event.is_set():
            rt0.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
            rt1.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
            if rt0.faulted: raise RuntimeError(rt0.get_fault_summary())
            if rt1.faulted: raise RuntimeError(rt1.get_fault_summary())

            ctrl0.hold_all_once()
            ctrl1.hold_all_once()
            time.sleep(tuning.loop_dt)

        # --- PHASE 2: MOVE TO SITTING TARGETS ---
        print("🛋️  1 second elapsed. Moving to default sitting angles...")
        arrived_at_sit = False
        while not arrived_at_sit and not stop_event.is_set():
            rt0.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
            rt1.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
            if rt0.faulted: raise RuntimeError(rt0.get_fault_summary())
            if rt1.faulted: raise RuntimeError(rt1.get_fault_summary())

            arrived_at_sit = True
            for motor_id in range(1, 13):
                live_cmd_deg[motor_id] = limit_target_step(
                    current_cmd_deg=live_cmd_deg[motor_id],
                    requested_deg=SIT_TARGETS_DEG[motor_id],
                    max_deg_per_s=MAX_LIVE_DEG_PER_S * motor_config[motor_id]["gear_ratio"],
                    dt=tuning.loop_dt,
                )
                if abs(live_cmd_deg[motor_id] - SIT_TARGETS_DEG[motor_id]) > 0.001:
                    arrived_at_sit = False

            send_live_targets(rt0, rt1, live_cmd_deg, motor_config)
            time.sleep(tuning.loop_dt)

        # --- PHASE 3: ZERO ALL MOTORS ---
        print("🔄 Reached sitting angles. Triggering temporary zero on all motors...")
        for motor_id in range(1, 13):
            runtime = runtime_for_motor_id(motor_id, rt0, rt1)
            runtime.zero_motor(motor_id, permanent=False)
            live_cmd_deg[motor_id] = 0.0

        # time.sleep(1.0)
        print("✅ Motors successfully zeroed. Holding sit position and listening for socket data.")

    else:
        # --- FAST TRACK (HOMING SKIPPED) ---
        print("\n" + "=" * 70)
        print("🎯 POST-HOMING LIVE CONTROL READY (HOMING SKIPPED)")
        print("   1. Assuming motors are already homed and sitting at 0.0.")
        print(f"   2. Waiting for grouped socket targets on {SOCKET_HOST}:{SOCKET_PORT}")
        print("=" * 70)
        # live_cmd_deg already contains the current physical angles (which should be ~0.0).
        # We don't zero them, we just let Phase 4 take over and gently hold them at true 0.0.

    # Start our live tracking at 0.0 for all motors
    last_live_targets_deg = {i: 0.0 for i in range(1, 13)}

    # Dynamically build current_leg_coords from our central SIT_COORDS.
    current_leg_coords = {leg: list(coords) for leg, coords in SIT_COORDS.items()}

    # --- PHASE 4: LIVE SOCKET CONTROL ---
    while not stop_event.is_set():
        rt0.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
        rt1.refresh_watchdogs(tuning.feedback_timeout_s, tuning.bus_silence_timeout_s)
        if rt0.faulted: raise RuntimeError(rt0.get_fault_summary())
        if rt1.faulted: raise RuntimeError(rt1.get_fault_summary())

        # Check for new socket commands
        item = drain_latest_packet(live_queue)
        if item is not None:
            leg_packet, speed_override = item

            # Apply dx, dy, dz offsets to the current tracking coordinates
            # (offsets are always relative to SIT_COORDS, not cumulative)
            for leg in LEG_ORDER:
                current_leg_coords[leg][0] = SIT_COORDS[leg][0] + leg_packet[leg][0]
                current_leg_coords[leg][1] = SIT_COORDS[leg][1] + leg_packet[leg][1]
                current_leg_coords[leg][2] = SIT_COORDS[leg][2] + leg_packet[leg][2]

            print("New Targets: ", {leg: [round(c, 2) for c in coords] for leg, coords in current_leg_coords.items()})
            last_live_targets_deg = convert_coords_to_motor_targets(
                current_leg_coords, motor_config
            )

            print("Raw Motor Targets (deg):", {k: round(v, 2) for k, v in last_live_targets_deg.items()})

            # The speed limit persists until the next packet changes it.
            current_speed_limit = speed_override if speed_override is not None else MAX_LIVE_DEG_PER_S

            if not first_live_packet_seen:
                print(f"📡 Socket packet received. Speed limit: {current_speed_limit:.1f} deg/s. Tracking live targets.")
                first_live_packet_seen = True

        # Every tick (packet or not): step each command toward its target at
        # most `speed × gear_ratio` deg/s, then transmit all 12 motors.
        for motor_id in range(1, 13):
            live_cmd_deg[motor_id] = limit_target_step(
                current_cmd_deg=live_cmd_deg[motor_id],
                requested_deg=last_live_targets_deg[motor_id],
                max_deg_per_s=current_speed_limit * motor_config[motor_id]["gear_ratio"],
                dt=tuning.loop_dt,
            )

        send_live_targets(rt0, rt1, live_cmd_deg, motor_config)
        time.sleep(tuning.loop_dt)

# ─── Program entry ───────────────────────────────────────────────────────────

def main():
    """Wire everything together and run the sequence described in the module docstring."""
    time.sleep(3)   # grace period (e.g. to step away / let the CAN HATs settle)
    CALIBRATION_REQUIRED = parse_calibration_flag()
    print("=" * 70)
    print("🤖 AK60 | 4-LEG HOMING | 12 MOTORS | 2 CAN BUSES")
    if CALIBRATION_REQUIRED:
        print("   Phase 1: Front Right (can1: M8→M9→M7)")
        print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
        print("   Phase 2: Back Right  (can1: M11→M12→M10)")
        print("            Front Left  (can0: M2→M3→M1)  ← simultaneous")
    else:
        print("   ⏩ Homing skipped (CALIBRATION_REQUIRED=False)")
    print("   Final:    Post-homing live control with grouped leg socket receiver process")
    print("=" * 70)

    print_lock = threading.Lock()
    stop_event = threading.Event()
    errors: List[str] = []

    # Barrier flags shared by the two bus threads (see controller_thread_entry).
    can0_phase1_done = threading.Event()
    can1_phase1_done = threading.Event()
    can0_phase2_done = threading.Event()
    can1_phase2_done = threading.Event()
    final_hold_takeover = threading.Event()

    # All tunables for homing + watchdogs (field meanings: lib/homing_controller.py).
    tuning = ControlTuning(
        active_kp=114.0,
        active_kd=1.2,
        hold_kp=54.0,
        hold_kd=0.9,
        homed_hold_kp=84.0,
        homed_hold_kd=1.2,
        loop_hz=100.0,
        min_move_time_s=1.2,
        seconds_per_deg=0.03,
        trigger_confirm_s=0.15,
        trigger_velocity_raw_max=4.0,
        feedback_timeout_s=0.15,
        bus_silence_timeout_s=0.40,
        startup_timeout_s=5.0,
        startup_poll_s=0.01,
        zero_feedback_timeout_s=1.5,
        zero_position_tolerance_deg=5.0,
        max_search_time_s=10.0,
        max_search_travel_deg=220.0,
        status_period_s=0.5,
        safe_idle_kp=48.0,
        safe_idle_kd=0.8,
        nudge_position_tolerance_deg=2.0,
        nudge_settle_time_s=0.12,
        search_max_vel_deg_s=100.0,
        search_max_acc_deg_s2=140.0,
        continuous_search_vel_deg_s=40.0,
        nudge_max_vel_deg_s=60.0,
        nudge_max_acc_deg_s2=168.0,
        advance_target_tolerance_deg=2.0,
        advance_settle_time_s=0.20,
        stall_progress_epsilon_deg=0.25,
        stall_timeout_s=0.50,
        wait_log_period_s=1.0,
        contact_hold_offset_deg=0.5,
        contact_release_move_deg=3.0,
        contact_stall_vel_max=2.0,
    )

    # Declared up-front so the `finally` block can clean up whatever got created.
    rt0 = None
    rt1 = None
    live_queue = None
    live_socket_stop = None
    live_socket_process = None

    try:
        # ── 1-2. Open both buses and verify every motor is talking ──
        can0_ids = collect_motor_ids(CAN_CONFIG["can0"])
        can1_ids = collect_motor_ids(CAN_CONFIG["can1"])

        rt0 = BusRuntime("can0", can0_ids, bitrate=1_000_000)
        rt1 = BusRuntime("can1", can1_ids, bitrate=1_000_000)

        ctrl0 = BusHomingController(
            runtime=rt0,
            phase1_config=CAN_CONFIG["can0"]["phase1"],
            phase2_config=CAN_CONFIG["can0"]["phase2"],
            tuning=tuning,
            print_lock=print_lock,
        )
        ctrl1 = BusHomingController(
            runtime=rt1,
            phase1_config=CAN_CONFIG["can1"]["phase1"],
            phase2_config=CAN_CONFIG["can1"]["phase2"],
            tuning=tuning,
            print_lock=print_lock,
        )

        print("\n📡 Waiting for periodic feedback from all motors...")
        ctrl0.verify_periodic_feedback_and_capture_boot_holds()
        ctrl1.verify_periodic_feedback_and_capture_boot_holds()
        print("✅ All motors are streaming feedback.\n")

        # ── 3. One second of gentle hold at the boot positions ──
        print("🧷 Sending low-gain idle hold at captured boot positions for 1 second...")
        for _ in range(int(tuning.loop_hz)):
            ctrl0.send_idle_hold_once()
            ctrl1.send_idle_hold_once()
            time.sleep(tuning.loop_dt)

        # ── 4. Homing (two bus threads in lock-step) ──
        if CALIBRATION_REQUIRED:
            print("🚀 Starting phase 1 on both CAN buses...\n")

            t0 = threading.Thread(
                target=controller_thread_entry,
                args=(
                    ctrl0,
                    can0_phase1_done,
                    can1_phase1_done,
                    can0_phase2_done,
                    final_hold_takeover,
                    stop_event,
                    errors,
                ),
                daemon=True,
            )
            t1 = threading.Thread(
                target=controller_thread_entry,
                args=(
                    ctrl1,
                    can1_phase1_done,
                    can0_phase1_done,
                    can1_phase2_done,
                    final_hold_takeover,
                    stop_event,
                    errors,
                ),
                daemon=True,
            )
            t0.start()
            t1.start()

            # Wait here until both buses report phase 2 complete (or an error).
            while not stop_event.is_set():
                if errors:
                    raise RuntimeError(" | ".join(errors))
                if can0_phase2_done.is_set() and can1_phase2_done.is_set():
                    break
                time.sleep(tuning.loop_dt)

            if errors:
                raise RuntimeError(" | ".join(errors))

            # A few extra hold frames from this thread, then tell the bus
            # threads to stop commanding so there is exactly one commander.
            for _ in range(5):
                ctrl0.hold_all_once()
                ctrl1.hold_all_once()
                time.sleep(tuning.loop_dt)

            final_hold_takeover.set()

            t0.join()
            t1.join()

            if errors:
                raise RuntimeError(" | ".join(errors))

        else:
            print("⏩ Skipping homing — CALIBRATION_REQUIRED=False. Proceeding to live control...")
            final_hold_takeover.set()

        # ── 5. Per-motor gains / polarity / gear ratios ──
        print(f"\n📄 Loading motor command config from {MOTOR_CONFIG_PATH}...")
        motor_config = load_motor_command_config(MOTOR_CONFIG_PATH)
        print("✅ Motor command config loaded.")

        # ── 6. Background telemetry loggers ──
        current_log_thread = threading.Thread(
            target=current_logger_thread_entry,
            args=(rt0, rt1, stop_event),
            daemon=True,
            name="CurrentLogger",
        )
        current_log_thread.start()
        print(
            f"📊 Motor current logger started at {CURRENT_LOG_HZ:.0f} Hz -> {CURRENT_LOG_PATH}"
        )

        temp_log_thread = threading.Thread(
            target=temp_logger_thread_entry,
            args=(rt0, rt1, stop_event),
            daemon=True,
            name="TempLogger",
        )
        temp_log_thread.start()
        print(
            f"🌡️  Motor temperature logger started at {TEMP_LOG_HZ:.0f} Hz -> {TEMP_LOG_PATH}"
        )

        # ── 7. Network receiver in its own process ──
        live_queue = mp.Queue(maxsize=LIVE_QUEUE_MAXSIZE)
        live_socket_stop = mp.Event()
        live_socket_process = mp.Process(
            target=socket_listener_process,
            args=(live_queue, live_socket_stop, SOCKET_PORT, SOCKET_HOST),
            daemon=True,
        )
        live_socket_process.start()
        print(f"📡 Socket receiver process started with PID {live_socket_process.pid}.")

        # ── 8. Sit/zero (if homed) then the live loop; blocks until stopped ──
        run_post_homing_live_control(
            ctrl0=ctrl0,
            ctrl1=ctrl1,
            rt0=rt0,
            rt1=rt1,
            tuning=tuning,
            stop_event=stop_event,
            live_queue=live_queue,
            motor_config=motor_config,
            did_homing=CALIBRATION_REQUIRED,
        )

    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — stopping...")
        stop_event.set()
        final_hold_takeover.set()
    except Exception as exc:
        print(f"\n❌ Runtime error: {exc}")
        stop_event.set()
        final_hold_takeover.set()
    finally:
        # ── 9. Orderly shutdown (always runs) ──
        # Stop the receiver first so nothing new arrives.
        if live_socket_stop is not None:
            live_socket_stop.set()
        if live_socket_process is not None:
            live_socket_process.join(timeout=1.0)
            if live_socket_process.is_alive():
                live_socket_process.terminate()
                live_socket_process.join(timeout=1.0)

        # Then make every motor limp and release the CAN sockets.
        # (Each step is wrapped so one failure can't prevent the rest.)
        print("\n🔌 Disabling motors and shutting down...")
        try:
            if rt0 is not None:
                rt0.disable_all()
        except Exception:
            pass
        try:
            if rt1 is not None:
                rt1.disable_all()
        except Exception:
            pass
        try:
            if rt0 is not None:
                rt0.stop()
        except Exception:
            pass
        try:
            if rt1 is not None:
                rt1.stop()
        except Exception:
            pass
        print("✅ Done.")


if __name__ == "__main__":
    mp.freeze_support()
    main()
