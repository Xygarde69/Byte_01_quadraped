#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/final/pre_and_post_homing.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# First complete 4-leg program: sequential homing of 12 motors on 2 CAN
# buses in 2 parallel phases (FR+BL, then BR+FL) followed by post-homing
# socket control. The ancestor of robot/main_with_ik.py.
# ════════════════════════════════════════════════════════════════════════
"""
AK60 — 4-LEG SEQUENTIAL HOMING + POST-HOMING SOCKET CONTROL
12 motors | 2 CAN buses | 2-phase parallel homing
Phase 1: Front Right (can1: M8→M9→M7) + Back Left (can0: M5→M6→M4)
Phase 2: Back Right (can1: M11→M12→M10) + Front Left (can0: M2→M3→M1)
Final:   200Hz loop — all angles in RAW MOTOR FRAME throughout.
         gr/flip applied once in ak60_send_targets.py before socket send.
"""

import numpy as np
import time
import threading
import can
import json
import socket
import pickle
import multiprocessing as mp
from ak60_v3_control import AK60V3Motor


# ── Homing Tuning ───────────────────────────────────────────
KP              = 150.0
KD              = 1.5
LOOP_HZ         = 200
LOOP_DT         = 1.0 / LOOP_HZ
MIN_MOVE_TIME   = 1.0
DEG_PER_SEC     = 0.0005
TRIGGER_CONFIRM = 0.3   # s

# ── Post-Homing Easing ──────────────────────────────────────
POST_MIN_MOVE_TIME = 1.0     # minimum seconds for any post-homing move
POST_DEG_PER_SEC   = 0.005   # seconds per degree for post-homing moves

# ── Socket Config ───────────────────────────────────────────
SOCKET_HOST = '127.0.0.1'
SOCKET_PORT = 50000

# ── Motor Config File ───────────────────────────────────────
MOTOR_CONFIG_FILE = 'motor_config.json'


# ── Leg Configurations ──────────────────────────────────────
# All angles here are RAW MOTOR FRAME — no gr/flip anywhere in this file
CAN_CONFIG = {
    'can0': {
        'phase1': [          # Back Left leg: M5 → M6 → M4
            (5, -60.0, -10.0, 4.0,  70.0),
            (6,  10.0,  10.0, 5.0, -38.0),
            (4,  60.0,  10.0, 4.0, -60.0),
        ],
        'phase2': [          # Front Left leg: M2 → M3 → M1
            (2, -60.0, -10.0, 4.0,  70.0),
            (3,  10.0,  10.0, 5.0, -38.0),
            (1,  60.0,  10.0, 4.0, -60.0),
        ],
    },
    'can1': {
        'phase1': [          # Front Right leg: M8 → M9 → M7
            (8,  60.0,  10.0, 4.0, -70.0),
            (9, -10.0, -10.0, 5.0,  38.0),
            (7, -60.0, -10.0, 4.0,  60.0),
        ],
        'phase2': [          # Back Right leg: M11 → M12 → M10
            (11,  60.0,  10.0, 4.0, -70.0),
            (12, -10.0, -10.0, 5.0,  38.0),
            (10, -60.0, -10.0, 4.0,  60.0),
        ],
    },
}


# ═══════════════════════════════════════════════════════════════
#  MOTOR UNIT
# ═══════════════════════════════════════════════════════════════
class MotorUnit:
    def __init__(self, bus, motor_id, can_interface, first_target, step_deg, trigger_current, nudge_deg):
        self.motor_id        = motor_id
        self.step_deg        = step_deg
        self.trigger_current = trigger_current
        self.nudge_deg       = nudge_deg
        self.hold_deg        = 0.0

        self.motor           = AK60V3Motor(motor_id, can_interface, bus=bus)

        self.position    = 0.0
        self.current     = 0.0
        self.temperature = 0.0

        self.src_deg     = 0.0
        self.dest_deg    = first_target
        self.movetime    = max(MIN_MOVE_TIME, abs(first_target) * DEG_PER_SEC)
        self.move_start  = None
        self.move_active = False

        self.above_thresh_since = None
        self.zeroed             = False

    def init(self):
        self.motor.enable()
        time.sleep(0.3)
        self.motor.set_zero_position()

    def read_feedback(self):
        if self.motor.read_feedback(timeout=0.003):
            self.position    = np.degrees(self.motor.position)
            self.current     = abs(self.motor.current)
            self.temperature = self.motor.temperature

    def get_eased_position(self):
        if not self.move_active or self.move_start is None:
            return np.radians(self.dest_deg)
        elapsed = time.time() - self.move_start
        if elapsed >= self.movetime:
            self.move_active = False
            self.src_deg     = self.dest_deg
            return np.radians(self.dest_deg)
        s    = elapsed / self.movetime
        ease = 3*s*s - 2*s*s*s
        return np.radians(self.src_deg + ease * (self.dest_deg - self.src_deg))

    def start_move(self, from_deg, to_deg, min_time=None, spd=None):
        _min         = min_time if min_time is not None else MIN_MOVE_TIME
        _spd         = spd      if spd      is not None else DEG_PER_SEC
        self.src_deg     = from_deg
        self.dest_deg    = to_deg
        self.movetime    = max(_min, abs(to_deg - from_deg) * _spd)
        self.move_start  = time.time()
        self.move_active = True

    def start_nudge(self):
        self.start_move(0.0, self.nudge_deg)

    def send_command(self):
        self.motor.send_mit_command(
            position=self.get_eased_position(), velocity=0.0,
            kp=min(450, KP), kd=KD, torque=0.0,
        )

    def hold_at_zero(self):
        self.motor.send_mit_command(np.radians(self.hold_deg), 0.0, KP, KD, 0.0)

    def hold_at_boot_zero(self):
        self.motor.send_mit_command(0.0, 0.0, KP, KD, 0.0)

    def check_current_trigger(self):
        now = time.time()
        if self.current >= self.trigger_current:
            if self.above_thresh_since is None:
                self.above_thresh_since = now
            elif now - self.above_thresh_since >= TRIGGER_CONFIRM:
                return True
        else:
            self.above_thresh_since = None
        return False

    def disable(self):
        try:
            self.motor.send_mit_command(0, 0, 0, 5, 0)
            time.sleep(0.1)
            self.motor.disable()
        except Exception:
            pass


# ═══════════════════════════════════════════════════════════════
#  BUS CONTROLLER  (homing — unchanged)
# ═══════════════════════════════════════════════════════════════
class BusController:
    def __init__(self, can_interface, bus, phase1_config, phase2_config, print_lock):
        self.iface      = can_interface
        self.print_lock = print_lock
        self.units      = {}
        self.phase1_ids = [c[0] for c in phase1_config]
        self.phase2_ids = [c[0] for c in phase2_config]

        for cfg in phase1_config + phase2_config:
            mid, first, step, trig, nudge = cfg
            u = MotorUnit(bus, mid, can_interface, first, step, trig, nudge)
            self.units[mid] = u

    def _log(self, msg):
        with self.print_lock:
            print(msg)

    def init_all(self):
        for u in self.units.values():
            u.init()
            self._log(f"  [{self.iface}] M{u.motor_id} ✓ | trigger: {u.trigger_current}A | nudge: {u.nudge_deg:+.1f}°")

    def _run_sequence(self, sequence_ids, hold_at_nudge_ids, phase_name):
        zeroed_ids  = set()
        nudging_ids = set()
        active_idx  = 0
        last_status = 0.0

        first_u = self.units[sequence_ids[0]]
        first_u.start_move(first_u.position, first_u.dest_deg)
        self._log(f"\n[{self.iface}] {phase_name} ▶ M{sequence_ids[0]} → {first_u.dest_deg:.1f}°")

        while True:
            loop_start = time.time()

            for u in self.units.values():
                u.read_feedback()

            for motor_id, u in self.units.items():
                if motor_id in zeroed_ids:
                    u.hold_at_zero()
                elif motor_id in nudging_ids:
                    u.send_command()
                elif motor_id in hold_at_nudge_ids:
                    u.hold_at_zero()
                elif active_idx < len(sequence_ids) and motor_id == sequence_ids[active_idx]:
                    u.send_command()
                else:
                    u.hold_at_boot_zero()

            for motor_id in list(nudging_ids):
                u = self.units[motor_id]
                if not u.move_active:
                    u.hold_deg = u.nudge_deg
                    self._log(f"  ↩️  [{self.iface}] M{motor_id} nudge done → holding at {u.nudge_deg:+.1f}°")
                    nudging_ids.discard(motor_id)
                    zeroed_ids.add(motor_id)

            if active_idx < len(sequence_ids):
                active_id = sequence_ids[active_idx]
                active_u  = self.units[active_id]
                triggered = active_u.check_current_trigger()

                if not active_u.move_active and not triggered \
                        and active_u.current < active_u.trigger_current \
                        and active_id not in zeroed_ids and active_id not in nudging_ids:
                    next_target = active_u.dest_deg + active_u.step_deg
                    self._log(f"  📍 [{self.iface}] M{active_id}: {active_u.dest_deg:.1f}° → {next_target:.1f}°")
                    active_u.start_move(active_u.dest_deg, next_target)

                if triggered:
                    self._log(f"  ✅ [{self.iface}] M{active_id}: {active_u.current:.2f}A >= {active_u.trigger_current}A → zero at {active_u.position:.1f}°")
                    active_u.motor.set_zero_position()
                    active_u.zeroed = True
                    active_idx += 1

                    if active_u.nudge_deg != 0.0:
                        nudging_ids.add(active_id)
                        active_u.start_nudge()
                        self._log(f"     ↩️  [{self.iface}] M{active_id} nudge: 0° → {active_u.nudge_deg:+.1f}°")
                    else:
                        active_u.hold_deg = 0.0
                        zeroed_ids.add(active_id)
                        self._log(f"     [{self.iface}] M{active_id} nudge=0° → holding at new zero")

                    if active_idx < len(sequence_ids):
                        next_id = sequence_ids[active_idx]
                        next_u  = self.units[next_id]
                        next_u.start_move(next_u.position, next_u.dest_deg)
                        self._log(f"  🚀 [{self.iface}] Next: M{next_id} → {next_u.dest_deg:.1f}°")

            now = time.time()
            if now - last_status > 1.0:
                last_status = now
                parts = []
                for mid in sequence_ids:
                    u = self.units[mid]
                    if mid in zeroed_ids:
                        s = "✅"
                    elif mid in nudging_ids:
                        s = "↩️ "
                    elif u.move_active:
                        s = "🚀"
                    else:
                        s = "⏸️ "
                    parts.append(f"{s}M{mid}:{u.position:.1f}°|{u.current:.2f}A")
                self._log(f"  [{self.iface}] {phase_name}: " + "  ".join(parts))

            if active_idx >= len(sequence_ids) and not nudging_ids:
                break

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))

    def thread_run(self, phase1_barrier):
        self._run_sequence(
            sequence_ids      = self.phase1_ids,
            hold_at_nudge_ids = set(),
            phase_name        = "Phase1"
        )
        self._log(f"\n⏳ [{self.iface}] Phase 1 done — waiting at barrier for other bus...")
        phase1_barrier.wait()
        self._log(f"\n🚦 [{self.iface}] Both buses ready — Phase 2 start!")

        self._run_sequence(
            sequence_ids      = self.phase2_ids,
            hold_at_nudge_ids = set(self.phase1_ids),
            phase_name        = "Phase2"
        )
        self._log(f"\n🏁 [{self.iface}] Phase 2 complete!")

    def disable_all(self):
        for u in self.units.values():
            u.disable()
            print(f"  M{u.motor_id} disabled ✓")


# ═══════════════════════════════════════════════════════════════
#  SOCKET LISTENER PROCESS  — pure pass-through, no conversion
# ═══════════════════════════════════════════════════════════════
def socket_listener_process(dest_queue, port=SOCKET_PORT, host=SOCKET_HOST):
    """
    Dumb pipe — receives pickle'd numpy array of 12 RAW motor angles
    (gr/flip already applied by sender) and forwards to control loop.

    Sender must use ak60_send_targets.py which handles gr/flip conversion.
    """
    print(f"[Socket Process PID {mp.current_process().pid}] Starting on {host}:{port}...")
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Socket Process] Listening on {host}:{port}")

        while True:
            try:
                conn, addr = server.accept()
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                conn.close()

                if data:
                    angles = pickle.loads(data)
                    angles = np.asarray(angles, dtype=float)
                    dest_queue.put(angles)
                    print(f"[Socket Process] Received raw targets: {angles.tolist()}°")

            except Exception as e:
                print(f"[Socket Process] Connection error: {e}")

    except Exception as e:
        print(f"[Socket Process] Server error: {e}")
    finally:
        server.close()
        print("[Socket Process] Shutdown")


# ═══════════════════════════════════════════════════════════════
#  POST-HOMING CONTROL LOOP — pure raw motor frame, no conversion
# ═══════════════════════════════════════════════════════════════
def post_homing_loop(ctrl0: BusController, ctrl1: BusController,
                     dest_queue: mp.Queue, motor_config: dict):
    """
    200Hz loop — operates entirely in RAW MOTOR FRAME.
    No gear_ratio or flip logic here at all.
    Sender (ak60_send_targets.py) is responsible for conversion.

    kp/kd loaded from motor_config.json per motor.
    """

    # ── Collect all MotorUnits sorted M1→M12 ────────────────
    all_units: dict[int, MotorUnit] = {}
    all_units.update(ctrl0.units)
    all_units.update(ctrl1.units)
    motor_ids = sorted(all_units.keys())
    n = len(motor_ids)

    # ── Per-motor kp/kd from JSON ────────────────────────────
    cfg = motor_config['motors']
    def get_kp(mid): return float(cfg[str(mid)]['kp'])
    def get_kd(mid): return float(cfg[str(mid)]['kd'])

    # ── Easing state — initialised at nudge_deg (raw frame) ──
    src_deg     = np.array([all_units[m].hold_deg for m in motor_ids], dtype=float)
    dest_deg    = src_deg.copy()
    move_start  = [None]  * n
    move_time   = [0.0]   * n
    move_active = [False] * n

    def start_move_post(i, from_deg, to_deg):
        src_deg[i]     = from_deg
        dest_deg[i]    = to_deg
        move_time[i]   = max(POST_MIN_MOVE_TIME, abs(to_deg - from_deg) * POST_DEG_PER_SEC)
        move_start[i]  = time.time()
        move_active[i] = True

    def get_eased_pos(i):
        if not move_active[i] or move_start[i] is None:
            return dest_deg[i]
        elapsed = time.time() - move_start[i]
        if elapsed >= move_time[i]:
            move_active[i] = False
            src_deg[i]     = dest_deg[i]
            return dest_deg[i]
        s    = elapsed / move_time[i]
        ease = 3*s*s - 2*s*s*s
        return src_deg[i] + ease * (dest_deg[i] - src_deg[i])

    current_target     = src_deg.copy()
    socket_cmd_received = False

    print("\n" + "="*65)
    print("🎮  POST-HOMING CONTROL LOOP — 200Hz | RAW MOTOR FRAME")
    print("    Holding at nudge positions:")
    for m in motor_ids:
        print(f"      M{m:>2}: {all_units[m].hold_deg:+.1f}°")
    print(f"    Socket: {SOCKET_HOST}:{SOCKET_PORT}")
    print("    Waiting for raw target angles via socket...")
    print("="*65 + "\n")

    last_status = 0.0

    try:
        while True:
            loop_start = time.time()

            # ── Read feedback ────────────────────────────────
            for m in motor_ids:
                all_units[m].read_feedback()

            # ── Poll queue for new raw targets ───────────────
            try:
                new_targets = dest_queue.get_nowait()   # raw motor frame, degrees
                if len(new_targets) == n:
                    socket_cmd_received = True
                    print(f"[Post-Homing] 📥 New raw targets: {new_targets.tolist()}°")
                    for i, m in enumerate(motor_ids):
                        if abs(new_targets[i] - current_target[i]) > 0.1:
                            start_move_post(i, all_units[m].position, new_targets[i])
                    current_target = new_targets.copy()
                else:
                    print(f"[Post-Homing] ⚠️  Expected {n} targets, got {len(new_targets)} — ignored")
            except Exception:
                pass   # queue empty

            # ── Send commands — raw angle straight to motor ──
            for i, m in enumerate(motor_ids):
                u = all_units[m]
                all_units[m].motor.send_mit_command(
                    position = np.radians(get_eased_pos(i)),
                    velocity = 0.0,
                    kp       = min(450, get_kp(m)),
                    kd       = get_kd(m),
                    torque   = 0.0,
                )

            # ── Periodic status ──────────────────────────────
            now = time.time()
            if now - last_status > 2.0:
                last_status = now
                parts = []
                for i, m in enumerate(motor_ids):
                    tag = "🚀" if move_active[i] else "✅"
                    parts.append(f"{tag}M{m}:{all_units[m].position:.1f}°→{current_target[i]:.1f}°")
                mode = "SOCKET" if socket_cmd_received else "NUDGE HOLD"
                print(f"  [Post|{mode}] " + "  ".join(parts))

            elapsed = time.time() - loop_start
            time.sleep(max(0.0, LOOP_DT - elapsed))

    except KeyboardInterrupt:
        print("\n🛑 [Post-Homing] Ctrl+C — exiting control loop")
        raise


# ═══════════════════════════════════════════════════════════════
#  MAIN
# ═══════════════════════════════════════════════════════════════
def main():
    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass

    print("=" * 65)
    print("🤖  AK60  |  4-LEG HOMING  |  12 MOTORS  |  2 CAN BUSES")
    print("   Phase 1: Front Right (can1: M8→M9→M7)")
    print("            Back Left   (can0: M5→M6→M4)  ← simultaneous")
    print("   Phase 2: Back Right  (can1: M11→M12→M10)")
    print("            Front Left  (can0: M2→M3→M1)   ← simultaneous")
    print("   Final:   200Hz raw-frame loop, socket accepts raw angles")
    print("=" * 65)

    print(f"\n📄 Loading motor config from {MOTOR_CONFIG_FILE}...")
    with open(MOTOR_CONFIG_FILE, 'r') as f:
        motor_config = json.load(f)
    print("✅ Motor config loaded")

    print_lock = threading.Lock()

    bus0 = can.interface.Bus(channel='can0', interface='socketcan', bitrate=1_000_000)
    bus1 = can.interface.Bus(channel='can1', interface='socketcan', bitrate=1_000_000)

    ctrl0 = BusController('can0', bus0,
                          CAN_CONFIG['can0']['phase1'],
                          CAN_CONFIG['can0']['phase2'],
                          print_lock)
    ctrl1 = BusController('can1', bus1,
                          CAN_CONFIG['can1']['phase1'],
                          CAN_CONFIG['can1']['phase2'],
                          print_lock)

    print("\n🔌 Enabling all 12 motors...")
    ctrl0.init_all()
    ctrl1.init_all()
    print("✅ All 12 motors enabled — holding at boot zero\n")

    phase1_barrier = threading.Barrier(2)
    t0 = threading.Thread(target=ctrl0.thread_run, args=(phase1_barrier,), daemon=True)
    t1 = threading.Thread(target=ctrl1.thread_run, args=(phase1_barrier,), daemon=True)
    t0.start(); t1.start()
    t0.join();  t1.join()

    print("\n" + "="*65)
    print("🏁 ALL 12 MOTORS HOMED")
    print("="*65)

    dest_queue     = mp.Queue()
    socket_process = mp.Process(
        target = socket_listener_process,
        args   = (dest_queue, SOCKET_PORT, SOCKET_HOST),
        daemon = True,
    )
    socket_process.start()
    print(f"📡 Socket listener started (PID {socket_process.pid})")
    time.sleep(0.5)

    try:
        post_homing_loop(ctrl0, ctrl1, dest_queue, motor_config)
    except KeyboardInterrupt:
        print("\n🛑 Ctrl+C — releasing all motors...")
    finally:
        ctrl0.disable_all()
        ctrl1.disable_all()
        bus0.shutdown()
        bus1.shutdown()
        socket_process.terminate()
        socket_process.join(timeout=2)
        if socket_process.is_alive():
            socket_process.kill()
        print("✅ Done!")


if __name__ == "__main__":
    main()
