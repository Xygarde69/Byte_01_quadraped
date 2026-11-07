#!/usr/bin/env python3
"""
knee_current_monitor.py — real-time current of the 4 knee motors
══════════════════════════════════════════════════════════════════════════════

Run (from ``robot/``) on the Pi, in its own terminal, while ``main_with_ik.py``
is running:

    python3 knee_current_monitor.py              # live single line, 20 Hz
    python3 knee_current_monitor.py --scroll     # one line per update (log style)
    python3 knee_current_monitor.py --hz 50      # faster refresh

HOW IT WORKS
    It listens directly on can0 / can1 (SocketCAN lets several programs read
    the same bus) and decodes the knee motors' status frames — the same frames
    the runtime uses — so it sees every reply (~100 Hz per motor), not the 5 Hz
    CSV log.  It is RECEIVE-ONLY: it never sends anything, so it cannot disturb
    the motors.  Knee motors: fl=3, bl=6 (can0), fr=9, br=12 (can1).

    The motors only reply when the runtime commands them, so if main_with_ik.py
    is not running the values go STALE.

    Columns: current (A) per knee, and the peak |current| since start
    (reset with Ctrl+C → restart, or see --scroll output for history).
"""
import argparse
import sys
import threading
import time

import can

from lib.ak60_v3_control import AK60V3Motor

KNEE_MOTORS = {"fl": 3, "bl": 6, "fr": 9, "br": 12}
BUS_FOR_MOTOR = {3: "can0", 6: "can0", 9: "can1", 12: "can1"}
STALE_AFTER_S = 0.5


class KneeState:
    def __init__(self):
        self.lock = threading.Lock()
        self.current = {mid: None for mid in KNEE_MOTORS.values()}
        self.peak = {mid: 0.0 for mid in KNEE_MOTORS.values()}
        self.last_rx = {mid: 0.0 for mid in KNEE_MOTORS.values()}
        self.count = {mid: 0 for mid in KNEE_MOTORS.values()}


def reader_thread(channel: str, motor_ids, state: KneeState, stop: threading.Event):
    # Kernel-side filter: only the status frames (mode 0x29) of our knee motors.
    filters = [
        {"can_id": (AK60V3Motor.FEEDBACK_MODE_ID << 8) | mid,
         "can_mask": 0x1FFFFFFF, "extended": True}
        for mid in motor_ids
    ]
    try:
        bus = can.interface.Bus(channel=channel, interface="socketcan",
                                can_filters=filters, receive_own_messages=False)
    except Exception as e:
        print(f"\n[{channel}] cannot open bus: {e}", file=sys.stderr)
        return
    try:
        while not stop.is_set():
            msg = bus.recv(timeout=0.2)
            if msg is None:
                continue
            fb = AK60V3Motor.decode_feedback_message(msg)
            if fb is None:
                continue
            mid = AK60V3Motor.rx_motor_id(msg)
            if mid not in state.current:
                continue
            with state.lock:
                state.current[mid] = fb["current_A"]
                state.peak[mid] = max(state.peak[mid], abs(fb["current_A"]))
                state.last_rx[mid] = time.time()
                state.count[mid] += 1
    finally:
        bus.shutdown()


def format_line(state: KneeState) -> str:
    now = time.time()
    parts = []
    with state.lock:
        for leg, mid in KNEE_MOTORS.items():
            cur = state.current[mid]
            if cur is None:
                parts.append(f"{leg.upper()}(m{mid}):   --  ")
            elif now - state.last_rx[mid] > STALE_AFTER_S:
                parts.append(f"{leg.upper()}(m{mid}): STALE ")
            else:
                parts.append(f"{leg.upper()}(m{mid}): {cur:+6.2f}A")
        peaks = " ".join(f"{state.peak[mid]:.2f}" for mid in KNEE_MOTORS.values())
    return f"{time.strftime('%H:%M:%S')}  " + "  ".join(parts) + f"   | peak A: {peaks}"


def main():
    ap = argparse.ArgumentParser(description="Real-time knee motor current monitor (receive-only).")
    ap.add_argument("--hz", type=float, default=20.0, help="display refresh rate (default 20)")
    ap.add_argument("--scroll", action="store_true", help="print a new line each update instead of overwriting")
    args = ap.parse_args()

    state = KneeState()
    stop = threading.Event()
    threads = []
    for channel in ("can0", "can1"):
        mids = [m for m, ch in BUS_FOR_MOTOR.items() if ch == channel]
        t = threading.Thread(target=reader_thread, args=(channel, mids, state, stop), daemon=True)
        t.start()
        threads.append(t)

    print("Knee current monitor — Ctrl+C to quit (receive-only, never sends on CAN)")
    period = 1.0 / args.hz
    try:
        while True:
            line = format_line(state)
            if args.scroll:
                print(line, flush=True)
            else:
                sys.stdout.write("\r\033[2K" + line)
                sys.stdout.flush()
            time.sleep(period)
    except KeyboardInterrupt:
        pass
    finally:
        stop.set()
        for t in threads:
            t.join(timeout=1.0)
        print()


if __name__ == "__main__":
    main()
