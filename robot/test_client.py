#!/usr/bin/env python3
"""
test_client.py — minimal interactive command-line client for the state server
══════════════════════════════════════════════════════════════════════════════

Type a state name at the prompt and it is POSTed to ``state_manager_server_v2``
(``POST /change-state`` on port 8000).  Every request asks for 3 gait cycles
(only relevant for moving states) and identifies itself as "cli_tester".

    python3 test_client.py
    Enter state > stand          # → STAND
    Enter state > forward        # → 3 trot cycles forward
    Enter state > quit

Valid states (case-insensitive): sit, stand, up, forward, backward, left,
right, cw, ccw  (jump is accepted by the server but does nothing).
``up`` raises the body one step (knee +5°, thigh −2.5°); repeat to go higher,
``stand`` goes back to the normal height.

KNEE CURRENT MONITOR
    The top line of the terminal continuously shows the current (A) of the
    four knee motors (ids 3, 6, 9, 12), read from ``logs/motor_currents.csv``
    which ``main_with_ik.py`` writes at 5 Hz.  This only works when the client
    runs on the Pi itself (same ``robot/`` folder).  "STALE" means no new row
    for > 2 s (runtime not running / logger stopped).

The server replies only after the motion finishes, and replies 429 if the robot
is still busy with a previous command.  ``requests.post`` is called without a
timeout, so a long gait simply makes the client wait.
Defaults to localhost; edit ``API_URL`` to control a robot over the network.
"""
import os
import shutil
import sys
import threading
import time

import requests

# The URL where your FastAPI server is running
API_URL = "http://127.0.0.1:8000/change-state"

# ─── Knee current monitor ────────────────────────────────────────────────────
CURRENT_CSV = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs", "motor_currents.csv")
KNEE_MOTORS = {"fl": 3, "bl": 6, "fr": 9, "br": 12}
MONITOR_HZ = 5.0
STALE_AFTER_S = 2.0


def read_last_current_row():
    """Return (timestamp, [12 currents]) from the last full CSV row, or None."""
    try:
        with open(CURRENT_CSV, "rb") as fh:
            fh.seek(0, os.SEEK_END)
            size = fh.tell()
            fh.seek(max(0, size - 1024))
            lines = fh.read().decode("utf-8", "ignore").splitlines()
    except OSError:
        return None
    # The last line may be half-written — walk back to the newest complete one.
    for line in reversed(lines):
        parts = line.split(",")
        if len(parts) != 13 or parts[0].startswith("timestamp"):
            continue
        try:
            return float(parts[0]), [float(v) for v in parts[1:]]
        except ValueError:
            continue
    return None


def format_knee_line() -> str:
    row = read_last_current_row()
    if row is None:
        return f"⚡ Knee current: no data ({CURRENT_CSV} not found / empty)"
    ts, currents = row
    vals = "  ".join(f"{leg.upper()}(m{mid}) {currents[mid - 1]:+6.2f}A"
                     for leg, mid in KNEE_MOTORS.items())
    age = time.time() - ts
    tag = f"  STALE {age:.0f}s" if age > STALE_AFTER_S else ""
    return f"⚡ Knee current: {vals}{tag}"


class KneeCurrentMonitor:
    """
    Keeps the knee currents on the top terminal line.  Rows 2..N are made a
    scroll region so the prompt and server replies scroll underneath without
    overwriting the status line.
    """
    def __init__(self):
        self._stop = threading.Event()
        self._out_lock = threading.Lock()
        self._rows = 0
        self._thread = threading.Thread(target=self._run, daemon=True)

    def _setup_region(self):
        cols, rows = shutil.get_terminal_size()
        self._rows = rows
        # Scroll region rows 2..rows, then park the cursor at the bottom.
        sys.stdout.write(f"\033[2;{rows}r\033[{rows};1H")
        sys.stdout.flush()

    def start(self):
        if not sys.stdout.isatty():
            return
        sys.stdout.write("\n")   # make room so nothing is lost under the status line
        self._setup_region()
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            line = format_knee_line()
            with self._out_lock:
                if shutil.get_terminal_size()[1] != self._rows:
                    # Terminal resized: redo the region (saving cursor around it).
                    sys.stdout.write("\0337")
                    self._setup_region()
                    sys.stdout.write("\0338")
                cols = shutil.get_terminal_size()[0]
                # save cursor → row 1 → clear line → write → restore cursor
                sys.stdout.write(f"\0337\033[1;1H\033[2K{line[:cols - 1]}\0338")
                sys.stdout.flush()
            self._stop.wait(1.0 / MONITOR_HZ)

    def stop(self):
        if not self._thread.is_alive():
            return
        self._stop.set()
        self._thread.join(timeout=1.0)
        # Restore full-screen scrolling.
        sys.stdout.write(f"\033[r\033[{shutil.get_terminal_size()[1]};1H\n")
        sys.stdout.flush()


def send_command(target_state: str, sender: str = "cli_tester"):
    """Sends a POST request to the quadruped API."""
    payload = {
        "new_state": target_state,
        "sender_id": sender,
        "cycles": 3
    }
    
    print(f"\n📡 Sending '{target_state}' command to robot...")
    
    try:
        # Send the POST request to the FastAPI server
        response = requests.post(API_URL, json=payload)
        
        # Check how the server responded
        if response.status_code == 200:
            print(f"✅ Success: {response.json()}")
        elif response.status_code == 429:
            print(f"⏳ Rejected: Robot is currently busy! ({response.json().get('detail')})")
        else:
            print(f"❌ Error {response.status_code}: {response.text}")
            
    except requests.exceptions.ConnectionError:
        print("🚨 Connection Error: Could not reach the API. Is Uvicorn running?")
    except requests.exceptions.Timeout:
        print("⏱️ Timeout: The server took too long to respond.")
    except Exception as e:
        print(f"⚠️ Unexpected Error: {e}")

def main():
    print("=" * 50)
    print("🤖 Quadruped API Client")
    print("Available states: sit, stand, up, forward, backward, left, right, cw, ccw")
    print("Type 'quit' or 'exit' to close this client.")
    print("=" * 50)

    monitor = KneeCurrentMonitor()
    monitor.start()
    try:
        while True:
            try:
                # Prompt the user for a command
                cmd = input("\nEnter state > ").strip().lower()

                if cmd in ['quit', 'exit']:
                    print("Closing client...")
                    break
                elif cmd == "":
                    continue

                # Send the command (the status line keeps updating meanwhile)
                send_command(cmd)

            except (KeyboardInterrupt, EOFError):
                print("\nClosing client...")
                break
    finally:
        monitor.stop()

if __name__ == "__main__":
    main()