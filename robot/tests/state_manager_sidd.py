# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  robot/tests/state_manager_sidd.py
# STATUS    ·  EXPERIMENT · state manager that launches a gait subprocess
# ────────────────────────────────────────────────────────────────────────
# Synchronous state manager (sit/stand) that can also start/stop a trot
# gait script as a subprocess. Sends deltas (target - current) instead
# of absolute offsets.
# ════════════════════════════════════════════════════════════════════════
import pickle
import socket
import subprocess
import time
from config.robot_config import SOCKET_HOST, SOCKET_PORT, LEG_ORDER


class QuadrupedStateManager:
    def __init__(self):
        # Internal tracking of the current position (starts at sit: 0,0,0)
        self.current_positions = {leg: [0.0, 0.0, 0.0] for leg in LEG_ORDER}

        # Define Coordinate targets for specific states
        self.states = {
            "sit":   {"dx": 0.0, "dy": 0.0, "dz": 0.0},
            "stand": {"dx": 0.0, "dy": 0.0, "dz": -14.0},
        }

        # Holds the running trot subprocess
        self._trot_process = None

    # ─────────────────────────────────────────────────────────────────────────
    # EXISTING STATE API  (unchanged)
    # ─────────────────────────────────────────────────────────────────────────
    def change_state(self, target_state: str):
        """API-like function to trigger a state transition."""
        if target_state not in self.states:
            print(f"Error: State '{target_state}' not defined.")
            return

        target = self.states[target_state]
        print(f"\nTransitioning to: {target_state.upper()}")

        # Calculate the delta needed to get from CURRENT to TARGET
        payload_deltas = {}

        for leg in LEG_ORDER:
            curr_x, curr_y, curr_z = self.current_positions[leg]

            dx = target["dx"] - curr_x
            dy = target["dy"] - curr_y
            dz = target["dz"] - curr_z

            payload_deltas[leg] = [dx, dy, dz]

            # Update internal tracker so we know where the leg is now
            self.current_positions[leg] = [target["dx"], target["dy"], target["dz"]]

        self._send_to_robot(payload_deltas)

    def _send_to_robot(self, payload: dict):
        """Sends the calculated dx, dy, dz deltas to main_with_ik.py."""
        try:
            data = pickle.dumps(payload)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.connect((SOCKET_HOST, SOCKET_PORT))
                s.sendall(data)
            print(f"✅ Sent deltas: {payload}")
        except ConnectionRefusedError:
            print("❌ Failed: Is main_with_ik.py running?")

    # ─────────────────────────────────────────────────────────────────────────
    # TROT GAIT API
    # ─────────────────────────────────────────────────────────────────────────
    def start_trot(self):
        """
        Launch trot_gait_node_sid.py as a subprocess.
        The robot should already be standing before calling this.
        """
        if self._trot_process and self._trot_process.poll() is None:
            print("⚠️  Trot is already running.")
            return

        print("🚀 Starting trot gait...")
        self._trot_process = subprocess.Popen(
            ["python3", "trot_gait_node_sid.py"],
            stdin=subprocess.PIPE,
            text=True,
            bufsize=1,
        )
        print("✅ Trot process started. Use send_trot_command() to control it.")

    def send_trot_command(self, cmd: str):
        """
        Send a command to the running trot process via stdin.

        Available commands (mirrors trot_gait_node_sid.py):
          f   → forward  1 cycle        ff  → forward  continuous
          b   → backward 1 cycle        bb  → backward continuous
          r   → strafe right 1 cycle    rr  → strafe right continuous
          l   → strafe left  1 cycle    ll  → strafe left  continuous
          y   → stop continuous gait
          s   → sit
          stand → stand
          q   → quit trot process
        """
        if self._trot_process and self._trot_process.poll() is None:
            self._trot_process.stdin.write(cmd + "\n")
            self._trot_process.stdin.flush()
            print(f"✅ Sent to trot: '{cmd}'")
        else:
            print("⚠️  Trot is not running. Call start_trot() first.")

    def stop_trot(self):
        """
        Gracefully stop the trot process.
        Sends 'y' to halt any continuous gait (feet land cleanly),
        then 'q' to exit the trot process.
        """
        if self._trot_process and self._trot_process.poll() is None:
            print("⏹  Stopping trot...")
            self.send_trot_command("y")   # stop continuous gait cleanly
            time.sleep(0.5)               # give it time to finish the cycle
            self.send_trot_command("q")   # quit the trot process
            self._trot_process.wait()
            self._trot_process = None
            print("✅ Trot process stopped.")
        else:
            print("⚠️  No trot process is running.")

    def is_trot_running(self) -> bool:
        """Returns True if the trot subprocess is currently active."""
        return self._trot_process is not None and self._trot_process.poll() is None


# ─────────────────────────────────────────────────────────────────────────────
# CLI
# ─────────────────────────────────────────────────────────────────────────────
HELP = """
Commands
──────────────────────────────────────────────
  stand          → stand up
  sit            → sit down

  trot           → start trot process
  f              → forward  1 cycle
  ff             → forward  continuous
  b              → backward 1 cycle
  bb             → backward continuous
  r              → strafe right 1 cycle
  rr             → strafe right continuous
  l              → strafe left  1 cycle
  ll             → strafe left  continuous
  y              → stop continuous gait
  stoptrot       → stop trot process entirely

  q              → quit
──────────────────────────────────────────────
"""

TROT_COMMANDS = {"f", "ff", "b", "bb", "r", "rr", "l", "ll", "y", "s", "stand"}

if __name__ == "__main__":
    robot = QuadrupedStateManager()
    print(HELP)

    while True:
        command = input("> ").strip().lower()

        if not command:
            continue

        if command == "q":
            # Stop trot cleanly before exit if still running
            if robot.is_trot_running():
                robot.stop_trot()
            break

        elif command in ("stand", "sit"):
            # If trot is running, let trot handle stand/sit via its own stdin
            if robot.is_trot_running():
                robot.send_trot_command(command)
            else:
                robot.change_state(command)

        elif command == "trot":
            robot.start_trot()

        elif command == "stoptrot":
            robot.stop_trot()

        elif command in TROT_COMMANDS:
            robot.send_trot_command(command)

        else:
            print(f"Unknown command: '{command}'")
            print(HELP)