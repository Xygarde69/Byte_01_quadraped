#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/with_ik/stand_up.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Stand-up sequence plus an interactive per-leg tester; sends joint
# angle commands to main_motor.py over TCP.
# ════════════════════════════════════════════════════════════════════════
"""
Stand-up sequence and interactive leg tester for quadruped robot.
Sends joint angle commands via TCP socket to main_motor.py.

Usage:
    python3 stand_up.py          →  interactive menu
    python3 stand_up.py test     →  jump straight to leg tester
    python3 stand_up.py standup  →  run stand-up sequence directly
"""

import math
import sys
import time
import curses
import socket
import pickle

import numpy as np
import _3dof_ik_with_shift_circle_method as ik


# ─── Link lengths (cm) ───────────────────────────────────────────────────────
L1              =  5.995
right_linkConst = -9.094
left_linkConst  =  9.094
L2              = 22.0
L3              = 21.5

# ─── Socket config ───────────────────────────────────────────────────────────
HOST = '127.0.0.1'
PORT = 50000

# ─── Axis label map ──────────────────────────────────────────────────────────
AXIS_LABEL = {0: 'X  (lateral)',
              1: 'Y  (height ↑)',
              2: 'Z  (fore/aft)'}

# ─── Default neutral foot positions  [wx, wy, wz]  Y-up world frame ─────────
#     wx = lateral,  wy = height (negative = foot below hip),  wz = fore/aft
NEUTRAL_LEFT  = np.array([left_linkConst,  -30.0, 0.0])
NEUTRAL_RIGHT = np.array([right_linkConst, -30.0, 0.0])


# ═════════════════════════════════════════════════════════════════════════════
# Socket helpers
# ═════════════════════════════════════════════════════════════════════════════

def send_array_to_motor(array, host=HOST, port=PORT, timeout=2.0):
    """
    Serialise *array* and send it to main_motor.py via TCP socket.

    main_motor.py duplicates each 3-element chunk, so a 6-element array
    [L1,L2,L3, R1,R2,R3] fans out to 12 motor targets:
        motors 1-3  ← L angles   (front-left)
        motors 4-6  ← L angles   (back-left)
        motors 7-9  ← R angles   (front-right)
        motors 10-12← R angles   (back-right)

    Returns True on success, False otherwise.
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        sock.sendall(pickle.dumps(array))
        sock.close()
        return True
    except ConnectionRefusedError:
        print("❌  Connection refused – is main_motor.py running?")
        return False
    except socket.timeout:
        print("❌  Connection timeout")
        return False
    except Exception as exc:
        print(f"❌  Send error: {exc}")
        return False


def test_connection(host=HOST, port=PORT):
    """Return True if the motor controller socket is reachable."""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((host, port))
        sock.close()
        return True
    except Exception:
        return False


# ═════════════════════════════════════════════════════════════════════════════
# IK helpers
# ═════════════════════════════════════════════════════════════════════════════

def wrap_to_180(angle_deg):
    """Wrap angle to (-180, 180]."""
    return (angle_deg + 180) % 360 - 180


def deadband(angle_deg, eps=1e-2):
    """Zero angles smaller than *eps* degrees."""
    return 0.0 if abs(angle_deg) < eps else angle_deg


def make_ref_angles(pos, linkConst):
    """
    Compute raw IK angles (radians) for *pos* – used as the reference /
    home offset that gets subtracted from every subsequent IK result.
    """
    t1, t2, t3 = ik.inverse_kinematics(
        pos[0], pos[1], pos[2], L1, linkConst, L2, L3
    )
    return np.array([t1, t2, t3])


def pos_to_deg(pos, linkConst, ref_angles=None):
    """
    Solve IK for *pos* and return joint angles in **degrees**.

    pos        : [wx, wy, wz]  in Y-up world frame
    linkConst  : horizontal base-circle radius (signed)
    ref_angles : [t1, t2, t3] in **radians** – subtracted when provided
                 (removes the home-position offset so motors command
                  relative motion from their zero position)

    Returns np.array([deg1, deg2, deg3]) after wrap+deadband.
    Raises  ValueError if target is out of reach.
    """
    t1, t2, t3 = ik.inverse_kinematics(
        pos[0], pos[1], pos[2], L1, linkConst, L2, L3
    )
    if ref_angles is not None:
        t1 -= ref_angles[0]
        t2 -= ref_angles[1]
        t3 -= ref_angles[2]

    return np.array([
        deadband(wrap_to_180(math.degrees(t1))),
        deadband(wrap_to_180(math.degrees(t2))),
        deadband(wrap_to_180(math.degrees(t3))),
    ])


def build_cmd(left_deg, right_deg):
    """
    Concatenate left and right angles into a 6-element command array.
    main_motor.py handles the fan-out to all 12 motors.
    """
    return np.concatenate([left_deg, right_deg])


# ═════════════════════════════════════════════════════════════════════════════
# Stand-up sequence
# ═════════════════════════════════════════════════════════════════════════════

def run_standup(total_time=4.0, interval=0.05):
    """
    Interpolate from a crouched position to the neutral standing pose.
    Edit CROUCHED_LEFT / CROUCHED_RIGHT to match your robot's folded state.
    """
    print("\n=== Stand-up Sequence ===")

    # Starting (crouched) foot positions
    crouched_left  = np.array([left_linkConst,  -15.0, 0.0])
    crouched_right = np.array([right_linkConst, -15.0, 0.0])

    # Reference angles at the neutral standing pose (motors zeroed here)
    ref_left  = make_ref_angles(NEUTRAL_LEFT,  left_linkConst)
    ref_right = make_ref_angles(NEUTRAL_RIGHT, right_linkConst)

    start_at = time.time()
    print(f"Interpolating over {total_time:.1f} s  (interval {interval*1000:.0f} ms) …")

    while True:
        t     = min(time.time() - start_at, total_time)
        alpha = t / total_time          # 0 → 1  (linear ramp)

        left_pos  = (1.0 - alpha) * crouched_left  + alpha * NEUTRAL_LEFT
        right_pos = (1.0 - alpha) * crouched_right + alpha * NEUTRAL_RIGHT

        try:
            left_deg  = pos_to_deg(left_pos,  left_linkConst,  ref_left)
            right_deg = pos_to_deg(right_pos, right_linkConst, ref_right)
        except ValueError as exc:
            print(f"  IK error at t={t:.2f}s: {exc}")
        else:
            cmd = build_cmd(left_deg, right_deg)
            if send_array_to_motor(cmd):
                print(f"  t={t:5.2f}s  L:{left_deg.round(2)}  R:{right_deg.round(2)}")
            else:
                print("  ⚠️  Send failed")

        if t >= total_time:
            break

        time.sleep(max(0.0, interval - (time.time() - start_at - t)))

    print("=== Stand-up complete ===\n")


# ═════════════════════════════════════════════════════════════════════════════
# Interactive leg tester  (curses, arrow-key control)
# ═════════════════════════════════════════════════════════════════════════════

def _tester_ui(stdscr):
    """
    Curses UI for interactive single-axis leg testing.

    Key bindings
    ─────────────────────────────────────────────────
    L / R          select left or right leg pair
    X / Y / Z      select axis to move
    ↑  / ↓         move +step / −step cm along axis
    + / =          increase step size (max 10 cm)
    - / _          decrease step size (min 0.1 cm)
    S              re-send current position without moving
    Q              quit tester
    """
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)

    # Working foot positions (mutated by arrow keys)
    pos = {
        'left':  NEUTRAL_LEFT.copy(),
        'right': NEUTRAL_RIGHT.copy(),
    }

    # Reference angles at neutral (subtracted to get motor-relative degrees)
    ref = {
        'left':  make_ref_angles(NEUTRAL_LEFT,  left_linkConst),
        'right': make_ref_angles(NEUTRAL_RIGHT, right_linkConst),
    }

    lc = {'left': left_linkConst, 'right': right_linkConst}

    selected_side = 'left'
    selected_axis = 1          # 0=wx  1=wy(height)  2=wz
    step          = 0.5        # cm per keypress
    status_msg    = "Ready"

    # ── helpers ───────────────────────────────────────────────────────────────

    def compute_and_send():
        nonlocal status_msg
        try:
            l_deg = pos_to_deg(pos['left'],  lc['left'],  ref['left'])
            r_deg = pos_to_deg(pos['right'], lc['right'], ref['right'])
        except ValueError as exc:
            status_msg = f"⚠️  IK error: {str(exc)[:70]}"
            return
        cmd = build_cmd(l_deg, r_deg)
        if send_array_to_motor(cmd):
            status_msg = (f"✅ L:[{', '.join(f'{v:.1f}' for v in l_deg)}]°  "
                          f"R:[{', '.join(f'{v:.1f}' for v in r_deg)}]°")
        else:
            status_msg = "❌ Send failed – is main_motor.py running?"

    def angle_str(side):
        p = pos[side]
        try:
            raw_t1, raw_t2, raw_t3 = ik.inverse_kinematics(
                p[0], p[1], p[2], L1, lc[side], L2, L3
            )
            d = np.array([
                math.degrees(raw_t1 - ref[side][0]),
                math.degrees(raw_t2 - ref[side][1]),
                math.degrees(raw_t3 - ref[side][2]),
            ])
            return (f"θ1={d[0]:+8.2f}°  θ2={d[1]:+8.2f}°  θ3={d[2]:+8.2f}°")
        except ValueError as exc:
            return f"IK error: {str(exc)[:45]}"

    def safe_put(row, col, text, attr=0):
        h, w = stdscr.getmaxyx()
        if row >= h or col >= w:
            return
        try:
            stdscr.addstr(row, col, text[:w - col - 1], attr)
        except curses.error:
            pass

    # ── draw loop ─────────────────────────────────────────────────────────────

    def draw():
        stdscr.clear()
        r = 0

        safe_put(r, 0, "┌────────────────────────────────────────────────────────────┐"); r+=1
        safe_put(r, 0, "│         Interactive Leg Tester  –  Y-up world frame        │"); r+=1
        safe_put(r, 0, "└────────────────────────────────────────────────────────────┘"); r+=1
        r += 1

        # Controls legend
        safe_put(r, 0, "  Side:  [L] left (motors 1-6)      [R] right (motors 7-12)"); r+=1
        safe_put(r, 0, "  Axis:  [X] lateral   [Y] height   [Z] fore/aft           "); r+=1
        safe_put(r, 0, "  Step:  [+]/[-]  move: [↑]/[↓]  send: [S]  quit: [Q]     "); r+=1
        r += 1

        # Current selection summary
        safe_put(r, 0,
            f"  ► Side : {selected_side.upper():5s}  "
            f"Axis : {AXIS_LABEL[selected_axis]:20s}  "
            f"Step : {step:.2f} cm",
            curses.A_BOLD
        ); r += 1
        r += 1

        # Per-side position + angle table
        safe_put(r, 0,
            f"  {'Side':<6} {'wx (lat)':>10} {'wy (ht)':>10} {'wz (f/a)':>10}   "
            f"{'Relative joint angles':}",
            curses.A_UNDERLINE
        ); r += 1

        for side in ('left', 'right'):
            p   = pos[side]
            marker = "►" if side == selected_side else " "
            attr   = curses.A_BOLD if side == selected_side else 0
            safe_put(r, 0,
                f"  {marker} {side.upper():<5} "
                f"{p[0]:>10.3f} {p[1]:>10.3f} {p[2]:>10.3f}   "
                f"{angle_str(side)}",
                attr
            ); r += 1

        r += 1
        safe_put(r, 0, f"  Status: {status_msg}"); r += 1
        stdscr.refresh()

    # ── event loop ────────────────────────────────────────────────────────────

    draw()
    while True:
        key = stdscr.getch()

        if key in (ord('q'), ord('Q')):
            break

        elif key in (ord('l'), ord('L')):
            selected_side = 'left'
        elif key in (ord('r'), ord('R')):
            selected_side = 'right'

        elif key in (ord('x'), ord('X')):
            selected_axis = 0
        elif key in (ord('y'), ord('Y')):
            selected_axis = 1
        elif key in (ord('z'), ord('Z')):
            selected_axis = 2

        elif key in (ord('+'), ord('=')):
            step = min(10.0, round(step + 0.5, 2))
        elif key in (ord('-'), ord('_')):
            step = max(0.1, round(step - 0.1, 2))

        elif key == curses.KEY_UP:
            pos[selected_side][selected_axis] += step
            compute_and_send()
        elif key == curses.KEY_DOWN:
            pos[selected_side][selected_axis] -= step
            compute_and_send()

        elif key in (ord('s'), ord('S')):
            compute_and_send()

        draw()
        time.sleep(0.02)


def interactive_test():
    """Launch the curses-based interactive leg tester."""
    print("\nLaunching interactive tester – press Q inside to quit.\n")
    time.sleep(0.5)
    curses.wrapper(_tester_ui)
    print("\nTester closed.\n")


# ═════════════════════════════════════════════════════════════════════════════
# Entry point
# ═════════════════════════════════════════════════════════════════════════════

if __name__ == '__main__':
    print("=" * 60)
    print("  Quadruped  –  Stand-up / Leg Tester")
    print("=" * 60)

    # Optional CLI shortcut
    if len(sys.argv) > 1:
        arg = sys.argv[1].lower()
        if arg == 'test':
            interactive_test()
            sys.exit(0)
        elif arg == 'standup':
            run_standup()
            sys.exit(0)

    # Check connection
    print(f"\nChecking connection to {HOST}:{PORT} … ", end="", flush=True)
    if test_connection():
        print("✅  Connected")
    else:
        print("❌  Not connected")
        print("⚠️   Make sure main_motor.py is running first!")
        if input("Continue anyway? (y/n): ").strip().lower() != 'y':
            sys.exit(1)

    # Menu
    while True:
        print("\n  1 – Stand-up sequence")
        print("  2 – Interactive leg tester  (arrow-key IK test)")
        print("  q – Quit")
        choice = input("\nChoice: ").strip().lower()

        if choice == '1':
            run_standup()
        elif choice == '2':
            interactive_test()
        elif choice == 'q':
            print("Bye!")
            break
        else:
            print("Invalid choice – try again.")