#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/ik_sender.py
# STATUS    ·  WORKSPACE · latest experiments, not yet merged into robot/
# ────────────────────────────────────────────────────────────────────────
# Send one test coordinate to the legs and watch them move (run after
# the runtime shows LIVE CONTROL READY). No 'speed' key is sent, so the
# runtime's default speed limit applies.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 ik_sender.py
# NOTE      ·  HOST is hard-coded to a LAN IP; edit it for your network.
# ════════════════════════════════════════════════════════════════════════
"""
Send a single test coordinate to one leg and watch it move.
Run AFTER main_with_current_temp.py is up and showing LIVE CONTROL READY.

No "speed" key is sent here, so main_with_ik.py uses the safe default
MAX_LIVE_DEG_PER_S (400 deg/s). Only tapping_gait.py overrides speed.
"""
import pickle
import socket
import time

HOST = "10.154.75.34"
PORT = 50000
'''
# ── CHANGE THIS to test ──────────────────────────────────────────────────────
TARGET_LEG = "fr"               # which leg to move: fl, bl, fr, br
TARGET_COORDS = (-9.094, 11.0, 9.0)   # (x, y, z) in cm
HOLD_SECONDS  = 3.0             # how long to hold the position
# ────────────────────────────────────────────────────────────────────────────
'''
# Keep all other legs at sitting position (safe — no movement)
SITTING = [-9.094, 11.0, 13.0]    # sit
# SITTING = [-9.094, 25.0, 10.0]  # stand
CUSTOM = [-9.094, 11.0, 6.0]
#Leftside -10 went in to the bot
#Right Side -10 went in to the bot
HOLD_SECONDS  = 3.0             # how long to hold the position
payload = {
    "fl": list(SITTING),
    "bl": list(SITTING),
    "fr": list(SITTING),
    "br": list(SITTING),
}
#payload[TARGET_LEG] = list(TARGET_COORDS)  # only move this one leg

# No "speed" key → main_with_ik.py uses its safe default MAX_LIVE_DEG_PER_S

#print(f"Sending {TARGET_LEG} → {TARGET_COORDS}")
print(f"All other legs hold sitting position.")

data = pickle.dumps(payload)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(data)

print(f"Sent. Holding for {HOLD_SECONDS}s... watch the leg.")
time.sleep(HOLD_SECONDS)
print("Done.")