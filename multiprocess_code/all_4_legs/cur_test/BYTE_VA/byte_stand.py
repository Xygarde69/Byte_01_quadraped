#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VA/byte_stand.py
# STATUS    ·  PROTOTYPE · voice assistant (BYTE_VA)
# ────────────────────────────────────────────────────────────────────────
# One-shot: send the STAND pose [-9.65, 20, 6] cm to all legs on
# 127.0.0.1:50000, hold 3 s. Triggered by app.py for 'stand'.
# NOTE      ·  Old coordinate protocol - not compatible with robot/.
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

HOST = "127.0.0.1"
PORT = 50000
'''
# ── CHANGE THIS to test ──────────────────────────────────────────────────────
TARGET_LEG = "fr"               # which leg to move: fl, bl, fr, br
TARGET_COORDS = (-9.65, 11.0, 9.0)   # (x, y, z) in cm
HOLD_SECONDS  = 3.0             # how long to hold the position
# ────────────────────────────────────────────────────────────────────────────
'''
# Keep all other legs at STAND position (safe — no movement)
STAND = [-9.65, 20.0, 6.0]
#STAND = [-9.65, 11.0, 9.0]
CUSTOM = [-9.65, 25.0, 6.0]
#Leftside -10 went in to the bot
#Right Side -10 went in to the bot
HOLD_SECONDS  = 3.0             # how long to hold the position
payload = {
    "fl": list(STAND),
    "bl": list(STAND),
    "fr": list(STAND),
    "br": list(STAND),
}
#payload[TARGET_LEG] = list(TARGET_COORDS)  # only move this one leg

# No "speed" key → main_with_ik.py uses its safe default MAX_LIVE_DEG_PER_S

#print(f"Sending {TARGET_LEG} → {TARGET_COORDS}")
print(f"All other legs hold STAND position.")

data = pickle.dumps(payload)

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((HOST, PORT))
    s.sendall(data)

print(f"Sent. Holding for {HOLD_SECONDS}s... watch the leg.")
time.sleep(HOLD_SECONDS)
print("Done.")