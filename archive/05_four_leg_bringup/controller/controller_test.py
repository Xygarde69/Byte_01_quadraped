#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/controller/controller_test.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Prints all inputs of the first detected gamepad (buttons and sticks).
# Ctrl+C to exit.
# ════════════════════════════════════════════════════════════════════════
"""
Quick test — prints all inputs from the first detected gamepad.
Press buttons and move sticks to verify values are coming in.
Ctrl+C to exit.
"""
from evdev import InputDevice, categorize, ecodes, list_devices

# ── Find the controller ───────────────────────────────────────────────────────
devices = [InputDevice(path) for path in list_devices()]
gamepad = InputDevice("/dev/input/event2")

for dev in devices:
    if any(k in dev.name.lower() for k in ("xbox", "wireless", "redgear", "gamepad", "controller")):
        gamepad = dev
        break

if gamepad is None:
    print("No gamepad found. Devices available:")
    for dev in devices:
        print(f"  {dev.path}  →  {dev.name}")
    exit(1)

print(f"Found: {gamepad.name}  ({gamepad.path})")
print("Press buttons / move sticks — Ctrl+C to stop\n")

# ── Read events ───────────────────────────────────────────────────────────────
for event in gamepad.read_loop():
    if event.type == ecodes.EV_KEY:
        key = categorize(event)
        print(f"  BUTTON  code={event.code}  value={event.value}  ({key})")
    elif event.type == ecodes.EV_ABS:
        print(f"  AXIS    code={event.code}  value={event.value}")
