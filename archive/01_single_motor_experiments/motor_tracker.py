#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/motor_tracker.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Moves one motor in proportion to a pixel error (P_GAIN degrees per
# pixel, REVERSE_MOTOR flag) - a target-tracking demo.
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import time
import socket
import json
import threading
import can
from ak60_v3_control import AK60V3Motor

# ==========================================
# --- 1. CONFIGURATION & TUNING ---
# ==========================================
P_GAIN = 0.04             # Sensitivity: Deg to move per pixel
REVERSE_MOTOR = True      # Set to True based on your successful test!
DEADBAND_PIXELS = 30.0    # Stop moving if error is within this many pixels
MAX_DEG_PER_S = 80.0      # Speed limit: Maximum degrees the motor can turn per second

# Global State
desired_target_angle_deg = 0.0  # Where the vision system WANTS the motor to go
current_cmd_angle_deg = 0.0     # Where the motor is ACTUALLY commanded to go right now
last_error_x = 0.0
is_locked = False

# --- 2. The Slew Rate Limiter (From your seniors' code) ---
def limit_target_step(current_cmd_deg: float, requested_deg: float, max_deg_per_s: float, dt: float) -> float:
    max_step = max_deg_per_s * dt
    delta = requested_deg - current_cmd_deg

    if delta > max_step:
        return current_cmd_deg + max_step
    if delta < -max_step:
        return current_cmd_deg - max_step
    return requested_deg

# --- 3. SoftRealtimeLoop ---
class SoftRealtimeLoop:
    def __init__(self, dt=0.02):
        self.dt = dt
        self.start_time = None
        self.iteration = 0
    def __iter__(self): return self
    def __next__(self):
        if self.start_time is None: self.start_time = time.time()
        current_time = time.time() - self.start_time
        target_time = (self.iteration + 1) * self.dt
        sleep_time = target_time - current_time
        if sleep_time > 0: time.sleep(sleep_time)
        self.iteration += 1
        return time.time() - self.start_time

# --- 4. UDP Listener ---
def udp_listener():
    global desired_target_angle_deg, last_error_x, is_locked, current_cmd_angle_deg
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", 5005))
    
    while True:
        try:
            data, _ = sock.recvfrom(1024)
            msg = json.loads(data.decode('utf-8'))
            last_error_x = msg.get("error_x", 0)
            
            if abs(last_error_x) <= DEADBAND_PIXELS:
                is_locked = True
                # Lock the desired target to exactly where the motor currently is
                desired_target_angle_deg = current_cmd_angle_deg 
            else:
                is_locked = False
                delta = last_error_x * P_GAIN
                
                # We calculate the next desired step relative to where the motor currently is
                # This prevents "wind-up" lag if the UDP packets arrive faster than the motor can move
                if REVERSE_MOTOR:
                    desired_target_angle_deg = current_cmd_angle_deg + delta
                else:
                    desired_target_angle_deg = current_cmd_angle_deg - delta
                    
        except Exception:
            pass

# --- 5. Main Control Loop ---
def run_target_lock(motor_id=2, can_interface='can0'):
    global current_cmd_angle_deg, desired_target_angle_deg, last_error_x, is_locked
    motor = AK60V3Motor(motor_id=motor_id, can_interface=can_interface)
    
    try:
        motor.enable()
        time.sleep(0.5)
        motor.set_zero_position(permanent=False)
        print(f"🚀 Tracking Active | Max Speed: {MAX_DEG_PER_S}°/s | DEADBAND: ±{DEADBAND_PIXELS}px")
        print("-" * 75)

        KP, KD = 40.0, 0.5
        loop = SoftRealtimeLoop(dt=0.02) # 50Hz loop
        
        for t in loop:
            # 1. Apply the Smoothing Limiter
            current_cmd_angle_deg = limit_target_step(
                current_cmd_deg=current_cmd_angle_deg,
                requested_deg=desired_target_angle_deg,
                max_deg_per_s=MAX_DEG_PER_S,
                dt=loop.dt
            )
            
            # 2. Convert to Radians and Send
            target_rad = np.radians(current_cmd_angle_deg)
            
            try:
                motor.send_mit_command(
                    position=target_rad,
                    velocity=0.0,
                    kp=KP,
                    kd=KD,
                    torque=0.0
                )
            except can.CanOperationError:
                pass
            
            motor.read_feedback(timeout=0.001)

            if loop.iteration % 5 == 0:
                lock_status = "🟩 LOCKED" if is_locked else "🟨 MOVING"
                print(f"Time: {t:.2f}s | {lock_status} | Pxl Error: {last_error_x:+6.1f} | Cmd: {current_cmd_angle_deg:+6.2f}°   ", end='\r')

    except KeyboardInterrupt:
        print("\n\nStopping Tracking...")
    finally:
        try:
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
        except: pass
        time.sleep(0.5)
        motor.disable()
        motor.close()
        print("System Offline.")

if __name__ == '__main__':
    threading.Thread(target=udp_listener, daemon=True).start()
    run_target_lock(motor_id=2, can_interface='can1')