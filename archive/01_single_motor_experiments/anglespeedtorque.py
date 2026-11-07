#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/anglespeedtorque.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Position step test variant (the name suggests it records angle, speed
# and torque).
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Position Step Test
Performs smooth position stepping with error tracking
Based on MIT mode position control
"""

import numpy as np
import time
import sys

# Import your AK60 V3.0 control library
from ak60_v3_control import AK60V3Motor


class SoftRealtimeLoop:
    """
    Simple soft real-time loop implementation
    Maintains consistent loop timing with reporting
    """
    def __init__(self, dt=0.02, report=True, fade=0):
        self.dt = dt
        self.report = report
        self.fade = fade
        self.start_time = None
        self.iteration = 0
        
    def __enter__(self):
        self.start_time = time.time()
        return self
        
    def __exit__(self, *args):
        if self.report:
            elapsed = time.time() - self.start_time
            print(f"\nLoop Statistics:")
            print(f"  Total time: {elapsed:.2f}s")
            print(f"  Iterations: {self.iteration}")
            print(f"  Average rate: {self.iteration/elapsed:.1f}Hz")
    
    def __iter__(self):
        return self
    
    def __next__(self):
        if self.start_time is None:
            self.start_time = time.time()
        
        current_time = time.time() - self.start_time
        target_time = (self.iteration + 1) * self.dt
        sleep_time = target_time - current_time
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -self.dt and self.report:
            print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
        
        self.iteration += 1
        return time.time() - self.start_time


def position_step_test(motor_ids=[2, 3], can_interface='can0'):
    """
    Position step test for AK60-6 V3.0
    """

    print(f"Initializing AK60-6 V3.0 motors (IDs={motor_ids})...")
    motors = {}
    for motor_id in motor_ids:
        motors[motor_id] = AK60V3Motor(
            motor_id=motor_id,
            can_interface=can_interface
        )
        print(f"  Motor ID {motor_id} initialized")

    try:
        # ================= SETUP =================
        print("\n=== SETUP PHASE ===")
        for motor_id, motor in motors.items():
            motor.enable()
            print(f"  Motor ID {motor_id} enabled")

        time.sleep(0.5)

        for motor_id, motor in motors.items():
            motor.set_zero_position(permanent=False)
            print(f"  Motor ID {motor_id} zero position set")

        print("Waiting 3 seconds...")
        time.sleep(3)

        # ================= GAINS =================
        KP_MOTOR1 = 500.0   # rigid lock
        KD_MOTOR1 = 5.0

        KP_MOTOR3 = 100.0
        KD_MOTOR3 = 6.0

        print("\n=== STARTING POSITION STEP TEST ===")
        print("Motor ID 3 rotates in OPPOSITE direction")
        print("Press Ctrl+C to stop\n")

        # ================= TEST CONFIG =================
        angles = np.arange(0, 361, 30)   # 0° → 360° in 5° steps
        interval_between_angles = 0.008
        err_threshold = 1.0

        loop = SoftRealtimeLoop(dt=0.02, report=True)
        last_update = 0
        angle_idx = 0
        cycle = 0

        current_positions_rad = {mid: 0.0 for mid in motor_ids}

        for t in loop:
            # ---------- READ FEEDBACK ----------
            for mid, motor in motors.items():
                if motor.read_feedback(timeout=0.001):
                    current_positions_rad[mid] = motor.position

            # ---------- INITIAL HOLD ----------
            if t < 1.0:
                for mid, motor in motors.items():
                    if mid == 1:
                        motor.send_mit_command(0, 0, KP_MOTOR1, KD_MOTOR1, 0)
                    else:
                        motor.send_mit_command(0, 0, KP_MOTOR3, KD_MOTOR3, 0)
                last_update = t
                continue

            # ---------- POSITION STEPPING ----------
            if t > last_update + interval_between_angles:

                if cycle % 2 == 0:
                    target_angle_deg = angles[angle_idx]
                else:
                    target_angle_deg = 360.0 - angles[angle_idx]

                # 🔴 DIRECTION REVERSED HERE 🔴
                target_angle_rad = -np.radians(target_angle_deg)

                for mid, motor in motors.items():
                    if mid == 1:
                        motor.send_mit_command(0, 0, KP_MOTOR1, KD_MOTOR1, 0)
                    else:
                        motor.send_mit_command(
                            target_angle_rad, 0,
                            KP_MOTOR3, KD_MOTOR3, 0
                        )

                current_deg = np.degrees(current_positions_rad[3])
                error_deg = current_deg + target_angle_deg

                print(
                    f"t={t:5.2f}s | "
                    f"M1: {np.degrees(current_positions_rad[1]):6.1f}° | "
                    f"M3 Target: {-target_angle_deg:6.1f}° | "
                    f"M3 Actual: {current_deg:6.1f}° | "
                    f"Error: {error_deg:+5.1f}° | "
                    f"Cycle {cycle} | Step {angle_idx}"
                )

                last_update = t

                if abs(error_deg) <= err_threshold:
                    angle_idx += 1
                    if angle_idx >= len(angles):
                        angle_idx = 0
                        cycle += 1
                        print("\n" + "="*60)
                        print(f"CYCLE {cycle} COMPLETED")
                        print("="*60 + "\n")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nTest interrupted by user")

    finally:
        print("\n=== SHUTDOWN ===")
        for mid, motor in motors.items():
            motor.send_mit_command(0, 0, 0, 5, 0)
            time.sleep(0.1)
            motor.disable()
            motor.close()
            print(f"Motor ID {mid} stopped")

        print("All motors safely stopped")


if __name__ == "__main__":
    MOTOR_IDS = [1, 3]
    CAN_INTERFACE = "can0"

    print("="*70)
    print("AK60-6 V3.0 Position Step Test")
    print("Motor 1: Rigid Zero Lock")
    print("Motor 3: Opposite Direction Rotation")
    print("="*70)

    position_step_test(MOTOR_IDS, CAN_INTERFACE)

