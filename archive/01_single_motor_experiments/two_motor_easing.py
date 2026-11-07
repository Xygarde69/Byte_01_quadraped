#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/two_motor_easing.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Two motors on S-curve time-based trajectories.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Position Test - Multiple Motors with S-curve Time-based Trajectory
"""

import numpy as np
import time
import sys
import can
import math

from ak60_v3_control import AK60V3Motor


def s_curve_01(s: float) -> float:
    """
    Smooth S-curve from 0 to 1 for s in [0,1]:
    f(s) = 3s^2 - 2s^3
    """
    s = max(0.0, min(1.0, s))
    return 3.0 * s * s - 2.0 * s * s * s


class SoftRealtimeLoop:
    def __init__(self, dt=0.01, report=True, fade=0):
        self.dt = dt
        self.report = report
        self.fade = fade
        self.start_time = None
        self.iteration = 0
       
    def __enter__(self):
        self.start_time = time.time()
        return self
       
    def __exit__(self, *args):
        if self.report and self.start_time is not None:
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
            # Optional: time.sleep(sleep_time)
            # time.sleep(sleep_time)
            pass
        elif sleep_time < -self.dt and self.report:
            print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
       
        self.iteration += 1
        return time.time() - self.start_time


def position_step_test(motor_ids=[1, 2], can_interface='can0'):
    """
    Position test for AK60-6 V3.0 - Multiple Motors with S-curve trajectory
    """
    print(f"Initializing shared CAN bus on {can_interface}...")
    shared_bus = can.interface.Bus(
        channel=can_interface,
        bustype='socketcan',
        bitrate=1000000
    )
    print(f"✓ Shared CAN bus initialized")
   
    print(f"\nInitializing {len(motor_ids)} motors...")
    motors = {}
    for motor_id in motor_ids:
        motors[motor_id] = AK60V3Motor(
            motor_id=motor_id,
            can_interface=can_interface,
            bus=shared_bus
        )
        print(f"  Motor ID {motor_id} initialized")
   
    motor_id_to_index = {motor_id: i for i, motor_id in enumerate(motor_ids)}

    try:
        print("\n=== SETUP PHASE ===")
        print("Enabling motors and setting zero positions...")
        for motor_id, motor in motors.items():
            motor.enable()
            print(f"  Motor ID {motor_id} enabled")
        time.sleep(0.5)
       
        for motor_id, motor in motors.items():
            motor.set_zero_position(permanent=False)
            print(f"  Motor ID {motor_id} zero position set")
        print("Waiting 3 seconds...")
        time.sleep(3)
       
        # Impedance gains
        KP = 150.0
        KD = 2.5
       
        print(f"Control gains: KP={KP}, KD={KD}")
        print("\n=== STARTING POSITION TEST (S-curve) ===")
        print("Press Ctrl+C to stop\n")
       
        # Desired angles per motor (deg)
        src  = np.array([0.0, 0.0])
        dest = np.array([1.0, 4.0])

        # === S-curve timing parameters ===
        BASE_ANGLE = 360.0       # Reference angle for 1 second
        BASE_TIME  = 1.0        # Seconds for BASE_ANGLE
        MIN_TIME   = 0.08       # Minimum move time for very small angles
        SMALL_ANGLE_THRESH = 5.0  # Below this, treat as "small angle"

        # Pre-compute move times per motor
        angle_diff = np.abs(dest - src)          # degrees
        move_time = np.zeros_like(angle_diff)
        for i in range(len(angle_diff)):
            if angle_diff[i] < SMALL_ANGLE_THRESH:
                # Small move: very short time, essentially "snap" but still smooth
                move_time[i] = MIN_TIME
            else:
                # Scale time with angle, but not below MIN_TIME
                move_time[i] = max(MIN_TIME, BASE_TIME * (angle_diff[i] / BASE_ANGLE))

        print("Per-motor planned move times (s):", move_time)

        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0)  # ~100 Hz
       
        current_positions_rad = {motor_id: 0.0 for motor_id in motor_ids}
        current_angle_deg = {motor_id: 0.0 for motor_id in motor_ids}

        cycle = 0
        cycle_time = time.time()
        direction = +1  # +1: src->dest, -1: dest->src
        move_start_time = None

        for t in loop:
            # Feedback
            for motor_id, motor in motors.items():
                if motor.read_feedback(timeout=0.0005):
                    current_positions_rad[motor_id] = motor.position
                    current_angle_deg[motor_id] = math.degrees(current_positions_rad[motor_id])

            # Initial 1 s settle at zero
            if t < 1.0 and cycle == 0:
                for motor_id, motor in motors.items():
                    motor.send_mit_command(
                        position=0.0,
                        velocity=0.0,
                        kp=KP,
                        kd=KD,
                        torque=0.0
                    )
                move_start_time = t  # prepare for first move
                continue

            # Start new move if needed
            if move_start_time is None:
                move_start_time = t
                cycle_time = time.time()

            # Compute target for each motor based on S-curve
            elapsed = t - move_start_time
            targets_deg = np.zeros_like(src)

            all_finished = True
            for i, motor_id in enumerate(motor_ids):
                T_i = move_time[i]
                if elapsed >= T_i:
                    # Movement finished for this motor
                    targets_deg[i] = dest[i] if direction > 0 else src[i]
                else:
                    all_finished = False
                    s = elapsed / T_i
                    s_sc = s_curve_01(s)
                    if direction > 0:
                        targets_deg[i] = src[i] + (dest[i] - src[i]) * s_sc
                    else:
                        targets_deg[i] = dest[i] + (src[i] - dest[i]) * s_sc

            # Send command to all motors
            targets_rad = np.radians(targets_deg)
            for i, motor_id in enumerate(motor_ids):
                motor = motors[motor_id]
                motor.send_mit_command(
                    position=targets_rad[i],
                    velocity=0.0,
                    kp=KP,
                    kd=KD,
                    torque=3.0 if direction > 0 else 0.0
                )

            # Status print (reduce frequency)
            if loop.iteration % 20 == 0:
                for i, motor_id in enumerate(motor_ids):
                    err = current_angle_deg[motor_id] - targets_deg[i]
                    print(
                        f"t={t:.2f}s | M{motor_id} | "
                        f"Target: {targets_deg[i]:6.1f}° | "
                        f"Actual: {current_angle_deg[motor_id]:6.1f}° | "
                        f"Error: {err:+6.1f}° | "
                        f"Cycle: {cycle} | "
                        f"Dir: {direction:+d}"
                    )

            # If all motors finished their segment, switch direction
            if all_finished:
                cycle += 1
                print(f"\n{'='*60}")
                print(f"CYCLE {cycle} COMPLETED! Time Taken: {time.time() - cycle_time:.2f}s")
                print(f"{'='*60}\n")

                # Swap direction: src->dest then dest->src
                direction *= -1
                move_start_time = None
                time.sleep(1.0)  # pause between cycles

    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
   
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
   
    finally:
        print("\n=== SHUTDOWN ===")
        print("Stopping motors...")
        for motor_id, motor in motors.items():
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
            time.sleep(0.001)
            print(f"  Motor ID {motor_id} stopped")
        time.sleep(0.5)
       
        for motor_id, motor in motors.items():
            motor.disable()
            print(f"  Motor ID {motor_id} disabled")
       
        shared_bus.shutdown()
        print("Shared CAN bus closed")
        print("\nTest complete!")


if __name__ == '__main__':
    MOTOR_IDS = [1, 2]
    CAN_INTERFACE = 'can0'
   
    print("="*70)
    print("  AK60-6 V3.0 Position Test - Multiple Motors, S-curve")
    print("  Time-based S-curve profile with load-ready impedance")
    print("="*70)
   
    position_step_test(motor_ids=MOTOR_IDS, can_interface=CAN_INTERFACE)