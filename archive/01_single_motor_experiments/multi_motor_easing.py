#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/multi_motor_easing.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Multiple motors on S-curve time-based trajectories.
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
        KP = 300.0
        KD = 1
       
        print(f"Control gains: KP={KP}, KD={KD}")
        print("\n=== STARTING POSITION TEST (S-curve) ===")
        print("Press Ctrl+C to stop\n")
       
        # Desired angles per motor (deg)
        src  = np.array([0.0])
        dest = np.array([40.0])

        # === S-curve timing parameters ===
        BASE_ANGLE = 360.0       # Reference angle for 1 second
        BASE_TIME  = 5.0        # Seconds for BASE_ANGLE
        MIN_TIME   = 0.2       # Minimum move time for very small angles
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
                time.sleep(5)  # pause between cycles

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
    MOTOR_IDS = [2]
    CAN_INTERFACE = 'can0'
   
    print("="*70)
    print("  AK60-6 V3.0 Position Test - Multiple Motors, S-curve")
    print("  Time-based S-curve profile with load-ready impedance")
    print("="*70)
   
    position_step_test(motor_ids=MOTOR_IDS, can_interface=CAN_INTERFACE)


# SAMPLE OUTPUT

# === STARTING POSITION TEST (S-curve) ===
# Press Ctrl+C to stop

# Per-motor planned move times (s): [0.08       0.11111111 0.16666667]
# t=1.13s | M1 | Target:   10.0° | Actual:    0.0° | Error:  -10.0° | Cycle: 0 | Dir: +1
# t=1.13s | M2 | Target:   40.0° | Actual:    0.0° | Error:  -40.0° | Cycle: 0 | Dir: +1
# t=1.13s | M3 | Target:   51.2° | Actual:    0.0° | Error:  -51.2° | Cycle: 0 | Dir: +1

# ============================================================
# CYCLE 1 COMPLETED! Time Taken: 1.17s
# ============================================================

# t=2.19s | M1 | Target:    8.9° | Actual:    0.0° | Error:   -8.9° | Cycle: 1 | Dir: -1
# t=2.19s | M2 | Target:   37.6° | Actual:    0.0° | Error:  -37.6° | Cycle: 1 | Dir: -1
# t=2.19s | M3 | Target:   58.3° | Actual:    0.0° | Error:  -58.3° | Cycle: 1 | Dir: -1
# t=2.24s | M1 | Target:    1.1° | Actual:    0.0° | Error:   -1.1° | Cycle: 1 | Dir: -1
# t=2.24s | M2 | Target:   15.5° | Actual:    0.0° | Error:  -15.5° | Cycle: 1 | Dir: -1
# t=2.24s | M3 | Target:   40.3° | Actual:    0.0° | Error:  -40.3° | Cycle: 1 | Dir: -1
# t=2.28s | M1 | Target:    0.0° | Actual:    0.0° | Error:   +0.0° | Cycle: 1 | Dir: -1
# t=2.28s | M2 | Target:    0.1° | Actual:    0.0° | Error:   -0.1° | Cycle: 1 | Dir: -1
# t=2.28s | M3 | Target:   17.1° | Actual:    0.0° | Error:  -17.1° | Cycle: 1 | Dir: -1
# t=2.33s | M1 | Target:    0.0° | Actual:    1.0° | Error:   +1.0° | Cycle: 1 | Dir: -1
# t=2.33s | M2 | Target:    0.0° | Actual:    0.0° | Error:   +0.0° | Cycle: 1 | Dir: -1
# t=2.33s | M3 | Target:    0.7° | Actual:    7.6° | Error:   +6.9° | Cycle: 1 | Dir: -1

# ============================================================
# CYCLE 2 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=3.40s | M1 | Target:    6.9° | Actual:    1.0° | Error:   -5.9° | Cycle: 2 | Dir: +1
# t=3.40s | M2 | Target:   17.2° | Actual:   26.1° | Error:   +8.9° | Cycle: 2 | Dir: +1
# t=3.40s | M3 | Target:   13.1° | Actual:   63.4° | Error:  +50.3° | Cycle: 2 | Dir: +1
# t=3.45s | M1 | Target:   10.0° | Actual:    7.7° | Error:   -2.3° | Cycle: 2 | Dir: +1
# t=3.45s | M2 | Target:   38.7° | Actual:   26.1° | Error:  -12.6° | Cycle: 2 | Dir: +1
# t=3.45s | M3 | Target:   38.3° | Actual:   52.1° | Error:  +13.8° | Cycle: 2 | Dir: +1
# t=3.49s | M1 | Target:   10.0° | Actual:    7.7° | Error:   -2.3° | Cycle: 2 | Dir: +1
# t=3.49s | M2 | Target:   40.0° | Actual:    3.9° | Error:  -36.1° | Cycle: 2 | Dir: +1
# t=3.49s | M3 | Target:   57.5° | Actual:   25.1° | Error:  -32.4° | Cycle: 2 | Dir: +1

# ============================================================
# CYCLE 3 COMPLETED! Time Taken: 0.18s
# ============================================================

# t=4.54s | M1 | Target:    8.6° | Actual:    0.0° | Error:   -8.6° | Cycle: 3 | Dir: -1
# t=4.54s | M2 | Target:   37.0° | Actual:    0.3° | Error:  -36.7° | Cycle: 3 | Dir: -1
# t=4.54s | M3 | Target:   57.9° | Actual:    3.0° | Error:  -54.9° | Cycle: 3 | Dir: -1
# t=4.59s | M1 | Target:    1.0° | Actual:    0.0° | Error:   -1.0° | Cycle: 3 | Dir: -1
# t=4.59s | M2 | Target:   15.4° | Actual:    0.0° | Error:  -15.4° | Cycle: 3 | Dir: -1
# t=4.59s | M3 | Target:   40.2° | Actual:    0.4° | Error:  -39.8° | Cycle: 3 | Dir: -1
# t=4.66s | M1 | Target:    0.0° | Actual:   11.0° | Error:  +11.0° | Cycle: 3 | Dir: -1
# t=4.66s | M2 | Target:    0.0° | Actual:    0.0° | Error:   +0.0° | Cycle: 3 | Dir: -1
# t=4.66s | M3 | Target:    7.0° | Actual:   19.4° | Error:  +12.4° | Cycle: 3 | Dir: -1

# ============================================================
# CYCLE 4 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=5.71s | M1 | Target:    0.8° | Actual:   11.0° | Error:  +10.2° | Cycle: 4 | Dir: +1
# t=5.71s | M2 | Target:    1.6° | Actual:    0.0° | Error:   -1.6° | Cycle: 4 | Dir: +1
# t=5.71s | M3 | Target:    1.1° | Actual:   48.0° | Error:  +46.9° | Cycle: 4 | Dir: +1
# t=5.73s | M1 | Target:    4.2° | Actual:   13.7° | Error:   +9.5° | Cycle: 4 | Dir: +1
# t=5.73s | M2 | Target:    9.8° | Actual:   43.6° | Error:  +33.8° | Cycle: 4 | Dir: +1
# t=5.73s | M3 | Target:    7.1° | Actual:   62.3° | Error:  +55.2° | Cycle: 4 | Dir: +1
# t=5.76s | M1 | Target:    8.6° | Actual:   13.7° | Error:   +5.1° | Cycle: 4 | Dir: +1
# t=5.76s | M2 | Target:   23.1° | Actual:   43.6° | Error:  +20.5° | Cycle: 4 | Dir: +1
# t=5.76s | M3 | Target:   18.4° | Actual:   53.1° | Error:  +34.7° | Cycle: 4 | Dir: +1
# t=5.81s | M1 | Target:   10.0° | Actual:    0.2° | Error:   -9.8° | Cycle: 4 | Dir: +1
# t=5.81s | M2 | Target:   39.9° | Actual:   26.9° | Error:  -13.0° | Cycle: 4 | Dir: +1
# t=5.81s | M3 | Target:   43.0° | Actual:   24.9° | Error:  -18.1° | Cycle: 4 | Dir: +1
# t=5.86s | M1 | Target:   10.0° | Actual:    0.2° | Error:   -9.8° | Cycle: 4 | Dir: +1
# t=5.86s | M2 | Target:   40.0° | Actual:    0.2° | Error:  -39.8° | Cycle: 4 | Dir: +1
# t=5.86s | M3 | Target:   60.0° | Actual:    2.6° | Error:  -57.4° | Cycle: 4 | Dir: +1

# ============================================================
# CYCLE 5 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=6.89s | M1 | Target:    8.6° | Actual:    0.2° | Error:   -8.4° | Cycle: 5 | Dir: -1
# t=6.89s | M2 | Target:   36.9° | Actual:    0.2° | Error:  -36.7° | Cycle: 5 | Dir: -1
# t=6.89s | M3 | Target:   57.8° | Actual:    8.3° | Error:  -49.5° | Cycle: 5 | Dir: -1
# t=6.95s | M1 | Target:    0.0° | Actual:    0.2° | Error:   +0.2° | Cycle: 5 | Dir: -1
# t=6.95s | M2 | Target:    8.4° | Actual:    0.2° | Error:   -8.2° | Cycle: 5 | Dir: -1
# t=6.95s | M3 | Target:   32.7° | Actual:   34.7° | Error:   +2.0° | Cycle: 5 | Dir: -1
# t=7.03s | M1 | Target:    0.0° | Actual:   13.7° | Error:  +13.7° | Cycle: 5 | Dir: -1
# t=7.03s | M2 | Target:    0.0° | Actual:   38.4° | Error:  +38.4° | Cycle: 5 | Dir: -1
# t=7.03s | M3 | Target:    0.3° | Actual:   58.8° | Error:  +58.5° | Cycle: 5 | Dir: -1

# ============================================================
# CYCLE 6 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=8.07s | M1 | Target:    1.5° | Actual:   13.7° | Error:  +12.2° | Cycle: 6 | Dir: +1
# t=8.07s | M2 | Target:    3.3° | Actual:   43.7° | Error:  +40.4° | Cycle: 6 | Dir: +1
# t=8.07s | M3 | Target:    2.3° | Actual:   63.2° | Error:  +60.9° | Cycle: 6 | Dir: +1
# t=8.09s | M1 | Target:    5.3° | Actual:   13.7° | Error:   +8.4° | Cycle: 6 | Dir: +1
# t=8.09s | M2 | Target:   12.6° | Actual:   43.7° | Error:  +31.1° | Cycle: 6 | Dir: +1
# t=8.09s | M3 | Target:    9.3° | Actual:   63.1° | Error:  +53.8° | Cycle: 6 | Dir: +1
# t=8.12s | M1 | Target:    9.7° | Actual:   11.7° | Error:   +2.0° | Cycle: 6 | Dir: +1
# t=8.12s | M2 | Target:   28.2° | Actual:   43.7° | Error:  +15.5° | Cycle: 6 | Dir: +1
# t=8.12s | M3 | Target:   23.5° | Actual:   46.1° | Error:  +22.6° | Cycle: 6 | Dir: +1
# t=8.18s | M1 | Target:   10.0° | Actual:   11.7° | Error:   +1.7° | Cycle: 6 | Dir: +1
# t=8.18s | M2 | Target:   40.0° | Actual:    4.9° | Error:  -35.1° | Cycle: 6 | Dir: +1
# t=8.18s | M3 | Target:   53.9° | Actual:   19.9° | Error:  -34.0° | Cycle: 6 | Dir: +1

# ============================================================
# CYCLE 7 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=9.23s | M1 | Target:    9.3° | Actual:   -0.1° | Error:   -9.4° | Cycle: 7 | Dir: -1
# t=9.23s | M2 | Target:   38.6° | Actual:    1.2° | Error:  -37.4° | Cycle: 7 | Dir: -1
# t=9.23s | M3 | Target:   59.0° | Actual:    0.7° | Error:  -58.3° | Cycle: 7 | Dir: -1
# t=9.25s | M1 | Target:    5.9° | Actual:    7.5° | Error:   +1.6° | Cycle: 7 | Dir: -1
# t=9.25s | M2 | Target:   30.4° | Actual:    1.2° | Error:  -29.2° | Cycle: 7 | Dir: -1
# t=9.25s | M3 | Target:   53.0° | Actual:   18.3° | Error:  -34.7° | Cycle: 7 | Dir: -1
# t=9.28s | M1 | Target:    1.4° | Actual:   13.4° | Error:  +12.0° | Cycle: 7 | Dir: -1
# t=9.28s | M2 | Target:   16.8° | Actual:    1.2° | Error:  -15.6° | Cycle: 7 | Dir: -1
# t=9.28s | M3 | Target:   41.5° | Actual:   44.4° | Error:   +2.9° | Cycle: 7 | Dir: -1
# t=9.32s | M1 | Target:    0.0° | Actual:   13.7° | Error:  +13.7° | Cycle: 7 | Dir: -1
# t=9.32s | M2 | Target:    0.8° | Actual:   43.0° | Error:  +42.2° | Cycle: 7 | Dir: -1
# t=9.32s | M3 | Target:   20.3° | Actual:   62.1° | Error:  +41.8° | Cycle: 7 | Dir: -1
# t=9.38s | M1 | Target:    0.0° | Actual:   13.6° | Error:  +13.6° | Cycle: 7 | Dir: -1
# t=9.38s | M2 | Target:    0.0° | Actual:   43.7° | Error:  +43.7° | Cycle: 7 | Dir: -1
# t=9.38s | M3 | Target:    0.0° | Actual:   63.3° | Error:  +63.3° | Cycle: 7 | Dir: -1

# ============================================================
# CYCLE 8 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=10.41s | M1 | Target:    1.8° | Actual:    8.5° | Error:   +6.7° | Cycle: 8 | Dir: +1
# t=10.41s | M2 | Target:    3.9° | Actual:   43.7° | Error:  +39.8° | Cycle: 8 | Dir: +1
# t=10.41s | M3 | Target:    2.7° | Actual:   56.3° | Error:  +53.6° | Cycle: 8 | Dir: +1
# t=10.45s | M1 | Target:    8.6° | Actual:    1.2° | Error:   -7.4° | Cycle: 8 | Dir: +1
# t=10.45s | M2 | Target:   23.0° | Actual:   23.2° | Error:   +0.2° | Cycle: 8 | Dir: +1
# t=10.45s | M3 | Target:   18.3° | Actual:   30.4° | Error:  +12.1° | Cycle: 8 | Dir: +1
# t=10.49s | M1 | Target:   10.0° | Actual:    0.0° | Error:  -10.0° | Cycle: 8 | Dir: +1
# t=10.49s | M2 | Target:   38.3° | Actual:    0.4° | Error:  -37.9° | Cycle: 8 | Dir: +1
# t=10.49s | M3 | Target:   37.4° | Actual:    6.1° | Error:  -31.3° | Cycle: 8 | Dir: +1
# t=10.55s | M1 | Target:   10.0° | Actual:   -0.1° | Error:  -10.1° | Cycle: 8 | Dir: +1
# t=10.55s | M2 | Target:   40.0° | Actual:    0.1° | Error:  -39.9° | Cycle: 8 | Dir: +1
# t=10.55s | M3 | Target:   59.9° | Actual:    0.4° | Error:  -59.5° | Cycle: 8 | Dir: +1

# ============================================================
# CYCLE 9 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=11.59s | M1 | Target:    8.5° | Actual:    0.0° | Error:   -8.5° | Cycle: 9 | Dir: -1
# t=11.59s | M2 | Target:   36.8° | Actual:    0.0° | Error:  -36.8° | Cycle: 9 | Dir: -1
# t=11.59s | M3 | Target:   57.8° | Actual:    1.0° | Error:  -56.8° | Cycle: 9 | Dir: -1
# t=11.60s | M1 | Target:    5.4° | Actual:   10.5° | Error:   +5.1° | Cycle: 9 | Dir: -1
# t=11.60s | M2 | Target:   29.3° | Actual:   13.0° | Error:  -16.3° | Cycle: 9 | Dir: -1
# t=11.60s | M3 | Target:   52.2° | Actual:   18.7° | Error:  -33.5° | Cycle: 9 | Dir: -1
# t=11.62s | M1 | Target:    2.0° | Actual:   10.5° | Error:   +8.5° | Cycle: 9 | Dir: -1
# t=11.62s | M2 | Target:   19.1° | Actual:   24.1° | Error:   +5.0° | Cycle: 9 | Dir: -1
# t=11.62s | M3 | Target:   43.6° | Actual:   46.2° | Error:   +2.6° | Cycle: 9 | Dir: -1
# t=11.64s | M1 | Target:    0.0° | Actual:   13.6° | Error:  +13.6° | Cycle: 9 | Dir: -1
# t=11.64s | M2 | Target:    9.0° | Actual:   42.9° | Error:  +33.9° | Cycle: 9 | Dir: -1
# t=11.64s | M3 | Target:   33.4° | Actual:   62.1° | Error:  +28.7° | Cycle: 9 | Dir: -1
# t=11.66s | M1 | Target:    0.0° | Actual:   13.7° | Error:  +13.7° | Cycle: 9 | Dir: -1
# t=11.66s | M2 | Target:    2.0° | Actual:   43.7° | Error:  +41.7° | Cycle: 9 | Dir: -1
# t=11.66s | M3 | Target:   23.1° | Actual:   62.8° | Error:  +39.7° | Cycle: 9 | Dir: -1
# t=11.69s | M1 | Target:    0.0° | Actual:   12.9° | Error:  +12.9° | Cycle: 9 | Dir: -1
# t=11.69s | M2 | Target:    0.0° | Actual:   42.2° | Error:  +42.2° | Cycle: 9 | Dir: -1
# t=11.69s | M3 | Target:   10.4° | Actual:   45.3° | Error:  +34.9° | Cycle: 9 | Dir: -1

# ============================================================
# CYCLE 10 COMPLETED! Time Taken: 0.17s
# ============================================================

# t=12.74s | M1 | Target:    0.2° | Actual:   12.9° | Error:  +12.7° | Cycle: 10 | Dir: +1
# t=12.74s | M2 | Target:    0.5° | Actual:    7.4° | Error:   +6.9° | Cycle: 10 | Dir: +1
# t=12.74s | M3 | Target:    0.3° | Actual:   15.4° | Error:  +15.1° | Cycle: 10 | Dir: +1
# t=12.77s | M1 | Target:    3.1° | Actual:   -0.1° | Error:   -3.2° | Cycle: 10 | Dir: +1
# t=12.77s | M2 | Target:    7.0° | Actual:    0.5° | Error:   -6.5° | Cycle: 10 | Dir: +1
# t=12.77s | M3 | Target:    5.0° | Actual:    1.1° | Error:   -3.9° | Cycle: 10 | Dir: +1
# t=12.79s | M1 | Target:    6.8° | Actual:   -0.1° | Error:   -6.9° | Cycle: 10 | Dir: +1
# t=12.79s | M2 | Target:   17.0° | Actual:    0.0° | Error:  -17.0° | Cycle: 10 | Dir: +1
# t=12.79s | M3 | Target:   12.9° | Actual:    0.4° | Error:  -12.5° | Cycle: 10 | Dir: +1
# t=12.81s | M1 | Target:    9.5° | Actual:   -0.1° | Error:   -9.6° | Cycle: 10 | Dir: +1
# t=12.81s | M2 | Target:   27.3° | Actual:    0.0° | Error:  -27.3° | Cycle: 10 | Dir: +1
# t=12.81s | M3 | Target:   22.5° | Actual:    0.4° | Error:  -22.1° | Cycle: 10 | Dir: +1
# t=12.83s | M1 | Target:   10.0° | Actual:   -0.1° | Error:  -10.1° | Cycle: 10 | Dir: +1
# t=12.83s | M2 | Target:   35.7° | Actual:    0.0° | Error:  -35.7° | Cycle: 10 | Dir: +1
# t=12.83s | M3 | Target:   32.7° | Actual:    0.4° | Error:  -32.3° | Cycle: 10 | Dir: +1
# t=12.85s | M1 | Target:   10.0° | Actual:   -0.1° | Error:  -10.1° | Cycle: 10 | Dir: +1
# t=12.85s | M2 | Target:   39.9° | Actual:    0.0° | Error:  -39.9° | Cycle: 10 | Dir: +1
# t=12.85s | M3 | Target:   43.1° | Actual:    3.0° | Error:  -40.1° | Cycle: 10 | Dir: +1
# t=12.87s | M1 | Target:   10.0° | Actual:   11.8° | Error:   +1.8° | Cycle: 10 | Dir: +1
# t=12.87s | M2 | Target:   40.0° | Actual:    0.0° | Error:  -40.0° | Cycle: 10 | Dir: +1
# t=12.87s | M3 | Target:   54.7° | Actual:   25.7° | Error:  -29.0° | Cycle: 10 | Dir: +1