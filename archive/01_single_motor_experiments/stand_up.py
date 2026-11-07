#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/stand_up.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Stand-up variant derived from the position step test (MIT mode).
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
        
        # Calculate when next iteration should occur
        target_time = (self.iteration + 1) * self.dt
        sleep_time = target_time - current_time
        
        if sleep_time > 0:
            time.sleep(sleep_time)
        elif sleep_time < -self.dt and self.report:
            print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
        
        self.iteration += 1
        return time.time() - self.start_time


def position_step_test(motor_ids=[1, 3], can_interface='can0'):
    """
    Position step test for AK60-6 V3.0
    
    Args:
        motor_ids: List of motor CAN IDs (default: [1, 3])
        can_interface: CAN interface name (default: 'can0')
    """
    
    # Initialize motors
    print(f"Initializing AK60-6 V3.0 motors (IDs={motor_ids})...")
    motors = {}
    for motor_id in motor_ids:
        motors[motor_id] = AK60V3Motor(motor_id=motor_id, can_interface=can_interface)
        print(f"  Motor ID {motor_id} initialized")
    
    try:
        # Setup phase
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
        
        # Impedance gains for position control
        # AK60-6: KP range 0-500, KD range 0-5
        KP = 50.0  # Position stiffness (adjust based on your load)
        KD = 3.0   # Damping (adjust based on your load)
        
        print(f"Control gains: KP={KP}, KD={KD}")
        print("\n=== STARTING POSITION STEP TEST ===")
        print("Press Ctrl+C to stop\n")
        
        # Test parameters
        angles = np.linspace(0, -30, 300)  # 800 steps from 0° to 360°
        interval_between_angles = (50/1000)    # 8ms between position updates
        err_threshold = 1.0                # Position error threshold (degrees)
        
        # Loop timing
        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0)  # 50Hz control loop
        
        last_update = 0
        angle_idx = 0
        cycle = 0
        current_positions_rad = {motor_id: 0.0 for motor_id in motor_ids}
        
        for t in loop:
            # Read motor feedback for all motors
            for motor_id, motor in motors.items():
                if motor.read_feedback(timeout=0.001):
                    current_positions_rad[motor_id] = motor.position
            
            # Initial settling period (1 second)
            if t < 1.0:
                # Command zero position with impedance control
                for motor_id, motor in motors.items():
                    motor.send_mit_command(
                        position=0.0,
                        velocity=0.0,
                        kp=KP,
                        kd=KD,
                        torque=0.0
                    )
                last_update = t
                
            else:
                # Position stepping phase
                if t > last_update + interval_between_angles:
                    
                    # Determine target angle based on cycle direction
                    if cycle % 2 == 0:
                        # Even cycles: forward (0° → 360°)
                        target_angle_deg = angles[angle_idx]
                    else:
                        # Odd cycles: reverse (360° → 0°)
                        target_angle_deg = 360.0 - angles[angle_idx]
                    
                    # Convert to radians
                    target_angle_rad = np.radians(target_angle_deg)
                    
                    # Send position command via MIT mode to all motors
                    for motor_id, motor in motors.items():
                        motor.send_mit_command(
                            position=target_angle_rad,
                            velocity=0.0,
                            kp=KP,
                            kd=KD,
                            torque=0.0
                        )
                    
                    # Calculate errors for all motors
                    errors_deg = {}
                    for motor_id in motor_ids:
                        current_angle_deg = np.degrees(current_positions_rad[motor_id])
                        errors_deg[motor_id] = current_angle_deg - target_angle_deg
                    
                    # Print status for all motors
                    status_str = f"t={t:.2f}s | Target: {target_angle_deg:6.1f}° | "
                    for motor_id in motor_ids:
                        current_angle_deg = np.degrees(current_positions_rad[motor_id])
                        status_str += f"M{motor_id}: {current_angle_deg:6.1f}° (Err: {errors_deg[motor_id]:+5.1f}°) | "
                    status_str += f"Cycle: {cycle} | Step: {angle_idx}/800"
                    print(status_str)
                    
                    last_update = t
                    
                    # Advance to next angle or cycle (only if ALL motors within threshold)
                    all_within_threshold = all(abs(err) <= err_threshold for err in errors_deg.values())
                    
                    if all_within_threshold:
                        # Error is acceptable for all motors, move to next position
                        angle_idx += 1
                        
                        if angle_idx >= len(angles):
                            # Completed full sweep
                            angle_idx = 0
                            cycle += 1
                            print(f"\n{'='*60}")
                            print(f"CYCLE {cycle} COMPLETED!")
                            print(f"{'='*60}\n")
                    # else: stay at current angle until all motors' errors are acceptable
            
            # Small delay for loop timing
            time.sleep(0.001)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Safe shutdown
        print("\n=== SHUTDOWN ===")
        print("Stopping motors...")
        
        # Send zero torque command with high damping to stop smoothly
        for motor_id, motor in motors.items():
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
            print(f"  Motor ID {motor_id} stopped")
        time.sleep(0.5)
        
        for motor_id, motor in motors.items():
            motor.disable()
            motor.close()
            print(f"  Motor ID {motor_id} disabled and closed")
        print("All motors stopped and CAN bus closed")
        print("\nTest complete!")


if __name__ == '__main__':
    # Configuration
    MOTOR_IDS = [2]      # Motor CAN IDs
    CAN_INTERFACE = 'can0'  # Your CAN interface
    
    print("="*70)
    print("  AK60-6 V3.0 Position Step Test")
    print(f"  Testing motors: {MOTOR_IDS}")
    print("  Tests smooth position control with error tracking")
    print("="*70)
    
    # Run the test
    position_step_test(motor_ids=MOTOR_IDS, can_interface=CAN_INTERFACE)
