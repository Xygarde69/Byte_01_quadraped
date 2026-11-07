#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/single_motor_easing.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Single-motor step test with easing.
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

import math

def ease_in_cubic(x: float) -> float:
    return 1 - math.pow(1 - x, 2.5)

def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


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
            #time.sleep(sleep_time)
            #print(sleep_time)
            pass
        elif sleep_time < -self.dt and self.report:
            print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
        
        self.iteration += 1
        return time.time() - self.start_time
        

def position_step_test(motor_id=3, can_interface='can0'):
    """
    Position step test for AK60-6 V3.0
    
    Args:
        motor_id: Motor CAN ID (default: 3)
        can_interface: CAN interface name (default: 'can0')
    """
    
    # Initialize motor
    print(f"Initializing AK60-6 V3.0 motor (ID={motor_id})...")
    motor = AK60V3Motor(motor_id=motor_id, can_interface=can_interface)
    
    try:
        # Setup phase
        print("\n=== SETUP PHASE ===")
        print("Enabling motor and setting zero position...")
        motor.enable()
        time.sleep(0.5)
        
        motor.set_zero_position(permanent=False)
        print("Zero position set. Waiting 3 seconds...")
        time.sleep(3)
        
        # Impedance gains for position control
        # AK60-6: KP range 0-500, KD range 0-5
        KP = 480.0  # Position stiffness (adjust based on your load)
        KD = 2   # Damping (adjust based on your load)
        
        print(f"Control gains: KP={KP}, KD={KD}")
        print("\n=== STARTING POSITION STEP TEST ===")
        print("Press Ctrl+C to stop\n")
        
        # Test parameters
        src = 0
        dest = 65
        mapped_value = map_range(abs(dest - src), 0, 360, 0, 1)
        eased_val = ease_in_cubic(mapped_value) 
        # mapped_eased_value = map_range(eased_val, 0, 1, 0, 360)
        # easing_const = 2
        # steps = int(mapped_eased_value * easing_const)
        steps = int(map_range(eased_val, 0, 1, 0, 800))
        angles = np.linspace(src, dest, steps)
        interval_between_angles = 1/1000000 				
        err_threshold = 2.0
    	  	
        # Loop timing
        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0)  # 50Hz control loop
        
        last_update = 0
        angle_idx = 0
        cycle = 0
        current_position_rad = 0.0
        neg_index = +1
    	
        cycle_time = time.time()

        for t in loop:
            # Read motor feedback
            if motor.read_feedback(timeout=0.001):
                current_position_rad = motor.position
                # print("Read Position : ", current_position_rad, " at time: ",time.time())
            
            current_angle_deg = np.degrees(current_position_rad)
            # Initial settling period (1 second)
            if t < 1.0:
                # Command zero position with impedance control
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
                # print("t=", t, "(last_update + interval_between_angles) = ", (last_update + interval_between_angles))
                if t > (last_update + interval_between_angles):
                    
                    # Determine target angle based on cycle direction
                    if cycle % 2 == 0:
                        # Even cycles: forward (0° → angle°)
                        target_angle_deg = src + angles[angle_idx]
                        neg_index = +1
                    else:
                        # Odd cycles: reverse (angle° → 0°)
                        target_angle_deg = dest - angles[angle_idx]
                        neg_index = -1
                    
                    # Convert to radians
                    target_angle_rad = np.radians(target_angle_deg)
                    
                    # Send position command via MIT mode
                    motor.send_mit_command(
                        position=target_angle_rad,
                        velocity=5*neg_index, # 25
                        kp=KP,
                        kd=KD,
                        torque=2.0*neg_index # 5
                    )
                    last_update = t
                    
                    # Calculate error
                    error_deg = current_angle_deg - target_angle_deg
                    
                    # Print status
                    print(f"t={t:.2f}s | Target: {target_angle_deg:6.1f}° | "
                          f"Actual: {current_angle_deg:6.1f}° | "
                          f"Error: {error_deg:+5.1f}° | "
                          f"Cycle: {cycle} | Step: {angle_idx+1}/{steps}")
                    
                    
                    
                    # Advance to next angle or cycle
                    if abs(error_deg) < err_threshold:
                        # print("Error is acceptable, move to next position")
                        angle_idx += 1
                        
                        if angle_idx >= len(angles):
                            # Completed full sweep
                            angle_idx = 0
                            cycle += 1
                            print(f"\n{'='*60}")
                            print(f"CYCLE {cycle} COMPLETED!, Time Taken: {time.time() - cycle_time}")
                            print(f"{'='*60}\n")
                            time.sleep(3)
                            cycle_time = time.time()
                    # else: stay at current angle until error is acceptable
            
            # Small delay for loop timing
            #time.sleep(0.00001)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
    
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Safe shutdown
        print("\n=== SHUTDOWN ===")
        print("Stopping motor...")
        
        # Send zero torque command with high damping to stop smoothly
        motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
        time.sleep(0.5)
        
        motor.disable()
        motor.close()
        print("Motor stopped and CAN bus closed")
        print("\nTest complete!")


if __name__ == '__main__':
    # Configuration
    MOTOR_ID = 1            # Your motor's CAN ID
    CAN_INTERFACE = 'can0'  # Your CAN interface
    
    print("="*70)
    print("  AK60-6 V3.0 Position Step Test")
    print("  Tests smooth position control with error tracking")
    print("="*70)
    
    # Run the test
    position_step_test(motor_id=MOTOR_ID, can_interface=CAN_INTERFACE)
        
        
