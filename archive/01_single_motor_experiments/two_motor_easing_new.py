#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/two_motor_easing_new.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Two-motor easing test (bug-fixed revision of two_motor_easing.py).
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Position Step Test - Multiple Motors (FIXED)
Performs smooth position stepping with error tracking
Based on MIT mode position control
"""

import numpy as np
import time
import sys
import can

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
            pass
        elif sleep_time < -self.dt and self.report:
            print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
        
        self.iteration += 1
        return time.time() - self.start_time
        

def position_step_test(motor_ids=[1, 2], can_interface='can0'):
    """
    Position step test for AK60-6 V3.0 - Multiple Motors
    
    Args:
        motor_ids: List of motor CAN IDs (default: [1, 2])
        can_interface: CAN interface name (default: 'can0')
    """
    
    # ========== FIX: Create ONE shared CAN bus ==========
    print(f"Initializing shared CAN bus on {can_interface}...")
    shared_bus = can.interface.Bus(
        channel=can_interface,
        bustype='socketcan',
        bitrate=1000000
    )
    print(f"✓ Shared CAN bus initialized")
    
    # Initialize motors with the shared bus
    print(f"\nInitializing {len(motor_ids)} motors...")
    motors = {}
    for motor_id in motor_ids:
        # Pass the shared bus to each motor instance
        motors[motor_id] = AK60V3Motor(
            motor_id=motor_id, 
            can_interface=can_interface,
            bus=shared_bus  # Use shared bus
        )
        print(f"  Motor ID {motor_id} initialized")
    
    # Create an index mapping
    motor_id_to_index = {motor_id: i for i, motor_id in enumerate(motor_ids)}

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
        KP = 150.0  # Position stiffness
        KD = 1.5      # Damping
        
        print(f"Control gains: KP={KP}, KD={KD}")
        print("\n=== STARTING POSITION STEP TEST ===")
        print("Press Ctrl+C to stop\n")
        
        # Test parameters
        src  = np.array([0, 0], dtype=float)
        dest = np.array([65, 20], dtype=float)

        # Per-motor mapping & easing
        mapped_value = map_range(np.abs(dest - src), 0, 360, 0, 1)
        eased_value  = np.array([ease_in_cubic(x) for x in mapped_value])

        # Steps per motor
        steps_per_motor = map_range(eased_value, 0, 1, 0, 400).astype(int)
        steps_per_motor = np.maximum(steps_per_motor, 1)

        # Per-motor trajectories
        angles = []
        for i in range(len(src)):
            motor_angles = np.linspace(src[i], dest[i], steps_per_motor[i])
            angles.append(motor_angles)

        # ========== FIX: Increase interval to prevent buffer overflow ==========
        interval_between_angles = 1/1000000
        err_threshold = 2.0
    	  	
        # Loop timing
        loop = SoftRealtimeLoop(dt=0.01, report=True, fade=0)  # 50Hz control loop
        
        last_update = 0
        angle_idx = np.zeros(len(src), dtype=int)

        cycle = 0
        current_positions_rad = {motor_id: 0.0 for motor_id in motor_ids}
        current_angle_deg = {motor_id: 0.0 for motor_id in motor_ids}

        neg_index = +1
        cycle_time = time.time()

        for t in loop:
            # Read motor feedback for all motors
            for motor_id, motor in motors.items():
                if motor.read_feedback(timeout=0.001):
                    current_positions_rad[motor_id] = motor.position
                    current_angle_deg[motor_id] = np.degrees(current_positions_rad[motor_id])

            # Initial settling period (1 second)
            if t < 1.0:
                for motor_id, motor in motors.items():
                    motor.send_mit_command(
                        position=0.0,
                        velocity=0.0,
                        kp=KP,
                        kd=KD,
                        torque=0.0
                    )
                    # ========== FIX: Small delay between motor commands ==========
                    # time.sleep(0.001)  # 1ms between commands
                last_update = t

            else:
                # Position stepping phase
                if t > (last_update):
                    
                    # Determine target angle based on cycle direction
                    if cycle % 2 == 0:
                        target_angle_deg = np.array([
                            angles[i][min(angle_idx[i], len(angles[i]) - 1)]
                            for i in range(len(src))
                        ])
                        neg_index = +1
                    else:
                        target_angle_deg = np.array([
                            dest[i] - angles[i][min(angle_idx[i], len(angles[i]) - 1)]
                            for i in range(len(src))
                        ])
                        neg_index = -1
                    
                    # Convert to radians
                    target_angle_rad = np.radians(target_angle_deg)

                    # Send commands to all motors
                    for motor_id, motor in motors.items():
                        i = motor_id_to_index[motor_id]                       
                        if angle_idx[i] < len(angles[i]):
                            motor.send_mit_command(
                                position=target_angle_rad[i],
                                velocity=0.0*neg_index,
                                kp=KP,
                                kd=KD,
                                torque= 3.0 if neg_index > 0 else 0.0
                            )
                            # ========== FIX: Small delay between motor commands ==========
                            # time.sleep(0.001)  # 1ms between commands
                    
                    last_update = t
                    
                    # Calculate error (per motor)
                    current_angle_deg_arr = np.array([
                        current_angle_deg[motor_id] for motor_id in motor_ids
                    ])
                    error_deg = current_angle_deg_arr - target_angle_deg

                    # Print status per motor
                    for i in range(len(src)):
                        print(
                            f"t={t:.2f}s | M{motor_ids[i]} | "
                            f"Target: {target_angle_deg[i]:6.1f}° | "
                            f"Actual: {current_angle_deg[motor_ids[i]]:6.1f}° | "
                            f"Error: {error_deg[i]:+6.1f}° | "
                            f"Cycle: {cycle} | "
                            f"Step: {angle_idx[i]+1}/{len(angles[i])}"
                        )

                    # Advance to next angle (per motor)
                    for i in range(len(src)):
                        if angle_idx[i] < len(angles[i]) - 1:
                            # if abs(error_deg[i]) < err_threshold:
                            angle_idx[i] += 1
                    
                    # Check if all motors completed their trajectories
                    if all(angle_idx[i] == len(angles[i]) - 1 for i in range(len(src))):
                        cycle += 1
                        print(f"\n{'='*60}")
                        print(f"CYCLE {cycle} COMPLETED!, Time Taken: {time.time() - cycle_time:.2f}s")
                        print(f"{'='*60}\n")

                        time.sleep(3)
                        cycle_time = time.time()

                        # Reset all motors for next cycle
                        angle_idx[:] = 0

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
        
        for motor_id, motor in motors.items():
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
            time.sleep(0.001)
            print(f"  Motor ID {motor_id} stopped")
        time.sleep(0.5)
        
        for motor_id, motor in motors.items():
            motor.disable()
            # Don't close individual motor bus instances
            print(f"  Motor ID {motor_id} disabled")
        
        # ========== FIX: Close the shared bus only once ==========
        shared_bus.shutdown()
        print("Shared CAN bus closed")
        print("\nTest complete!")


if __name__ == '__main__':
    # Configuration
    MOTOR_IDS = [1, 2] 
    CAN_INTERFACE = 'can0'
    
    print("="*70)
    print("  AK60-6 V3.0 Position Step Test - Multiple Motors")
    print("  Tests smooth position control with error tracking")
    print("="*70)
    
    # Run the test
    position_step_test(motor_ids=MOTOR_IDS, can_interface=CAN_INTERFACE)


# #!/usr/bin/env python3
# """
# AK60-6 V3.0 Position Step Test
# Performs smooth position stepping with error tracking
# Based on MIT mode position control
# """

# import numpy as np
# import time
# import sys

# # Import your AK60 V3.0 control library
# from ak60_v3_control import AK60V3Motor

# import math

# def ease_in_cubic(x: float) -> float:
#     return 1 - math.pow(1 - x, 2.5)

# def map_range(x, in_min, in_max, out_min, out_max):
#     return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


# class SoftRealtimeLoop:
#     """
#     Simple soft real-time loop implementation
#     Maintains consistent loop timing with reporting
#     """
#     def __init__(self, dt=0.02, report=True, fade=0):
#         self.dt = dt
#         self.report = report
#         self.fade = fade
#         self.start_time = None
#         self.iteration = 0
        
#     def __enter__(self):
#         self.start_time = time.time()
#         return self
        
#     def __exit__(self, *args):
#         if self.report:
#             elapsed = time.time() - self.start_time
#             print(f"\nLoop Statistics:")
#             print(f"  Total time: {elapsed:.2f}s")
#             print(f"  Iterations: {self.iteration}")
#             print(f"  Average rate: {self.iteration/elapsed:.1f}Hz")
    
#     def __iter__(self):
#         return self
    
#     def __next__(self):
#         if self.start_time is None:
#             self.start_time = time.time()
        
#         current_time = time.time() - self.start_time
        
#         # Calculate when next iteration should occur
#         target_time = (self.iteration + 1) * self.dt
#         sleep_time = target_time - current_time
        
#         if sleep_time > 0:
#             #time.sleep(sleep_time)
#             #print(sleep_time)
#             pass
#         elif sleep_time < -self.dt and self.report:
#             print(f"Warning: Loop running {-sleep_time*1000:.1f}ms behind")
        
#         self.iteration += 1
#         return time.time() - self.start_time
        

# def position_step_test(motor_ids=[1, 2], can_interface='can0'):
#     """
#     Position step test for AK60-6 V3.0
    
#     Args:
#         motor_id: Motor CAN ID (default: 3)
#         can_interface: CAN interface name (default: 'can0')
#     """
    
#     # Initialize motor
#     print(f"Initializing AK60-6 V3.0 motor (ID={motor_ids})...")
#     motors = {}
#     for motor_id in motor_ids:
#         motors[motor_id] = AK60V3Motor(motor_id=motor_id, can_interface=can_interface)
#         print(f"  Motor ID {motor_id} initialized")
    
#     # After initializing motors, create an index mapping
#     motor_id_to_index = {motor_id: i for i, motor_id in enumerate(motor_ids)}

#     try:
#         # Setup phase
#         print("\n=== SETUP PHASE ===")
#         print("Enabling motors and setting zero positions...")
#         for motor_id, motor in motors.items():
#             motor.enable()
#             print(f"  Motor ID {motor_id} enabled")
#         time.sleep(0.5)
        
#         for motor_id, motor in motors.items():
#             motor.set_zero_position(permanent=False)
#             print(f"  Motor ID {motor_id} zero position set")
#         print("Waiting 3 seconds...")
#         time.sleep(3)
        
#         # Impedance gains for position control
#         # AK60-6: KP range 0-500, KD range 0-5
#         KP = 480.0  # Position stiffness (adjust based on your load)
#         KD = 2   # Damping (adjust based on your load)
        
#         print(f"Control gains: KP={KP}, KD={KD}")
#         print("\n=== STARTING POSITION STEP TEST ===")
#         print("Press Ctrl+C to stop\n")
        
#         # -------- Test parameters --------
#         src  = np.array([0, 0], dtype=float)
#         dest = np.array([20, 20], dtype=float)

#         # -------- Per-motor mapping & easing --------
#         mapped_value = map_range(np.abs(dest - src), 0, 360, 0, 1)
#         eased_value  = np.array([ease_in_cubic(x) for x in mapped_value])

#         # -------- Steps per motor --------
#         steps_per_motor = map_range(eased_value, 0, 1, 0, 800).astype(int)

#         # Safety: ensure at least 1 step
#         steps_per_motor = np.maximum(steps_per_motor, 1)

#         # -------- Per-motor trajectories --------
#         angles = []

#         for i in range(len(src)):
#             motor_angles = np.linspace(src[i], dest[i], steps_per_motor[i])
#             angles.append(motor_angles)

#         interval_between_angles = 1/1000 				# 1us
#         err_threshold = 2.0
    	  	
#         # Loop timing
#         loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0)  # 50Hz control loop
        
#         last_update = 0
#         angle_idx = np.zeros(len(src), dtype=int)

#         cycle = 0
#         current_positions_rad = {motor_id: 0.0 for motor_id in motor_ids}
#         current_angle_deg = {motor_id: 0.0 for motor_id in motor_ids}

#         neg_index = +1
    	
#         cycle_time = time.time()

#         for t in loop:
#             # Read motor feedback for all motors
#             for motor_id, motor in motors.items():
#                 if motor.read_feedback(timeout=0.001):
#                     current_positions_rad[motor_id] = motor.position
#                     current_angle_deg[motor_id] = np.degrees(current_positions_rad[motor_id])

#             # Initial settling period (1 second)
#             if t < 1.0:
#                 # Command zero position with impedance control
#                 for motor_id, motor in motors.items():
#                     motor.send_mit_command(
#                         position=0.0,
#                         velocity=0.0,
#                         kp=KP,
#                         kd=KD,
#                         torque=0.0
#                     )
#                 last_update = t

#             else:
#                 # Position stepping phase
#                 # print("t=", t, "(last_update + interval_between_angles) = ", (last_update + interval_between_angles))
#                 if t > (last_update + interval_between_angles):
                    
#                     # Determine target angle based on cycle direction
#                     if cycle % 2 == 0:
#                         # Even cycles: forward (0° → angle°)
#                         # target_angle_deg = src + angles[angle_idx]
#                         target_angle_deg = np.array([
#                             angles[i][min(angle_idx[i], len(angles[i]) - 1)]
#                             for i in range(len(src))
#                         ])
#                         neg_index = +1
#                     else:
#                         # Odd cycles: reverse (angle° → 0°)
#                         # target_angle_deg = dest - angles[angle_idx]
#                         target_angle_deg = np.array([
#                             dest[i] - angles[i][min(angle_idx[i], len(angles[i]) - 1)]
#                             for i in range(len(src))
#                         ])

#                         neg_index = -1
                    
#                     # Convert to radians
#                     target_angle_rad = np.radians(target_angle_deg)

#                     for motor_id, motor in motors.items():
#                         i = motor_id_to_index[motor_id]                       
#                         # Send position command via MIT mode
#                         if angle_idx[i] < len(angles[i]):
#                             motor.send_mit_command(
#                                 position=target_angle_rad[i],
#                                 velocity=0.0*neg_index, # 25
#                                 kp=KP,
#                                 kd=KD,
#                                 torque=0.0*neg_index # 5
#                             )
                    
#                     last_update = t
                    
#                     # Calculate error (per motor)
#                     current_angle_deg_arr = np.array([
#                         current_angle_deg[motor_id] for motor_id in motor_ids
#                     ])
#                     error_deg = current_angle_deg_arr - target_angle_deg

#                     # Print status per motor
#                     for i in range(len(src)):
#                         print(
#                             f"t={t:.2f}s | M{motor_ids[i]} | "  # Fixed: Show actual motor ID
#                             f"Target: {target_angle_deg[i]:6.1f}° | "
#                             f"Actual: {current_angle_deg[motor_ids[i]]:6.1f}° | "  # Fixed: Use motor_ids[i]
#                             f"Error: {error_deg[i]:+6.1f}° | "
#                             f"Cycle: {cycle} | "
#                             f"Step: {angle_idx[i]+1}/{len(angles[i])}"  # Fixed: Use angle_idx[i]
#                         )

#                     # Advance to next angle (per motor)
#                     for i in range(len(src)):
#                         if angle_idx[i] < len(angles[i]) - 1:
#                             if abs(error_deg[i]) < err_threshold:
#                                 angle_idx[i] += 1
                    
#                     # Check if all motors completed their trajectories
#                     if all(angle_idx[i] == len(angles[i]) - 1 for i in range(len(src))):
#                         cycle += 1
#                         print(f"\n{'='*60}")
#                         print(f"CYCLE {cycle} COMPLETED!, Time Taken: {time.time() - cycle_time:.2f}s")
#                         print(f"{'='*60}\n")

#                         time.sleep(3)
#                         cycle_time = time.time()

#                         # Reset all motors for next cycle
#                         angle_idx[:] = 0
            
#             # Small delay for loop timing
#             #time.sleep(0.00001)
#     except KeyboardInterrupt:
#         print("\n\nTest interrupted by user")
    
#     except Exception as e:
#         print(f"\nError occurred: {e}")
#         import traceback
#         traceback.print_exc()
    
#     finally:
#         # Safe shutdown
#         print("\n=== SHUTDOWN ===")
#         print("Stopping motors...")
        
#         # Send zero torque command with high damping to stop smoothly
#         for motor_id, motor in motors.items():
#             motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
#             print(f"  Motor ID {motor_id} stopped")
#         time.sleep(0.5)
        
#         for motor_id, motor in motors.items():
#             motor.disable()
#             motor.close()
#             print(f"  Motor ID {motor_id} disabled and closed")
#         print("All motors stopped and CAN bus closed")
#         print("\nTest complete!")


# if __name__ == '__main__':
#     # Configuration
#     # MOTOR_ID = 2            # Your motor's CAN ID
#     MOTOR_IDS = [1, 2] 
#     CAN_INTERFACE = 'can0'  # Your CAN interface
    
#     print("="*70)
#     print("  AK60-6 V3.0 Position Step Test")
#     print("  Tests smooth position control with error tracking")
#     print("="*70)
    
#     # Run the test
#     position_step_test(motor_ids=MOTOR_IDS, can_interface=CAN_INTERFACE)
        
        
