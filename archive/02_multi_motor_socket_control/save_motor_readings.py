# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/save_motor_readings.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Records motor positions to motor_positions.txt using a
# SoftRealtimeLoop (motors 1-3); the record half of the record/replay
# pair with replay_saved_readings.py.
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import time
import sys
import can
import math
import multiprocessing as mp
import socket
import pickle
from ak60_v3_control import AK60V3Motor  # Your library [file:2]
import json

motor_ids = [1,2,3]
num_motors = len(motor_ids)

class SoftRealtimeLoop:
    def __init__(self, dt=0.01, report=True, fade=0):
        self.dt = dt
        self.report = report
        self.fade = fade
        self.starttime = None
        self.iteration = 0
   
    def __enter__(self):
        self.starttime = time.time()
        return self

    def __exit__(self, *args):
        if self.report and self.starttime is not None:
            elapsed = time.time() - self.starttime
            print(f"Statistics: Total time {elapsed:.2f}s")
            print(f"Iterations: {self.iteration}")
            print(f"Average rate: {self.iteration/elapsed:.1f}Hz")

    def __iter__(self):
        return self

    def __next__(self):
        if self.starttime is None:
            self.starttime = time.time()
        currenttime = time.time() - self.starttime
        targettime = (self.iteration + 1) * self.dt
        sleeptime = targettime - currenttime
        if sleeptime > 0:
            # time.sleep(sleeptime)
            pass
        elif sleeptime < -self.dt and self.report:
            print(f"Warning: Loop running {-sleeptime*1000:.1f}ms behind")
        self.iteration += 1
        return time.time() - self.starttime


def read_save(can_interface='can0'):
    print(f"[Motor Process PID {mp.current_process().pid}] Starting...")
    print("STARTING EXTERNAL QUEUE POSITION CONTROL (50Hz)")
    print("Run 'python motor_controller_sender.py' in another terminal!")
    print("Press Ctrl+C to stop")
    
    print(f"Initializing shared CAN bus on {can_interface}...")
    shared_bus = can.interface.Bus(channel=can_interface, bustype='socketcan', bitrate=1000000)
    print("Shared CAN bus initialized. ✓")
    
    print(f"Initializing {len(motor_ids)} motors ...")
    motors = {}
    for motor_id in motor_ids:
        motors[motor_id] = AK60V3Motor(
            motor_id=motor_id,
            can_interface=can_interface,
            bus=shared_bus
        )
        print(f"Motor ID: {motor_id} initialized.")
    
    try:
        print("SETUP PHASE")
        print("Enabling motors and setting zero positions...")
        for motor_id, motor in motors.items():
            motor.enable()
            print(f"Motor ID {motor_id} enabled")
        time.sleep(0.5)
                     
        for motor_id, motor in motors.items():
            motor.set_zero_position(permanent=False)
            print(f"Motor ID {motor_id} zero position set")
        
        print("Waiting 3 seconds...")
        time.sleep(3)

        current_positions_deg = np.zeros(num_motors)

        loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0)

        for motor_id, motor in motors.items():
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
            time.sleep(0.01)
        
        start_key = input("Press Enter to start reading and saving motor positions...")

        with loop:
            for t in loop: 
                # Read current positions
                feedback_err = False
                for i, motor_id in enumerate(motor_ids):
                    motor = motors[motor_id]
                    if motors[motor_id].read_feedback(timeout=0.010):
                        current_positions_deg[i] = math.degrees(motor.position)
                    else:
                        feedback_err = True
                
                if not feedback_err:
                    # log the positions of motor in text file
                    string_to_save = ""
                    for i, motor_id in enumerate(motor_ids):
                        string_to_save += f"{motor_id}:{current_positions_deg[i]:.2f},"
                    with open("motor_positions.txt", "a") as f:
                        f.write(f"{string_to_save}\n")
                        print("Saved: {string_to_save}")

    except KeyboardInterrupt:
        print("\n[Motor Process] Interrupted by user")
    except Exception as e:
        print(f"[Motor Process] Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[Motor Process] SHUTDOWN")
        print("[Motor Process] Stopping motors...")
        for motor_id, motor in motors.items():
            motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
            time.sleep(0.01)
            print(f"[Motor Process] Motor ID {motor_id} stopped")
        time.sleep(0.5)
        for motor_id, motor in motors.items():
            motor.disable()
            print(f"[Motor Process] Motor ID {motor_id} disabled")
        shared_bus.shutdown()
        print("[Motor Process] Shared CAN bus closed")
        print("[Motor Process] Complete!")

if __name__ == '__main__':
    can_interface = 'can0'
    
    print("="*70)
    print("AK60-6 V3.0 Position Test - Multiple Motors, S-curve")
    print("MODIFIED: External multiprocessing.Queue() destination control")
    print("="*70)


    read_save(can_interface=can_interface)