#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/main_json_motor_controller_easing_multiprocess.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Multi-motor controller, true multiprocessing: socket listener in its
# own process, motor control in the main process, settings from
# motor_config.json, S-curve easing.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Multi-Motor Controller - TRUE MULTIPROCESSING
Socket listener runs in separate process with its own CPU core
Motor control runs in main process
"""

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


motor_ids = [10,11,12]
num_motors = len(motor_ids)

error_dict = {
    0: "No Error",
    1: "Motor Over Temperature",
    2: "Over Current",
    3:"Over Voltage",
    4:"Under Voltage",
    5:"Encoder Error",
    6:"MOSFET Over Temperature",
    7:"Motor Lock Up",
}



def scurve(s):
    """Smooth S-curve from 0 to 1 for s in [0,1]"""
    s = max(0.0, min(1.0, s))
    return 3.0*s*s - 2.0*s*s*s


def scurve_derivative(s):
    """
    Derivative of S-curve: velocity profile
    For scurve(s) = 3s² - 2s³
    Derivative: d/ds = 6s - 6s²
    """
    s = max(0.0, min(1.0, s))
    return 6.0*s - 6.0*s*s


def scurve_second_derivative(s):
    # scurve(s) = 3 s^2 - 2 s^3
    # d/ds = 6s - 6s^2
    # d²/ds² = 6 - 12s
    s = max(0.0, min(1.0, s))
    return 6.0 - 12.0*s


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


def json_reader_process(dest_queue):
    """
    ✅ SEPARATE PROCESS - Runs on its own CPU core
    Reads angles + KP/KD from JSON, sends complete config via queue
    """
    print(f"[JSON Process PID {mp.current_process().pid}] Starting...")

    # Initialize previous values for change detection
    prev_angles = None
    prev_kp = None
    prev_kd = None
    
    try:       
        while True:
            try:
                # ✅ SINGLE PROCESS reads ALL config data atomically
                with open('motor_config.json', 'r') as f:
                    config = json.load(f)
                
                # Extract complete config for all motors
                config_data = {} 
                for i, motor_id in enumerate(motor_ids): 
                    config_data[i] = {
                        'angle': float(config['motors'][str(motor_id)]['angle']),
                        'kp': float(config['motors'][str(motor_id)]['kp']),
                        'kd': float(config['motors'][str(motor_id)]['kd'])
                    }
                
                angle_array = np.array([config_data[i]['angle'] for i in range(0, num_motors)])
                kp_array = np.array([config_data[i]['kp'] for i in range(0, num_motors)])
                kd_array = np.array([config_data[i]['kd'] for i in range(0, num_motors)])

                # ✅ CHANGE DETECTION - Print only when values changed
                changed = False
                if prev_angles is None or not np.array_equal(angle_array, prev_angles):
                    changed = True
                if prev_kp is None or not np.array_equal(kp_array, prev_kp):
                    changed = True
                if prev_kd is None or not np.array_equal(kd_array, prev_kd):
                    changed = True
                
                # Send complete config packet
                packet = {
                    'angles': angle_array,
                    'kp': kp_array,
                    'kd': kd_array,
                    'timestamp': time.time()
                }
                
                dest_queue.put(packet)
  

                # Print ONLY on first read OR when values changed
                if changed:
                    print(f"\n\n\n[JSON CHANGED] angles={angle_array.tolist()}° KP={kp_array.tolist()} KD={kd_array.tolist()}")
                    prev_angles = angle_array.copy()
                    prev_kp = kp_array.copy()
                    prev_kd = kd_array.copy()      

                
                time.sleep(0.5)   
                
            except Exception as e:
                print(f"[JSON Process] Read error: {e}")
                time.sleep(0.1)  # Brief pause on error
                
    except KeyboardInterrupt:
        print("[JSON Process] Interrupted")
    finally: 
        print("[JSON Process] Shutdown")



def motor_can(can_interface='can1', dest_queue=None):
    """
    ✅ MAIN PROCESS - Runs on dedicated CPU core
    50Hz realtime motor control loop
    """
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

        
        # S-curve timing parameters
        BASEANGLE = 360.0
        BASETIME = 12.0
        MINTIME = 0.08
        SMALLANGLETHRESH = 5.0
        
        print("STARTING EXTERNAL QUEUE POSITION CONTROL")
        print("Press Ctrl+C to stop")
        
        
        current_positions_deg = np.zeros(num_motors)
        current_temp_deg = np.zeros(num_motors)

        current_dest_deg = np.zeros(num_motors)
        move_active = False
        movetime = np.zeros(num_motors)
        src_deg = np.zeros(num_motors)
        dest_deg = np.zeros(num_motors)
        movestarttime = None

        KP = np.zeros(num_motors)  # Will be updated from queue
        KD = np.zeros(num_motors)  # Will be updated from queue

        # m_load = np.full(num_motors, 3.2)    # kg
        m_load = np.array([1.2, 1.82, 0.15])    # kg
        r_load = np.array([0.11, 0.095, 0.108])    # m

        # r_load = np.full(num_motors, 0.235)  # m
        tau_ff_hold = 0.0  # Feedforward torque

        print("Waiting for config from JSON process...")
        
        loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0)

        hold_print = True
        last_print = time.time()
        last_print_2 = time.time()
      
        with loop:
            for t in loop: 

                # Read current positions
                for i, motor_id in enumerate(motor_ids):
                    motor = motors[motor_id]
                    if motor.read_feedback(timeout=0.010):
                        current_positions_deg[i] = math.degrees(motor.position)
                        current_temp_deg[i] = motor.temperature
                        err_status = motor.error
                        if err_status != 0:
                            print(f"\n\n\n\n\n\n[Motor Process] ⚠️ Motor ID {motor_id} Error: {error_dict.get(err_status, 'Unknown Error')} (Code {err_status})")

                # Initial 1 s settle at zero
                if t < 1.0: 
                    for i, motor_id in enumerate(motor_ids):
                        motors[motor_id].send_mit_command(
                            position=0.0,
                            velocity=0.0,
                            kp=150.0, 
                            kd=1.5,
                            torque=0.0
                        ) 
                    continue

                # Check for new destination
                new_config = None
                if dest_queue:
                    try:
                        new_config = dest_queue.get_nowait()
                    except:
                        pass  # Queue empty
                
                if new_config is not None:
                    # Update gains whenever new config arrives
                    KP = new_config['kp']
                    KD = new_config['kd']

                    if not np.array_equal(new_config['angles'], current_dest_deg):
                        # New move: current pos -> new dest
                        src_deg = current_positions_deg.copy()
                        dest_deg = new_config['angles'].copy()
                        current_dest_deg = dest_deg.copy()
                        # all_finished = False
                        
                        # Compute move times
                        angle_diff = np.abs(dest_deg - src_deg)
                        for i in range(num_motors):
                            if angle_diff[i] < SMALLANGLETHRESH:
                                movetime[i] = MINTIME
                            else:
                                movetime[i] = max(MINTIME, BASETIME * angle_diff[i] / BASEANGLE)
                        
                        movestarttime = time.time()
                        move_active = True
                        print(f"\n[Motor Process] NEW MOVE: {src_deg.tolist()}° -> {dest_deg.tolist()}° (times: {movetime.tolist()}s)")
                

                
                # Execute active move
                all_finished = True
                if move_active and movestarttime:
                    hold_print = True
                    elapsed = time.time() - movestarttime
                    # print("Elapsed:", elapsed, "movetime:", movetime, "movestarttime:", movestarttime, " | ", time.time())
                    targets_deg = np.zeros(num_motors)
                    targets_rad = np.zeros(num_motors)
                    targets_vel_rad = np.zeros(num_motors)
                    acc_rad = np.zeros(num_motors)
                    
                    
                    J_tot = m_load * r_load * r_load     # kg·m², shape (num_motors,)

                    for i, motor_id in enumerate(motor_ids):
                        Ti = movetime[i]
                        if elapsed >= Ti:
                            targets_deg[i] = dest_deg[i]
                        else:
                            eT = elapsed / Ti
                            s = scurve(eT)
                            ds_dt = scurve_derivative(eT) / Ti
                            d2s_dt2 = scurve_second_derivative(eT) / (Ti*Ti)
                            targets_deg[i] = src_deg[i] + (dest_deg[i] - src_deg[i]) * s
                            targets_vel_rad[i] = np.radians(ds_dt * (dest_deg[i] - src_deg[i]))
                            acc_rad[i] = np.radians((dest_deg[i] - src_deg[i])) * d2s_dt2

      
                        targets_rad[i] = np.radians(targets_deg[i]) 

                        tau_inertia = J_tot[i] * acc_rad[i]
                        tau_grav = m_load[i] * (9.81) * r_load[i] * np.sin(targets_rad[i])
                        tau_ff = tau_inertia + tau_grav

                        # print(f"Motor {motor_id}: Temp={current_temp_deg[i]:.1f}°C | TargetAngle={targets_rad[i]:.2f} | Acc={acc_rad[i]:.2f} rad/s² | Tau_inertia={tau_inertia:.3f} Nm | Tau_grav={tau_grav:.3f} Nm | Tau_ff={tau_ff:.3f} Nm")

                        motor = motors[motor_id]
                        motor.send_mit_command(
                            position=targets_rad[i], velocity=targets_vel_rad[i], kp=KP[i], kd=KD[i], torque=tau_ff
                        )

                        err_relative = abs(current_positions_deg[i] - targets_deg[i])
                        err_absolute = abs(current_positions_deg[i] - dest_deg[i])

                        if err_absolute > 0.5:  # 0.5° tolerance
                            all_finished = False

                    if time.time()>(last_print+0.2): # print every 0.1 second
                        last_print = time.time()
                        print(f"t {t:.1f} M {motor_ids}:  Temp={current_temp_deg[i]:.1f}°C | Target={np.round(targets_deg, 2)} | Actual={np.round(current_positions_deg, 2)} | TrackEr={np.round(np.abs(current_positions_deg - targets_deg), 2)} | FinalEr={np.round(np.abs(current_positions_deg - dest_deg), 2)} | KP={KP} KD={KD} Vel={np.round(np.degrees(targets_vel_rad), 1)}°/s")

                        # print(f"t {t:.1f} M {motor_ids}: Target={str(np.round(targets_deg, 2))} | Actual:{str(np.round(current_positions_deg, 2))} | TrackEr:{str(np.round(abs(current_positions_deg - targets_deg), 2))} | FinalEr:{str(np.round(abs(current_positions_deg - dest_deg), 2))} | KP={KP} KD={KD} Vel={math.degrees(targets_vel_rad)}°/s")
                                                            
                    if all_finished:
                        move_active = False
                        print(f"[Motor Process] ✅ Reached {current_dest_deg.tolist()}° (actual: {current_positions_deg.tolist()}°)  Temp={current_temp_deg[i]:.1f}°C") 
                        
                        # Use JSON gains if available, otherwise safe defaults
                        hold_kp = KP if np.any(KP > 0) else np.full(num_motors, 150.0)
                        hold_kd = KD if np.any(KD > 0) else np.full(num_motors, 1.5)
                        
                        for i, motor_id in enumerate(motor_ids):
                            theta_hold = np.radians(current_dest_deg[i])
                            tau_grav_hold = m_load[i] * 9.81 * r_load[i] * np.sin(theta_hold)
                            tau_ff_hold = tau_grav_hold   # inertia term is zero at steady state

                            motors[motor_id].send_mit_command(
                                position=theta_hold,
                                velocity=0.0,
                                kp=min(450, hold_kp[i]),
                                kd=hold_kd[i],
                                torque=0.0
                            )

                        print(f"\nHold at: {current_dest_deg.tolist()}° (KP={hold_kp.tolist()}, KD={hold_kd.tolist()}, Torque={tau_ff_hold:.2f} Nm), Temp={current_temp_deg[i]:.1f}°C")
                        
                
                # Default hold if no move
                elif not move_active: 
                    # Use JSON gains if available, otherwise safe defaults
                    hold_kp = KP if np.any(KP > 0) else np.full(num_motors, 150.0)
                    hold_kd = KD if np.any(KD > 0) else np.full(num_motors, 1.5)


                    for i, motor_id in enumerate(motor_ids):
                        theta_hold = np.radians(current_dest_deg[i])
                        tau_grav_hold = m_load[i] * 9.81 * r_load[i] * np.sin(theta_hold)
                        tau_ff_hold = tau_grav_hold   # inertia term is zero at steady state

                        motors[motor_id].send_mit_command(
                            position=theta_hold,
                            velocity=0.0,
                            kp=min(450, hold_kp[i]),
                            kd=hold_kd[i],
                            torque=0.0
                        )

                    if time.time()>(last_print_2+2.2): # print every 0.1 second
                        last_print_2 = time.time()
                        print(f"[Motor Process] (Actual: {current_positions_deg.tolist()}°) Temp={current_temp_deg.tolist()}°C")
                    
                    if hold_print: 
                        print(f"\nHold at: {current_dest_deg.tolist()}° (KP={hold_kp.tolist()}, KD={hold_kd.tolist()}, Torque={tau_ff_hold:.2f} Nm) Temp={current_temp_deg.tolist()}°C")
                        hold_print = False
    
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
    # ✅ Required for Windows multiprocessing
    # mp.set_start_method('spawn', force=True)

    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass  # Already set, that's fine

    # motor_ids = [1,2,3]
    can_interface = 'can1'
    
    print("="*70)
    print("AK60-6 V3.0 Position Test - Multiple Motors, S-curve")
    print("MODIFIED: External multiprocessing.Queue() destination control")
    print("="*70)

    # ✅ CROSS-PROCESS QUEUE (works on Windows, Linux, macOS)
    dest_queue = mp.Queue()

    # ✅ START SOCKET LISTENER AS SEPARATE PROCESS
    json_process = mp.Process(
        target=json_reader_process,
        args=(dest_queue,),
        daemon=True  # Dies when main process exits
    )
    json_process.start()

    print(f"✅ JSON process started (PID {json_process.pid})")
    time.sleep(1.0)  # Give process time to bind

    try:
        # ✅ MOTOR CONTROL RUNS IN MAIN PROCESS
        motor_can(can_interface=can_interface, dest_queue=dest_queue)
    finally:
        print("\nCleaning up processes...")
        json_process.terminate()
        json_process.join(timeout=2)
        if json_process.is_alive():
            json_process.kill()
        print("All processes terminated")
    
