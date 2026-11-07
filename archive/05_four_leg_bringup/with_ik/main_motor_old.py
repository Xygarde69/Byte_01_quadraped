#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/with_ik/main_motor_old.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Previous revision of main_motor.py.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Multi-Motor Controller
Socket listener runs in separate process with its own CPU core
Motor control runs in main process
"""

import queue

import numpy as np
import time
import sys
import can
import math
import multiprocessing as mp
import socket
import pickle
from ak60_v3_control import AK60V3Motor
import json
import platform


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


def socket_listener_process(dest_queue, port=50000, host='127.0.0.1'):
    """
    SEPARATE PROCESS - Runs on its own CPU core
    Receives numpy arrays via socket and puts in shared queue
    """
    print(f"[Socket Process] Process ID {mp.current_process().pid}] Starting...")
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((host, port))
        server.listen(5)
        print(f"[Socket Process] Socket server on {host}:{port} (separate process)")
        
        while True:
            try:
                conn, addr = server.accept()
                
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                
                if data:
                    array = pickle.loads(data)
                    dest_queue.put(array)
                    print(f"[Socket Process] Received: {array.tolist()}°")
                
                conn.close()
                
            except Exception as e:
                print(f"[Socket Process] Error: {e}")
                
    except Exception as e:
        print(f"[Socket Process] Server error: {e}")
    finally:
        server.close()
        print("[Socket Process] Shutdown")


def motor_can(motor_ids=[1], dest_queue=None):
    """
    MAIN PROCESS - Runs on dedicated CPU core
    50Hz realtime motor control loop
    """
    print(f"[Motor Process PID {mp.current_process().pid}] Starting...")  

    # Load motor config from JSON file
    config_file = 'motor_config.json'
    print(f"Loading motor configuration from {config_file}...")
    with open(config_file, 'r') as f:
        config = json.load(f)

    can_interfaces = set()                      # Collect unique CAN interfaces from motor config

    for motor_id in motor_ids:
        if str(motor_id) not in config['motors']:
            print(f"Error: Motor ID {motor_id} not found in config file")
            sys.exit(1)
        can_interfaces.add(config['motors'][str(motor_id)]['can'])
    
    print(f"Initializing shared CAN bus on {can_interfaces}...")
    shared_bus = [None]*len(can_interfaces)     # Support multiple interfaces if needed
    bus_can_map = [None]*len(can_interfaces)    # Map can interface to bus index
    
    for i, can_name in enumerate(can_interfaces):
        print(f" - {can_name}")
        shared_bus[i] = can.interface.Bus(channel=can_name, bustype='socketcan', bitrate=1000000)
        bus_can_map[i] = can_name
        print(f"Shared CAN {can_name} bus initialized. ✓")
    
    print(f"Initializing {len(motor_ids)} motors : {motor_ids}...")
    motors = {}
    for motor_id in motor_ids:
        can_interface = config['motors'][str(motor_id)]['can']
        motors[motor_id] = AK60V3Motor(
            motor_id=motor_id,
            can_interface=can_interface,
            bus=shared_bus[bus_can_map.index(can_interface)]
        )
        print(f"Motor ID: {motor_id} initialized on CAN {can_interface} ✓ Can ID: {bus_can_map.index(can_interface)}")
    
    try:
        print("="*20)
        print("SETUP PHASE")

        print("Enabling motors and setting zero positions...")
        for motor_id, motor in motors.items():
            motor.enable()
            print(f"Motor ID {motor_id} enabled")
        time.sleep(0.5)
        
        num_motors = len(motor_ids)
        
        KP = np.zeros(num_motors)
        KD = np.zeros(num_motors)

        for i, motor_id in enumerate(motors.keys()):
            # READ KP,KD from motor config
            KP[i] = config['motors'][str(motor_id)]['kp']
            KD[i] = config['motors'][str(motor_id)]['kd']

        print(f"Control gains: KP={KP}, KD={KD}")

        for motor_id, motor in motors.items():
            motor.set_zero_position(permanent=False)
            print(f"Motor ID {motor_id} zero position set")
        
        print("Waiting 3 seconds...")
        time.sleep(3)


        print("Setup complete. ✓")
        print("="*20)

        print("Starting main control loop...")
        print("Press Ctrl+C to stop")
        
        
        current_positions_deg = np.zeros(num_motors)
        current_dest_deg = np.zeros(num_motors)
        move_active = False
        src_deg = np.zeros(num_motors)
        dest_deg = np.zeros(num_motors)
        
        loop = SoftRealtimeLoop(dt=0.1, report=True, fade=0)  # 50Hz
      
        with loop:
            for t in loop: 

                # Read current positions
                for i, motor_id in enumerate(motor_ids):
                    motor = motors[motor_id]
                    if motor.read_feedback(timeout=0.010):
                        current_positions_deg[i] = math.degrees(motor.position)

                # Initial 1 s settle at zero
                if t < 1.0: 
                    for i, motor_id in enumerate(motor_ids):
                        motors[motor_id].send_mit_command(
                            position=0.0,
                            velocity=0.0,
                            kp=KP[i],
                            kd=KD[i],
                            torque=0.0
                        ) 
                    continue

                # Check for new destination
                new_dest = None
                if dest_queue:
                    try:
                        new_dest = dest_queue.get_nowait()
                        result = []
                        for i in range(0, len(new_dest), 3):
                            chunk = list(new_dest[i:i+3])
                            result += chunk + chunk  # repeat each chunk twice
                        new_dest = np.array(result)
                        print(f"[Motor Process] {len(new_dest)} New destinations from queue: {new_dest.tolist()}°")
                    # except queue.Empty:
                    except:
                        # print(f"[Motor Process] Queue is empty")
                        pass  # Queue empty
                
                if new_dest is not None and len(new_dest) == num_motors:
                    if not np.array_equal(new_dest, current_dest_deg):
                        # New move: current pos -> new dest
                        src_deg = current_positions_deg.copy()
                        dest_deg = new_dest.copy()
                        
                        move_active = True
                        print(f"[Motor Process] NEW MOVE: {src_deg.tolist()}° -> {dest_deg.tolist()}°")
                else:
                    print("No new destination or invalid length, holding current position.")

                # Execute active move
                all_finished = True
                if move_active:
                    targets_deg = np.zeros(num_motors)

                    for i, motor_id in enumerate(motor_ids):
                        # Apply gear ratio as specified in config
                        targets_deg[i] = dest_deg[i] * config['motors'][str(motor_id)]['gear_ratio'] 

                        # Apply flipping if specified in config
                        targets_deg[i] = -targets_deg[i] if config['motors'][str(motor_id)]['flipped'] else targets_deg[i] 
                        
                        motors[motor_id].send_mit_command(
                            position=np.radians(targets_deg[i]), velocity=0.0, kp=KP[i], kd=KD[i], torque=0.0
                        )

                        if t % 1.0 < 0.02: 
                            print(f"SENT t = {t:.1f} Motor {motor_id}: target={targets_deg[i]:.2f}°")

                        if abs(current_positions_deg[i] - targets_deg[i]) > 0.5:  # 0.5° tolerance
                            all_finished = False
                                        
                    # Status every 1s
                    if t % 1.0 < 1.02: 
                        for i, motor_id in enumerate(motor_ids):
                            err = abs(current_positions_deg[i] - targets_deg[i])
                            
                            print(f"Status at t={t:.1f} M{motor_id} Target:{targets_deg[i]:6.1f}° "
                                  f"Actual:{current_positions_deg[i]:6.1f}° Err:{err:6.1f}°")
                    
                    if all_finished:
                        current_dest_deg = dest_deg.copy()
                        move_active = False
                        print("✓"*20)
                        print(f"[Motor Process] ✓ Reached {current_dest_deg.tolist()}° "
                                f"(actual: {current_positions_deg.tolist()}°)")
                        
                        # Hold position with higher torque
                        for i, motor_id in enumerate(motor_ids):
                            motors[motor_id].send_mit_command(
                                position=np.radians(current_dest_deg[i]), velocity=0.0, 
                                kp=min(450, KP[i]*1.5), kd=KD[i], torque=0.0  # Passive hold
                            )
                
                # Default hold if no move
                elif not move_active: 
                    if t % 5.0 < 0.02: 
                        print("Hold at: ", current_dest_deg)
                    for i, motor_id in enumerate(motor_ids): 
                        motors[motor_id].send_mit_command(
                            position=np.radians(current_dest_deg[i]), velocity=0.0, 
                            kp=min(450, KP[i]*1.5), kd=KD[i], torque=0.0
                        )
    
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

        for bus in shared_bus:
            if bus is not None:
                bus.shutdown()
        print("[Motor Process] All motors stopped and disabled")
        print("[Motor Process] All CAN buses shutdown")
        print("[Motor Process] Complete!")   

if __name__ == '__main__':
    # Required for Windows multiprocessing
    # mp.set_start_method('spawn', force=True)

    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass  # Already set, that's fine

    # motor_ids = [1,2,3,4,5,6,7,8,9,10,11,12]  # Example motor IDs for 4 legs with 3 motors each 
    # motor_ids = [4,10]

    motor_ids = [] 

    #motor_ids.extend([1,2,3])  # front left leg, can0
    #motor_ids.extend([4,5,6])  # back left leg, can0
    motor_ids.extend([7,8,9])  # front right leg, can1
    #motor_ids.extend([10,11,12])  # back right leg, can1

    # 1,2,3 - front left leg
    # 4,5,6 - back left leg
    # 7,8,9 - front right leg
    # 10,11,12 - back right leg

    print("="*70)
    print("AK60-6 V3.0 Position Test - Multiple Motors")
    print("="*70)

    # CROSS-PROCESS QUEUE (works on Windows, Linux, macOS)
    dest_queue = mp.Queue()

    # START SOCKET LISTENER AS SEPARATE PROCESS
    socket_process = mp.Process(
        target=socket_listener_process,
        args=(dest_queue, 50000, '127.0.0.1'),
        daemon=True  # Dies when main process exits
    )
    socket_process.start()

    print(f"Socket process started (PID {socket_process.pid})")
    time.sleep(1.0)  # Give socket time to bind

    try:
        # MOTOR CONTROL RUNS IN MAIN PROCESS
        motor_can(motor_ids=motor_ids, dest_queue=dest_queue)
    finally:
        print("\nCleaning up processes...")
        socket_process.terminate()
        socket_process.join(timeout=2)
        if socket_process.is_alive():
            socket_process.kill()
        print("All processes terminated")