#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/motor_controller_receiver_multiprocess.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Same receiver with the socket listener in a separate process (own CPU
# core).
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Multi-Motor Controller - TRUE MULTIPROCESSING
Socket listener runs in separate process with its own CPU core
Motor control runs in main process
"""

import numpy as np
import time
import socket
import pickle
import multiprocessing as mp

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
            time.sleep(sleeptime)
        elif sleeptime < -self.dt and self.report:
            print(f"Warning: Loop running {-sleeptime*1000:.1f}ms behind")
        self.iteration += 1
        return time.time() - self.starttime


def socket_listener_process(dest_queue, port=50000, host='127.0.0.1'):
    """
    ✅ SEPARATE PROCESS - Runs on its own CPU core
    Receives numpy arrays via socket and puts in shared queue
    """
    print(f"[Socket Process PID {mp.current_process().pid}] Starting...")
    
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((host, port))
        server.listen(5)
        print(f"✅ Socket server on {host}:{port} (separate process)")
        
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


def motor_can(motor_ids=[1,2,3], can_interface='can0', dest_queue=None):
    """
    ✅ MAIN PROCESS - Runs on dedicated CPU core
    50Hz realtime motor control loop
    """
    print(f"[Motor Process PID {mp.current_process().pid}] Starting...")
    print("STARTING EXTERNAL QUEUE POSITION CONTROL (50Hz)")
    print("Run 'python motor_controller_sender.py' in another terminal!")
    print("Press Ctrl+C to stop")

    num_motors = len(motor_ids)
    current_positions_deg = np.zeros(num_motors)
    current_dest_deg = np.zeros(num_motors)
    move_active = False
    movetime = np.zeros(num_motors)
    src_deg = np.zeros(num_motors)
    dest_deg = np.zeros(num_motors)
    movestarttime = None

    loop = SoftRealtimeLoop(dt=0.02, report=True)

    try:
        with loop:
            for t in loop:
                new_dest = None
                if dest_queue:
                    try:
                        new_dest = dest_queue.get_nowait()
                    except:
                        pass

                if new_dest is not None and len(new_dest) == num_motors:
                    if not np.array_equal(new_dest, current_dest_deg):
                        src_deg = current_positions_deg.copy()
                        dest_deg = new_dest.copy()
                        
                        angle_diff = np.abs(dest_deg - src_deg)
                        for i in range(num_motors):
                            if angle_diff[i] < 5.0:
                                movetime[i] = 0.08
                            else:
                                movetime[i] = max(0.08, 1.0 * angle_diff[i] / 360.0)
                        
                        movestarttime = time.time()
                        move_active = True
                        print(f"[Motor Process] 🎯 NEW MOVE: {src_deg.tolist()}° → {dest_deg.tolist()}°")

                if move_active and movestarttime:
                    elapsed = time.time() - movestarttime
                    all_finished = True
                    for i in range(num_motors):
                        Ti = movetime[i]
                        if elapsed >= Ti:
                            current_positions_deg[i] = dest_deg[i]
                        else:
                            s = np.clip(elapsed / Ti, 0, 1)
                            s = 3*s*s - 2*s*s*s
                            current_positions_deg[i] = src_deg[i] + (dest_deg[i] - src_deg[i]) * s
                            all_finished = False
                    
                    if all_finished:
                        current_dest_deg = dest_deg.copy()
                        move_active = False
                        print(f"[Motor Process] ✅ REACHED: {current_dest_deg.tolist()}°")

    except KeyboardInterrupt:
        print("\n[Motor Process] Interrupted by user")
    except Exception as e:
        print(f"[Motor Process] Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("[Motor Process] SHUTDOWN")


if __name__ == '__main__':
    # ✅ Required for Windows multiprocessing
    # mp.set_start_method('spawn', force=True)
    try:
        mp.set_start_method('spawn')
    except RuntimeError:
        pass  # Already set, that's fine
    
    motor_ids = [1, 2, 3]
    can_interface = 'can0'
    
    print("="*70)
    print("TRUE MULTIPROCESSING Multi-Motor Controller (50Hz)")
    print("Socket listener and motor control on separate CPU cores")
    print("="*70)
    
    # ✅ CROSS-PROCESS QUEUE (works on Windows, Linux, macOS)
    dest_queue = mp.Queue()
    
    # ✅ START SOCKET LISTENER AS SEPARATE PROCESS
    socket_process = mp.Process(
        target=socket_listener_process,
        args=(dest_queue, 50000, '127.0.0.1'),
        daemon=True  # Dies when main process exits
    )
    socket_process.start()
    
    print(f"✅ Socket process started (PID {socket_process.pid})")
    time.sleep(1.0)  # Give socket time to bind
    
    try:
        # ✅ MOTOR CONTROL RUNS IN MAIN PROCESS
        motor_can(motor_ids=motor_ids, can_interface=can_interface, dest_queue=dest_queue)
    finally:
        print("\nCleaning up processes...")
        socket_process.terminate()
        socket_process.join(timeout=2)
        if socket_process.is_alive():
            socket_process.kill()
        print("All processes terminated")
