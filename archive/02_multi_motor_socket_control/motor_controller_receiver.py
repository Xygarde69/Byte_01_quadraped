#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/motor_controller_receiver.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Socket receiver: gets np.array destinations from any sender over TCP
# and drives motors with a 50 Hz loop and S-curve motion.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Multi-Motor Controller - Socket Receiver
Receives np.array destinations from ANY external sender via TCP socket
50Hz realtime loop with S-curve motion
"""

import numpy as np
import time
import socket
import pickle
import threading
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


def socket_listener(dest_queue, port=50000, host='127.0.0.1'):
    """
    TCP socket server that receives numpy arrays and puts them in queue
    """
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    
    try:
        server.bind((host, port))
        server.listen(5)
        print(f"✅ Socket server listening on {host}:{port}")
        
        while True:
            try:
                conn, addr = server.accept()
                # print(f"Connection from {addr}")
                
                data = b''
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        break
                    data += chunk
                
                if data:
                    array = pickle.loads(data)
                    dest_queue.put(array)
                    # print(f"Received array: {array}")
                
                conn.close()
                
            except Exception as e:
                print(f"Socket error: {e}")
                
    except Exception as e:
        print(f"Server error: {e}")
    finally:
        server.close()


def motor_can(motor_ids=[1,2,3], can_interface='can0', dest_queue=None):
    """
    Continuous position control with external dest queue
    """
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
                        print(f"🎯 NEW MOVE: {src_deg.tolist()}° → {dest_deg.tolist()}°")
                        print(f"   Times: {movetime.tolist()[:2]}s...")

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
                        print(f"✅ REACHED: {current_dest_deg.tolist()}°")

    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("SHUTDOWN")


if __name__ == '__main__':
    motor_ids = [1, 2, 3]
    can_interface = 'can0'
    
    print("="*70)
    print("INDEPENDENT Multi-Motor Controller (50Hz)")
    print("Socket-based Communication")
    print("="*70)
    
    # Create queue for motor controller
    dest_queue = mp.Queue()
    
    # Start socket listener in background thread
    listener_thread = threading.Thread(
        target=socket_listener, 
        args=(dest_queue, 50000, '127.0.0.1'),
        daemon=True
    )
    listener_thread.start()
    
    # Give socket server time to start
    time.sleep(0.5)
    
    # Run motor controller (blocking)
    motor_can(motor_ids=motor_ids, can_interface=can_interface, dest_queue=dest_queue)
