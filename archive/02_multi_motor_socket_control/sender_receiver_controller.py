# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/sender_receiver_controller.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Combined sender+receiver controller built on SoftRealtimeLoop.
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import time
import sys
import math
import multiprocessing as mp
# Add imports at top
import threading
import queue  # Thread-safe queue

class SoftRealtimeLoop:
    def __init__(self, dt=0.01, report=True, fade=0):
        self.dt = dt
        self.report = report
        self.fade = fade
        self.start_time = None
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
            pass
        elif sleeptime < -self.dt and self.report:
            print(f"Warning: Loop running {-sleeptime*1000:.1f}ms behind")
        self.iteration += 1
        return time.time() - self.starttime
    



def motor_can(motor_ids=[1,2,3], can_interface='can0', dest_queue=None):
    """
        Continuous position control with external dest queue
    """

    try:
        print("STARTING EXTERNAL QUEUE POSITION CONTROL")
        print("Send dest via dest_queue.put(np.array([10.0,40.0,60.0]))")
        print("Press Ctrl+C to stop")

        num_motors = len(motor_ids)
        current_positions_deg = np.zeros(num_motors)
        current_dest_deg = np.zeros(num_motors)
        src_deg = np.zeros(num_motors)
        dest_deg = np.zeros(num_motors)

        loop = SoftRealtimeLoop(dt=0.02, report=True, fade=0)  # 50Hz

        with loop:
            for t in loop:
                # Read current positions

                # Check for new destination
                new_dest = None
                if dest_queue:
                    try:
                        new_dest = dest_queue.get_nowait()
                    except:
                        pass  # Queue empty

                if new_dest is not None and len(new_dest) == num_motors:
                    if not np.array_equal(new_dest, current_dest_deg):
                        src_deg = current_positions_deg.copy()
                        dest_deg = new_dest.copy()
                        print(f"🎯 NEW MOVE: {src_deg}° → {dest_deg}°")
                        print(f"   Current: {current_positions_deg}°")


    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("SHUTDOWN")
        print("Stopping motors...")


# Replace sender_thread with thread version
def sender_thread(dest_queue):  # Now threading.Queue()
    print("Sender active (same THREAD)")
    destinations = {
        '1': np.array([10.0, 40.0, 60.0]),
        '2': np.array([30.0, 50.0,  0.0]),
        '3': np.array([0.0,  0.0,  0.0])
    }
    try:
        while True:
            cmd = input("Cmd (1/2/3/q): ").strip()
            if cmd.lower() == 'q': break
            if cmd in destinations:
                dest_queue.put(destinations[cmd])
                print(f"✅ Sent dest: {destinations[cmd]}°")
            else:
                print("❌ Invalid: 1/2/3/q")
    except EOFError:
        print("Sender input closed")

if __name__ == '__main__':
    motor_ids = [1, 2, 3]
    can_interface = 'can0'
    
    print("="*70)
    print("AK60-6 V3.0 Position Test - Multiple Motors, S-curve")
    print("MODIFIED: Thread-based interactive sender + 50Hz loop")
    print("="*70)
    
    # THREAD-SAFE queue (same process)
    dest_queue = queue.Queue()  # NOT mp.Queue!
    
    # Start sender THREAD (not process)
    sender_thread_obj = threading.Thread(target=sender_thread, args=(dest_queue,), daemon=True)
    sender_thread_obj.start()
    
    try:
        motor_can(motor_ids=motor_ids, can_interface=can_interface, dest_queue=dest_queue)
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    