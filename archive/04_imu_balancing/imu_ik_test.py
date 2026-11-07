#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/04_imu_balancing/imu_ik_test.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# IMU-stabilised weight-shift test: shifts the body (e.g. right /
# backward, away from FL) while an IMU feedback loop keeps it stable;
# has stability and step-size tuning blocks.
# ════════════════════════════════════════════════════════════════════════
import socket
import pickle
import time
import sys
import termios
import tty
import threading
import math
from mpu6050 import mpu6050

# -- Configuration --
HOST = "127.0.0.1"
PORT = 50000

# Base Stand Position
X_NOM = -9.094
Y_STAND = 25.0
Z_NOM = 9.0

# --- STABILITY TUNING ---
KP = 0.08          
KD = 0.005         
FILTER_ALPHA = 0.1 
DEADZONE_DEG = 1.0 
MAX_ADJ = 5.0      

# --- STEP SETTINGS ---
TARGET_LEG = "fl"
TARGET_COORDS = [-9.094, 18.0, 9.0] # Lifted position

# =================================================================
# COORDINATE AXIS MAPPING (Tweak these based on real-world movement)
# =================================================================
# Goal: Shift the body RIGHT (away from FL)
SHIFT_X = -3.5  # If the body moves LEFT instead, change this to +3.5

# Goal: Shift the body BACKWARD (away from FL)
SHIFT_Z = -3.5  # Flipped to Negative: This should now pull the body BACK
# =================================================================

class PhysicsDebugger:
    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.running = True
        
        # State tracking
        self.shifted = False
        self.lifted = False
        self.balancing = False
        
        self.filt_roll = 0.0
        self.filt_pitch = 0.0

    def get_data(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data() 
            raw_r = math.degrees(math.atan2(accel['y'], accel['z']))
            raw_p = math.degrees(math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
            
            self.filt_roll = (FILTER_ALPHA * raw_r) + ((1.0 - FILTER_ALPHA) * self.filt_roll)
            self.filt_pitch = (FILTER_ALPHA * raw_p) + ((1.0 - FILTER_ALPHA) * self.filt_pitch)
            return self.filt_roll, self.filt_pitch, gyro['x'], gyro['y']
        except:
            return self.filt_roll, self.filt_pitch, 0.0, 0.0

    def deadzone(self, err, thresh):
        if err > thresh: return err - thresh
        if err < -thresh: return err + thresh
        return 0.0

    def send_to_robot(self, payload):
        try:
            data = pickle.dumps(payload)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.04)
                s.connect((HOST, PORT))
                s.sendall(data)
        except Exception:
            pass 

    def loop(self):
        print("Moving to Standing Position...")
        self.send_to_robot({leg: [X_NOM, Y_STAND, Z_NOM] for leg in ["fl", "bl", "fr", "br"]})
        time.sleep(1.5)
        
        # Capture reference plane at startup
        self.ref_roll, self.ref_pitch, _, _ = self.get_data()
        
        while self.running:
            curr_r, curr_p, vel_r, vel_p = self.get_data()
            
            # Base variables
            dx = SHIFT_X if self.shifted else 0.0
            dz = SHIFT_Z if self.shifted else 0.0
            
            dy_r, dy_p = 0.0, 0.0

            if self.balancing:
                err_r = self.deadzone(curr_r - self.ref_roll, DEADZONE_DEG)
                err_p = self.deadzone(curr_p - self.ref_pitch, DEADZONE_DEG)
                
                dy_r = max(min((err_r * KP) + (vel_r * KD), MAX_ADJ), -MAX_ADJ)
                dy_p = max(min((err_p * KP) + (vel_p * KD), MAX_ADJ), -MAX_ADJ)

            payload = {}
            for leg in ["fl", "bl", "fr", "br"]:
                if self.lifted and leg == TARGET_LEG:
                    # Target leg goes to user coordinates, ignores everything else
                    payload[leg] = list(TARGET_COORDS)
                else:
                    # Stance legs get shift + IMU corrections
                    y_adj = Y_STAND
                    if self.balancing:
                        # Standard 4-leg math applied to remaining legs
                        if leg == "fr": y_adj += (-dy_r + dy_p)
                        if leg == "bl": y_adj += (dy_r - dy_p)
                        if leg == "br": y_adj += (-dy_r - dy_p)
                        if leg == "fl": y_adj += (dy_r + dy_p) # Only active if not lifted
                    
                    payload[leg] = [X_NOM + dx, y_adj, Z_NOM + dz]

            self.send_to_robot(payload)
            time.sleep(0.01)

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == "__main__":
    ctrl = PhysicsDebugger()
    t = threading.Thread(target=ctrl.loop)
    t.daemon = True
    t.start()

    print("="*50)
    print(" BYTE01 PHYSICS DEBUGGER (FL Lift)")
    print(" Press these in order:")
    print(" '1' -> SHIFT weight (Body should move Back & Right)")
    print(" '2' -> LIFT the FL leg (Bot should not fall!)")
    print(" '3' -> BALANCE (Activate IMU on remaining 3 legs)")
    print(" 'r' -> RESET everything to normal standing")
    print(" 'q' -> Quit")
    print("="*50)

    try:
        while True:
            key = get_key()
            if key == '1':
                ctrl.shifted = True
                print("[1] Weight shifted! Did the body move Back and Right?")
            elif key == '2':
                ctrl.lifted = True
                print("[2] FL Lifted! Is it standing on 3 legs without tipping?")
            elif key == '3':
                ctrl.balancing = True
                print("[3] IMU Active! Push the bot gently.")
            elif key == 'r':
                ctrl.shifted = False
                ctrl.lifted = False
                ctrl.balancing = False
                print("[RESET] Back to 4 legs, no shift, no IMU.")
            elif key == 'q':
                ctrl.running = False
                break
    except KeyboardInterrupt:
        ctrl.running = False