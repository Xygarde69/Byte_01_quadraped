# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/04_imu_balancing/imu_balance_test.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# IMU balance experiment: reads the MPU-6050 and sends corrected foot
# positions around a base stand pose (X_NOM = -9.094) to
# 127.0.0.1:50000.
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

# --- STABILITY & DEADZONE TUNING ---
KP = 0.1          # Proportional: Reaction to angle
KD = 0.005         # Derivative: The "Brake" (Damping)
FILTER_ALPHA = 0.1 # Smoothing: Lower is slower/smoother

# TILT THRESHOLD (The "Offset" you asked for)
# The bot will ignore any tilt smaller than this many degrees.
DEADZONE_DEG = 5.0 # Increase this to 2.0 or 3.0 if it still shakes at idle.

MAX_ADJ = 5.0      # Max +/- cm correction

class BalanceController:
    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.is_balancing = False
        self.running = True
        
        # State variables
        self.filt_roll = 0.0
        self.filt_pitch = 0.0

    def get_data(self):
        """Reads IMU and applies filtering."""
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data() 

            raw_r = math.degrees(math.atan2(accel['y'], accel['z']))
            raw_p = math.degrees(math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
            
            # Low-pass filter to ignore motor vibrations
            self.filt_roll = (FILTER_ALPHA * raw_r) + ((1.0 - FILTER_ALPHA) * self.filt_roll)
            self.filt_pitch = (FILTER_ALPHA * raw_p) + ((1.0 - FILTER_ALPHA) * self.filt_pitch)
            
            return self.filt_roll, self.filt_pitch, gyro['x'], gyro['y']
        except:
            return self.filt_roll, self.filt_pitch, 0.0, 0.0

    def apply_deadzone(self, error, threshold):
        """Remaps error so it starts smoothly after the threshold."""
        if error > threshold:
            return error - threshold
        if error < -threshold:
            return error + threshold
        return 0.0

    def set_reference(self):
        r, p, _, _ = self.get_data()
        self.ref_roll = r
        self.ref_pitch = p
        print(f"\n[REF SET] Roll: {r:.2f}, Pitch: {p:.2f}. Deadzone: {DEADZONE_DEG}°")
        self.is_balancing = True

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
        stand_payload = {leg: [X_NOM, Y_STAND, Z_NOM] for leg in ["fl", "bl", "fr", "br"]}
        self.send_to_robot(stand_payload)
        time.sleep(1.5) 
        
        while self.running:
            if self.is_balancing:
                curr_r, curr_p, vel_r, vel_p = self.get_data()
                
                # 1. Raw Error
                raw_err_r = curr_r - self.ref_roll
                raw_err_p = curr_p - self.ref_pitch

                # 2. Apply the "Offset" (Deadzone)
                # This ignores any tilt within +/- DEADZONE_DEG
                active_err_r = self.apply_deadzone(raw_err_r, DEADZONE_DEG)
                active_err_p = self.apply_deadzone(raw_err_p, DEADZONE_DEG)

                # 3. PD Calculation
                # dy = (Smooth Error * Kp) + (Velocity * Kd)
                dy_r = (active_err_r * KP) + (vel_r * KD)
                dy_p = (active_err_p * KP) + (vel_p * KD)

                # 4. Clamp and Coordinate
                dy_r = max(min(dy_r, MAX_ADJ), -MAX_ADJ)
                dy_p = max(min(dy_p, MAX_ADJ), -MAX_ADJ)

                payload = {
                    "fl": [X_NOM, Y_STAND + dy_r + dy_p, Z_NOM],
                    "fr": [X_NOM, Y_STAND - dy_r + dy_p, Z_NOM],
                    "bl": [X_NOM, Y_STAND + dy_r - dy_p, Z_NOM],
                    "br": [X_NOM, Y_STAND - dy_r - dy_p, Z_NOM]
                }
                self.send_to_robot(payload)

            time.sleep(0.01) # 100Hz loop

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
    controller = BalanceController()
    t = threading.Thread(target=controller.loop)
    t.daemon = True
    t.start()

    print("="*40)
    print(" BYTE01 PD STABLE BALANCE")
    print(" 1. Press 's' to set Reference Plane.")
    print(" 2. Press 'q' to Quit.")
    print("="*40)

    try:
        while True:
            key = get_key()
            if key == 's':
                controller.set_reference()
            elif key == 'q':
                controller.running = False
                break
    except KeyboardInterrupt:
        controller.running = False