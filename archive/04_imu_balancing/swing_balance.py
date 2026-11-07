# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/04_imu_balancing/swing_balance.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Swing-balance test: stand up, capture an IMU reference, shift weight
# to one side, then run a low-gain balancing loop (low-pass filter +
# deadzone).
# ════════════════════════════════════════════════════════════════════════
import socket
import pickle
import time
import threading
import math
from mpu6050 import mpu6050

# -- Configuration --
HOST = "127.0.0.1"
PORT = 50000

# Base Coordinates
X_NOM = -9.094
Y_STAND = 25.0 
Y_LIFT = 18.0   # Lift height confirmed
Z_NOM = 9.0

# --- STABILIZED TUNING (Reverted to Low Gains) ---
KP = 0.08          # Back to low gain to stop "crazy" movement
KD = 0.005         # Damping/Braking effect
FILTER_ALPHA = 0.1 # Strong filtering to ignore motor vibration
DEADZONE_DEG = 1.2 # "Offset" you requested: ignores tilts < 1.2 degrees

# --- TEST SETTINGS ---
TARGET_SWING_LEG = "fr" 
SHIFT_AMOUNT = 1.5      # Back to subtle shift

class SwingBalanceTest:
    def __init__(self):
        self.sensor = mpu6050(0x68)
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.filt_roll = 0.0
        self.filt_pitch = 0.0
        self.is_active = False

    def get_data(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro = self.sensor.get_gyro_data() 
            raw_r = math.degrees(math.atan2(accel['y'], accel['z']))
            raw_p = math.degrees(math.atan2(-accel['x'], math.sqrt(accel['y']**2 + accel['z']**2)))
            
            # Low-pass filter
            self.filt_roll = (FILTER_ALPHA * raw_r) + ((1.0 - FILTER_ALPHA) * self.filt_roll)
            self.filt_pitch = (FILTER_ALPHA * raw_p) + ((1.0 - FILTER_ALPHA) * self.filt_pitch)
            return self.filt_roll, self.filt_pitch, gyro['x'], gyro['y']
        except:
            return self.filt_roll, self.filt_pitch, 0.0, 0.0

    def apply_deadzone(self, error, threshold):
        """Remaps error to ignore small tilts."""
        if error > threshold: return error - threshold
        if error < -threshold: return error + threshold
        return 0.0

    def send_to_robot(self, payload):
        try:
            data = pickle.dumps(payload)
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(0.04)
                s.connect((HOST, PORT))
                s.sendall(data)
        except:
            pass

    def run_test(self):
        # 1. Stand Up
        print("Moving to Stand...")
        self.send_to_robot({l: [X_NOM, Y_STAND, Z_NOM] for l in ["fl", "bl", "fr", "br"]})
        time.sleep(2.0)

        # 2. Set Reference
        r, p, _, _ = self.get_data()
        self.ref_roll, self.ref_pitch = r, p
        print(f"Ref set. Deadzone active at {DEADZONE_DEG} degrees.")

        # 3. Weight Shift (Reverted Direction)
        dx_shift = SHIFT_AMOUNT if "l" in TARGET_SWING_LEG else -SHIFT_AMOUNT
        dz_shift = -SHIFT_AMOUNT if "f" in TARGET_SWING_LEG else SHIFT_AMOUNT
        
        # 4. Balancing Loop
        print(f"Lifting {TARGET_SWING_LEG}...")
        self.is_active = True
        
        while self.is_active:
            curr_r, curr_p, vel_r, vel_p = self.get_data()
            
            # Error Calculation with Deadzone
            err_r = self.apply_deadzone(curr_r - self.ref_roll, DEADZONE_DEG)
            err_p = self.apply_deadzone(curr_p - self.ref_pitch, DEADZONE_DEG)

            # PD Balance Logic
            dy_r = (err_r * KP) + (vel_r * KD)
            dy_p = (err_p * KP) + (vel_p * KD)

            payload = {}
            for leg in ["fl", "bl", "fr", "br"]:
                if leg == TARGET_SWING_LEG:
                    # Swing Leg: Moves to 18cm, no IMU interference
                    payload[leg] = [X_NOM + dx_shift, Y_LIFT, Z_NOM + dz_shift]
                else:
                    # Stance Legs: Counter-balance the weight
                    y_adj = Y_STAND
                    if leg == "fl": y_adj += (dy_r + dy_p)
                    if leg == "fr": y_adj += (-dy_r + dy_p)
                    if leg == "bl": y_adj += (dy_r - dy_p)
                    if leg == "br": y_adj += (-dy_r - dy_p)
                    
                    payload[leg] = [X_NOM + dx_shift, y_adj, Z_NOM + dz_shift]

            self.send_to_robot(payload)
            time.sleep(0.01)

if __name__ == "__main__":
    tester = SwingBalanceTest()
    try:
        tester.run_test()
    except KeyboardInterrupt:
        print("Stopping...")