# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/04_imu_balancing/imu_balance_test2.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Second iteration of the IMU balance controller: PD tuning, filter,
# deadzone, limits and calibration blocks at the top.
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


# ============================================================
#  CONFIGURATION
# ============================================================
HOST = "127.0.0.1"
PORT = 50000

# Base Positions
X_NOM    = -9.094
Y_CROUCH =  11.0
Y_STAND  =  25.0
Z_NOM    =   3.0

# Rise speed
Y_RISE_STEP  = 0.5
Y_RISE_DELAY = 0.03

# --- PD TUNING ---
KP = 0.04
KD = 0.008

# --- FILTER ---
COMP_ALPHA = 0.98

# --- DEADZONE ---
DEADZONE_DEG = 5.0

# --- LIMITS ---
MAX_ADJ_X = 3.0
MAX_ADJ_Z = 3.0

# --- CALIBRATION ---
GYRO_CALIB_SAMPLES = 500
REF_AVG_SAMPLES    = 100


# ============================================================
#  BALANCE CONTROLLER
# ============================================================
class BalanceController:
    def __init__(self):
        self.sensor = mpu6050(0x68)

        self.ref_roll  = 0.0
        self.ref_pitch = 0.0
        self.is_balancing = False
        self.running = True

        self.filt_roll  = 0.0
        self.filt_pitch = 0.0
        self.last_time  = time.time()

        self.gyro_bias_x = 0.0
        self.gyro_bias_y = 0.0

        self.current_y = Y_CROUCH

        print("\n[STEP 1] Calibrating gyroscope...")
        print("         Keep the robot perfectly still for ~1 second.")
        self.calibrate_gyro()
        print(f"         Done. Bias X={self.gyro_bias_x:.4f}  Y={self.gyro_bias_y:.4f}")


    def calibrate_gyro(self):
        bx, by = 0.0, 0.0
        for _ in range(GYRO_CALIB_SAMPLES):
            g = self.sensor.get_gyro_data()
            bx += g['x']
            by += g['y']
            time.sleep(0.002)
        self.gyro_bias_x = bx / GYRO_CALIB_SAMPLES
        self.gyro_bias_y = by / GYRO_CALIB_SAMPLES


    def get_data(self):
        try:
            accel = self.sensor.get_accel_data()
            gyro  = self.sensor.get_gyro_data()

            now = time.time()
            dt  = now - self.last_time
            self.last_time = now

            gx = gyro['x'] - self.gyro_bias_x
            gy = gyro['y'] - self.gyro_bias_y

            accel_roll  = math.degrees(math.atan2(accel['y'], accel['z']))
            accel_pitch = math.degrees(math.atan2(
                            -accel['x'],
                            math.sqrt(accel['y']**2 + accel['z']**2)))

            self.filt_roll  = (COMP_ALPHA * (self.filt_roll  + gx * dt)
                               + (1 - COMP_ALPHA) * accel_roll)
            self.filt_pitch = (COMP_ALPHA * (self.filt_pitch + gy * dt)
                               + (1 - COMP_ALPHA) * accel_pitch)

            return self.filt_roll, self.filt_pitch, gx, gy

        except Exception:
            return self.filt_roll, self.filt_pitch, 0.0, 0.0


    def apply_deadzone(self, error, threshold):
        if error >  threshold:
            return error - threshold
        if error < -threshold:
            return error + threshold
        return 0.0


    def set_reference(self):
        print(f"\n[STEP 2] Sampling reference plane ({REF_AVG_SAMPLES} samples)...")
        print("         Robot is flat/crouched. Keep it still.")
        rolls, pitches = [], []
        for _ in range(REF_AVG_SAMPLES):
            r, p, _, _ = self.get_data()
            rolls.append(r)
            pitches.append(p)
            time.sleep(0.01)
        self.ref_roll  = sum(rolls)  / len(rolls)
        self.ref_pitch = sum(pitches) / len(pitches)
        print(f"         Reference locked — Roll: {self.ref_roll:.2f}°  Pitch: {self.ref_pitch:.2f}°")


    def rise_to_stand(self):
        print(f"\n[STEP 3] Rising from Y={Y_CROUCH} to Y={Y_STAND}...")
        y = Y_CROUCH
        while y < Y_STAND:
            y = min(y + Y_RISE_STEP, Y_STAND)
            self.current_y = y
            payload = {leg: [X_NOM, y, Z_NOM]
                       for leg in ["fl", "bl", "fr", "br"]}
            self.send_to_robot(payload)
            time.sleep(Y_RISE_DELAY)
        self.current_y = Y_STAND
        print(f"         Standing at Y={Y_STAND}. Now balancing on X-Z plane.")
        self.is_balancing = True


    def stop_balancing(self):
        self.is_balancing = False
        payload = {leg: [X_NOM, self.current_y, Z_NOM]
                   for leg in ["fl", "bl", "fr", "br"]}
        self.send_to_robot(payload)
        print("\n[PAUSED] Balancing stopped. Bot holding position.")
        print("         Press 's' to re-sample and stand again.")
        print("         Press 'q' to quit.")


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
        while self.running:
            if self.is_balancing:
                curr_r, curr_p, vel_r, vel_p = self.get_data()

                raw_err_r = curr_r - self.ref_roll
                raw_err_p = curr_p - self.ref_pitch

                active_err_r = self.apply_deadzone(raw_err_r, DEADZONE_DEG)
                active_err_p = self.apply_deadzone(raw_err_p, DEADZONE_DEG)

                correction_r = (active_err_r * KP) + (vel_r * KD)
                correction_p = (active_err_p * KP) + (vel_p * KD)

                correction_r = max(min(correction_r, MAX_ADJ_X), -MAX_ADJ_X)
                correction_p = max(min(correction_p, MAX_ADJ_Z), -MAX_ADJ_Z)

                dx = correction_r
                dz = correction_p

                payload = {
                    "fl": [X_NOM + dx, self.current_y, Z_NOM + dz],
                    "fr": [X_NOM - dx, self.current_y, Z_NOM + dz],
                    "bl": [X_NOM + dx, self.current_y, Z_NOM - dz],
                    "br": [X_NOM - dx, self.current_y, Z_NOM - dz]
                }
                self.send_to_robot(payload)

            time.sleep(0.01)


# ============================================================
#  KEYBOARD INPUT
# ============================================================
def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


# ============================================================
#  ENTRY POINT
# ============================================================
if __name__ == "__main__":
    controller = BalanceController()

    t = threading.Thread(target=controller.loop)
    t.daemon = True
    t.start()

    print("\n" + "="*45)
    print("  BYTE01 — STATIC BALANCE CONTROLLER")
    print("  Coordinate system:")
    print("    X = left / right shift")
    print("    Y = height (fixed after standing)")
    print("    Z = forward / back reach")
    print("="*45)
    print("  s → Sample reference plane then rise to stand")
    print("  p → Pause / stop balancing (hold position)")
    print("  r → Reset (stop balance, re-sample, re-stand)")
    print("  q → Quit")
    print("="*45)

    try:
        while True:
            key = get_key()

            if key == 's':
                controller.set_reference()
                controller.rise_to_stand()

            elif key == 'p':
                controller.stop_balancing()

            elif key == 'r':
                print("\n[RESET] Stopping balance...")
                controller.is_balancing = False
                time.sleep(0.3)
                controller.set_reference()
                controller.rise_to_stand()

            elif key == 'q':
                print("\n[QUIT] Shutting down.")
                controller.is_balancing = False
                controller.running = False
                break

    except KeyboardInterrupt:
        controller.is_balancing = False
        controller.running = False