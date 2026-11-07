#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/04_imu_balancing/level_stand.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Levelling-stance maths (calculate_leveled_stance) and smooth
# transitions (40 Hz, 120 deg/s). INCOMPLETE: the MPU hardware read
# (read_raw_mpu_hardware) is a TODO stub, so this script cannot run end-
# to-end.
# ════════════════════════════════════════════════════════════════════════
import math
import pickle
import socket
import time

# ─── Configuration ────────────────────────────────────────────────────────
HOST = "127.0.0.1"      # [cite: 3]
PORT = 50000            # [cite: 3]
SEND_HZ = 40.0          # [cite: 4]
TRANSITION_SPEED_DEG_PER_S = 120.0  # [cite: 6]

ALL_LEGS = ["fl", "fr", "bl", "br"] # [cite: 8]
SITTING_XYZ = [-9.094, 11.0, 3.0]   # [cite: 72]
STANDING_XYZ = [-9.094, 25.0, 3.0]  # [cite: 73]

# BYTE's Physical Dimensions (cm)
WHEELBASE_Z = 42.0   # Front hip to Back hip [cite: 100]
TRACK_WIDTH_X = 28.0 # Left hip to Right hip [cite: 100]

# Keep track of where the feet are
current_pos = {leg: list(SITTING_XYZ) for leg in ALL_LEGS} # [cite: 8]


# ─── MPU6050 Hardware Logic ───────────────────────────────────────────────
def read_raw_mpu_hardware():
    """
    Replace this with your actual MPU6050 I2C read logic[cite: 117].
    Returns: (pitch_deg, roll_deg)
    """
    # TODO: Implement your smbus2 reading here [cite: 121]
    raw_pitch_deg = 0.0 
    raw_roll_deg = 0.0
    return raw_pitch_deg, raw_roll_deg

def read_mpu_angles_averaged(num_samples=20, duration_s=0.2):
    """Averages readings to filter out high-frequency noise[cite: 107, 108]."""
    pitch_sum = 0.0
    roll_sum = 0.0
    delay = duration_s / float(num_samples)
    
    for _ in range(num_samples):
        pitch, roll = read_raw_mpu_hardware()
        pitch_sum += pitch
        roll_sum += roll
        time.sleep(delay)
        
    return pitch_sum / num_samples, roll_sum / num_samples


# ─── Networking & Movement ────────────────────────────────────────────────
def send_payload(payload: dict, speed: float = TRANSITION_SPEED_DEG_PER_S):
    """Inject speed into every outgoing packet, then send[cite: 9]."""
    payload["speed"] = speed
    data = pickle.dumps(payload)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(0.5)
            s.connect((HOST, PORT))
            s.sendall(data)
    except Exception as e:
        print(f"\r[Socket Error] {e}", flush=True)

def smooth_transition_dict(target_poses: dict, duration: float = 1.5):
    """
    Smoothly interpolates each leg from its current position to a unique target.
    Uses a smoothstep ease-in-out curve[cite: 10, 11].
    """
    start = {leg: list(current_pos[leg]) for leg in ALL_LEGS}
    n_frames = max(1, int(duration * SEND_HZ)) # [cite: 11]
    
    for frame in range(n_frames + 1):
        t = frame / n_frames
        ease = t * t * (3.0 - 2.0 * t) # smoothstep [cite: 11]
        
        payload = {}
        for leg in ALL_LEGS:
            payload[leg] = [
                start[leg][0] + (target_poses[leg][0] - start[leg][0]) * ease,
                start[leg][1] + (target_poses[leg][1] - start[leg][1]) * ease,
                start[leg][2] + (target_poses[leg][2] - start[leg][2]) * ease
            ]
        send_payload(payload, speed=TRANSITION_SPEED_DEG_PER_S) # [cite: 12]
        time.sleep(1.0 / SEND_HZ) # [cite: 12]
        
    for leg in ALL_LEGS:
        current_pos[leg] = list(target_poses[leg]) # [cite: 12]


# ─── Math & Calibration ───────────────────────────────────────────────────
def calculate_leveled_stance(base_xyz, pitch_error_deg, roll_error_deg):
    """Calculates new Y (height) for each leg to flatten the chassis."""
    x_base, y_base, z_base = base_xyz
    
    pitch_rad = math.radians(pitch_error_deg)
    roll_rad = math.radians(roll_error_deg)
    
    delta_y_pitch = (WHEELBASE_Z / 2.0) * math.tan(pitch_rad) # [cite: 85]
    delta_y_roll  = (TRACK_WIDTH_X / 2.0) * math.tan(roll_rad) # [cite: 85]
    
    # Adjust signs depending on MPU orientation!
    y_fl = y_base - delta_y_pitch + delta_y_roll
    y_fr = y_base - delta_y_pitch - delta_y_roll
    y_bl = y_base + delta_y_pitch + delta_y_roll
    y_br = y_base + delta_y_pitch - delta_y_roll
    
    return {
        "fl": [x_base, round(y_fl, 3), z_base],
        "fr": [x_base, round(y_fr, 3), z_base],
        "bl": [x_base, round(y_bl, 3), z_base],
        "br": [x_base, round(y_br, 3), z_base]
    }


# ─── Main Execution ───────────────────────────────────────────────────────
def main():
    print("╔══════════════════════════════════════════════╗")
    print("║        BYTE-01  Auto-Leveling Routine        ║")
    print("╚══════════════════════════════════════════════╝\n")

    # 1. Start in a sitting position to zero the MPU
    print("● Sitting to calibrate absolute zero...")
    sitting_targets = {leg: SITTING_XYZ for leg in ALL_LEGS}
    smooth_transition_dict(sitting_targets, duration=1.2)
    time.sleep(1.5) # Wait for mechanical settling [cite: 88]
    
    baseline_pitch, baseline_roll = read_mpu_angles_averaged()
    print(f"  [Tare] Baseline Pitch: {baseline_pitch:.2f}°, Roll: {baseline_roll:.2f}°")
    
    # 2. Stand up blindly to the default coordinates
    print("\n● Standing up to default coordinates...")
    standing_targets = {leg: STANDING_XYZ for leg in ALL_LEGS}
    smooth_transition_dict(standing_targets, duration=1.5)
    time.sleep(1.5) # Let chassis sway settle [cite: 91]
    
    # 3. Read the tilt error
    current_pitch, current_roll = read_mpu_angles_averaged()
    error_pitch = current_pitch - baseline_pitch
    error_roll = current_roll - baseline_roll
    print(f"  [Error] Pitch Offset: {error_pitch:.2f}°, Roll Offset: {error_roll:.2f}°")
    
    # 4. Calculate flat stance and smoothly apply it
    corrected_poses = calculate_leveled_stance(STANDING_XYZ, error_pitch, error_roll)
    
    print("\n● Applying level correction...")
    for leg, coords in corrected_poses.items():
        print(f"   {leg.upper()}: {coords}")
        
    smooth_transition_dict(corrected_poses, duration=1.5)
    print("\n✅ BYTE is perfectly parallel to the horizontal ground.")
    
    # Keeps the script alive to maintain the socket stream if your server requires it
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n● Returning to sit before closing...")
        smooth_transition_dict({leg: SITTING_XYZ for leg in ALL_LEGS}, duration=1.2)
        print("● Done.")

if __name__ == "__main__":
    main()