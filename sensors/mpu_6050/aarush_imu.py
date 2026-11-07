# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  sensors/mpu_6050/aarush_imu.py
# STATUS    ·  SENSOR BENCH TOOL · stand-alone, no robot needed
# ────────────────────────────────────────────────────────────────────────
# Interactive MPU-6050 tool: R = take reference, M = capture maximum
# tilt angles (saved to imu_max_angles.json), S = start. Works out the
# effective height change at each corner of the body cuboid and renders
# a headless matplotlib plot to /tmp/imu_plot.png.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 aarush_imu.py
# ════════════════════════════════════════════════════════════════════════
import smbus2
import time
import math
import sys
import select
import json
import numpy as np
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend — no display needed
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from collections import deque
import os

# ================= FILE =================
MAX_FILE = "imu_max_angles.json"
PLOT_FILE = "/tmp/imu_plot.png"

# ================= BODY DIMENSIONS (mm) =================
L, W, H = 400, 280, 160

LEGS = {
    "A": np.array([ L/2, -W/2, 0]),  # Front Left
    "B": np.array([ L/2,  W/2, 0]),  # Front Right
    "C": np.array([-L/2,  W/2, 0]),  # Rear Right
    "D": np.array([-L/2, -W/2, 0])   # Rear Left
}

# ================= MPU SETUP =================
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

ACCEL_BIAS = [-0.0287, -0.0047, -0.0051]

# ================= IMU =================
def read_word(reg):
    h = bus.read_byte_data(MPU_ADDR, reg)
    l = bus.read_byte_data(MPU_ADDR, reg + 1)
    v = (h << 8) + l
    return v - 65536 if v >= 0x8000 else v

def get_accel():
    ax = read_word(ACCEL_XOUT_H)     / 16384 - ACCEL_BIAS[0]
    ay = read_word(ACCEL_XOUT_H + 2) / 16384 - ACCEL_BIAS[1]
    az = read_word(ACCEL_XOUT_H + 4) / 16384 - ACCEL_BIAS[2]
    return ax, ay, az

def get_roll_pitch(ax, ay, az):
    roll  = math.degrees(math.atan2(ay, az))
    pitch = math.degrees(math.atan2(-ax, math.sqrt(ay*ay + az*az)))
    return roll, pitch

def rotation_matrix(roll, pitch):
    r = math.radians(roll)
    p = math.radians(-pitch)
    Rx = np.array([[1,0,0],[0,math.cos(r),-math.sin(r)],[0,math.sin(r),math.cos(r)]])
    Ry = np.array([[math.cos(p),0,math.sin(p)],[0,1,0],[-math.sin(p),0,math.cos(p)]])
    return Ry @ Rx

# ================= PHYSICS =================
def effective_z(x, y, roll, pitch):
    return x * math.sin(math.radians(pitch)) + y * math.sin(math.radians(roll))

def level_from_angle(val, max_pos, max_neg):
    if val > 0 and max_pos > 0:
        return min(int(abs(val / max_pos) * 10), 10)
    if val < 0 and max_neg < 0:
        return min(int(abs(val / max_neg) * 10), 10)
    return 0

def color_from_level(lvl):
    if lvl == 0:  return "black"
    if lvl <= 2:  return "green"
    if lvl <= 5:  return "yellow"
    if lvl <= 8:  return "orange"
    return "red"

# ================= REFERENCE =================
def take_reference(n=600):
    sr = sp = 0
    for _ in range(n):
        ax, ay, az = get_accel()
        r, p = get_roll_pitch(ax, ay, az)
        sr += r; sp += p
        time.sleep(0.005)
    ref_r = sr / n
    ref_p = sp / n
    print(f"\nReference captured:")
    print(f"  Roll  reference: {ref_r:.2f} deg")
    print(f"  Pitch reference: {ref_p:.2f} deg\n")
    return ref_r, ref_p

# ================= MAX ANGLE =================
def save_max(data):
    with open(MAX_FILE, "w") as f:
        json.dump(data, f, indent=2)

def load_max():
    try:
        with open(MAX_FILE, "r") as f:
            return json.load(f)
    except:
        return None

def capture_max():
    print("\nRotate IMU to MAX tilt (Ctrl+C to stop)\n")
    m = {"roll_pos": 0, "roll_neg": 0, "pitch_pos": 0, "pitch_neg": 0}
    try:
        while True:
            ax, ay, az = get_accel()
            r, p = get_roll_pitch(ax, ay, az)
            m["roll_pos"]   = max(m["roll_pos"],   r)
            m["roll_neg"]   = min(m["roll_neg"],   r)
            m["pitch_pos"]  = max(m["pitch_pos"],  p)
            m["pitch_neg"]  = min(m["pitch_neg"],  p)
            time.sleep(0.01)
    except KeyboardInterrupt:
        save_max(m)
        print("Max angles saved:", m)

# ================= BODY CUBOID =================
def cuboid_vertices():
    x = [ L/2, -L/2]
    y = [ W/2, -W/2]
    z = [0, -H]
    return np.array([
        [x[0],y[0],z[0]], [x[0],y[1],z[0]], [x[1],y[1],z[0]], [x[1],y[0],z[0]],
        [x[0],y[0],z[1]], [x[0],y[1],z[1]], [x[1],y[1],z[1]], [x[1],y[0],z[1]]
    ])

EDGES = [(0,1),(1,2),(2,3),(3,0),
         (4,5),(5,6),(6,7),(7,4),
         (0,4),(1,5),(2,6),(3,7)]

# ================= PLOT RENDER =================
def render_plot(roll, pitch, hist, z_filt, intensity):
    fig = plt.figure(figsize=(15, 9))

    ax3d = fig.add_subplot(231, projection='3d')
    axA  = fig.add_subplot(232)
    axB  = fig.add_subplot(233)
    axD  = fig.add_subplot(235)
    axC  = fig.add_subplot(236)

    verts = cuboid_vertices() @ rotation_matrix(roll, pitch).T
    for e in EDGES:
        ax3d.plot(*zip(verts[e[0]], verts[e[1]]), color='black')

    ax3d.quiver(0, 0, 0, 200, 0, 0, color='red', linewidth=2)
    ax3d.text(210, 0, 0, "FRONT", color='red')

    extending, retracting = [], []

    for name, (x, y, _) in LEGS.items():
        z = z_filt[name]
        col = color_from_level(intensity)
        ax3d.plot([x, x], [y, y], [0, -z], color=col, linewidth=4)
        ax3d.text(x, y, 0, name, color=col, fontsize=14, weight='bold')

        if z > 0:  extending.append(name)
        elif z < 0: retracting.append(name)

    ax3d.set_xlim(-300, 300)
    ax3d.set_ylim(-300, 300)
    ax3d.set_zlim(-300, 300)
    ax3d.set_title(f"3D Body Orientation & Leg Load  |  Rating {intensity}/10")

    for axg, name in zip([axA, axB, axD, axC], ["A", "B", "D", "C"]):
        axg.cla()
        axg.plot(hist[name])
        axg.axhline(0, linestyle='--')
        axg.set_title(f"Leg {name}")
        if hist[name]:
            axg.text(0.05, 0.9, f"{hist[name][-1]:.1f} mm",
                     transform=axg.transAxes, weight='bold')
        axg.grid()

    fig.text(0.5, 0.03,
             f"EXTENDING: {', '.join(extending)}    |    RETRACTING: {', '.join(retracting)}",
             ha='center', fontsize=12, weight='bold')

    plt.tight_layout()
    plt.savefig(PLOT_FILE, dpi=80)
    plt.close(fig)  # Free memory — critical for long runs

# ================= MAIN =================
max_angles = load_max()
ref_roll = ref_pitch = None

z_filt = {k: 0.0 for k in LEGS}
hist   = {k: deque(maxlen=200) for k in LEGS}
ALPHA  = 0.2

frame = 0
SAVE_EVERY = 5  # Save plot every N frames to reduce CPU load

print("R → Reference | M → Max Capture | S → Start | Q → Stop | X → Exit")
print(f"Plot output: {PLOT_FILE}  (open in VSCode explorer to view)\n")

while True:
    cmd = input("Enter command: ").strip().upper()

    if cmd == "R":
        ref_roll, ref_pitch = take_reference()

    elif cmd == "M":
        capture_max()
        max_angles = load_max()

    elif cmd == "S":
        if ref_roll is None or max_angles is None:
            print("Do R and M first\n")
            continue

        print(f"Sensing... plots saved to {PLOT_FILE} every {SAVE_EVERY} frames")
        print("Press Enter to stop.\n")

        frame = 0
        while True:
            # ---- Non-blocking stop check ----
            if select.select([sys.stdin], [], [], 0)[0]:
                sys.stdin.readline()
                print("Stopped\n")
                break

            ax_v, ay_v, az_v = get_accel()
            roll, pitch = get_roll_pitch(ax_v, ay_v, az_v)
            roll  -= ref_roll
            pitch -= ref_pitch

            r_lvl = level_from_angle(roll,  max_angles["roll_pos"],  max_angles["roll_neg"])
            p_lvl = level_from_angle(pitch, max_angles["pitch_pos"], max_angles["pitch_neg"])
            intensity = max(r_lvl, p_lvl)

            for name, (x, y, _) in LEGS.items():
                z_raw = effective_z(x, y, roll, pitch)
                z_filt[name] = ALPHA * z_raw + (1 - ALPHA) * z_filt[name]
                hist[name].append(z_filt[name])

            # Print live data to terminal always
            ext = [n for n in LEGS if z_filt[n] > 0]
            ret = [n for n in LEGS if z_filt[n] < 0]
            print(f"\rRoll:{roll:+6.1f}°  Pitch:{pitch:+6.1f}°  Rating:{intensity}/10  "
                  f"EXT:{ext}  RET:{ret}  Frame:{frame}", end="", flush=True)

            # Save plot periodically
            frame += 1
            if frame % SAVE_EVERY == 0:
                render_plot(roll, pitch, hist, z_filt, intensity)

            time.sleep(0.05)

    elif cmd == "X":
        print("Exiting")
        bus.close()
        break
