# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/animated_movement.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Animates the leg along a path using the IK + matplotlib (link lengths
# in cm).
# ════════════════════════════════════════════════════════════════════════
import _3dof_ik_with_shift_circle_method as ik
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# Link lengths in cm
L1 = 5.995  # Link 1 length
linkConst = 9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length

# Trajectory parameters
x = 9.094
z_start, z_end = 6, -3 # right to left co-ordinates of bots leg for movement
y_start, y_end = 35, 27 # max up and max low of bots leg
y_amplitude = y_start - y_end  # max amplitude of sine wave for movement
num_points = 10  # For smoothness

# Prepare trajectory points
zs = np.linspace(z_start, z_end, num_points) # Interpolate z from start to end (right to left)
ts = np.linspace(0, np.pi, num_points) # Generate parameter t from 0 to π (for half a sine wave)
ys = y_amplitude * np.sin(ts) # y follows a sine curve with amplitude from bottom to top to bottom
coords = [(x, y_end-y, z) for y, z in zip(ys, zs)] # Build the trajectory points

fig = plt.figure(figsize=(8,6))
ax = fig.add_subplot(111, projection='3d')

def update(num):
    ax.cla()
    x, y, z = coords[num]
    theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
    ik.visualize.animate_3dof(L1, linkConst, L2, L3, theta1, theta2, theta3, ax=ax)
    ax.set_title(f'Pose {num+1}')

ani = animation.FuncAnimation(fig, update, frames=len(coords), interval=10)
plt.show()
