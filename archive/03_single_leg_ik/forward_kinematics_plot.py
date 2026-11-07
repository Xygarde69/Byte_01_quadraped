# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/forward_kinematics_plot.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Forward-kinematics plot of a leg (linkConst sign differs left/right).
# ════════════════════════════════════════════════════════════════════════
import _3dof_ik_with_shift_circle_method as ik
import matplotlib.pyplot as plt
import numpy as np
import math


# Link lengths in cm
L1          = 5.995
linkConst   = -9.094 # for right legs, 9.094 for left legs, as constant link is on opposite sides for the two legs
L2          = 22
L3          = 21.5


def forward_kinematics(theta1, theta2, theta3): 
    # Base origin
    O = np.array([0, 0, 0])

    # First joint above base is vertical link L1
    J1 = np.array([0, 0, L1])

    shifted_linkConst = (linkConst*np.cos(theta1), linkConst*np.sin(theta1))

    # New constant link extending along x axis from J1
    J_const = J1 + np.array([shifted_linkConst[0], shifted_linkConst[1], 0])

    # Second joint position (shoulder) relative to J_const using shoulder angle theta2 and base twist theta1
    J2 = J_const + np.array([
        L2 * np.cos(theta2) * np.cos(theta1+math.pi/2),
        L2 * np.cos(theta2) * np.sin(theta1+math.pi/2),
        L2 * np.sin(theta2)
    ])

    # End effector (third joint) position using sum of shoulder and elbow angles theta2 + theta3, relative to J2
    J3 = J2 + np.array([
        L3 * np.cos(theta2 + theta3) * np.cos(theta1+math.pi/2), # x
        L3 * np.cos(theta2 + theta3) * np.sin(theta1+math.pi/2), # y
        L3 * np.sin(theta2 + theta3) # z
    ])

    return J3

def plot_pose(theta1, theta2, theta3):
    # Let visualize._3dof create and own the figure — don't make one yourself
    ik.visualize._3dof(L1, linkConst, L2, L3, theta1, theta2, theta3)
    plt.show()


if __name__ == '__main__':
    theta1, theta2, theta3 = np.radians([0, 127, -123])  # motor angles (as per ik) after homing sequence
    print("X, Y, Z: ", forward_kinematics(theta1, theta2, theta3))
    plot_pose(theta1, theta2, theta3)
