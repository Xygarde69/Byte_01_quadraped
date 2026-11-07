# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/with_ik/_3dof_ik_with_shift_circle_method.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# 4-leg IK (documents the Y-up / XZ-ground coordinate convention).
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import matplotlib.pyplot as plt 
import math 
import visualization_functions as visualize


# ─────────────────────────────────────────────────────────────────────────────
# Coordinate convention used THROUGHOUT this file (Y-up / XZ-ground):
#
#   World frame (what you pass in / get back):
#       X  →  horizontal (fore/aft or left/right)
#       Y  →  vertical   (up)          ← height
#       Z  →  horizontal (left/right or fore/aft)
#
#   The IK solver works in an internal Z-up frame:
#       ik_x = world X
#       ik_y = world Z   (the "other" horizontal)
#       ik_z = world Y   (height)
#
#   The remapping is done automatically inside inverse_kinematics() so the
#   caller always works in the Y-up world frame.
# ─────────────────────────────────────────────────────────────────────────────


def inverse_kinematics(wx, wy, wz, L1, linkConst, L2, L3):
    """
    Solve IK for a target given in the Y-up world frame.

    Parameters
    ----------
    wx, wy, wz : float
        Target position.  wy is the HEIGHT (vertical / up axis).
    L1         : float   Vertical base link length.
    linkConst  : float   Constant horizontal offset (radius of base circle).
    L2, L3     : float   Upper-leg and lower-leg link lengths.

    Returns
    -------
    theta1, theta2, theta3 : float  Joint angles in radians.
    """

    # ── Remap world → IK-internal (Z-up) frame ───────────────────────────────
    # IK internally uses:  horizontal plane = (ik_x, ik_y),  height = ik_z
    ik_x = wx   # world X  stays X
    ik_y = wz   # world Z  becomes IK's Y  (the second horizontal axis)
    ik_z = wy   # world Y  becomes IK's Z  (height)
    # ─────────────────────────────────────────────────────────────────────────

    x, y, z = ik_x, ik_y, ik_z          # short aliases for the solver below
    home_linkConst = (linkConst, 0, L1)

    # Tangent length from origin to target (horizontal plane)
    try:
        lenTangent = math.sqrt(x**2 + y**2 - linkConst**2)
    except ValueError as e:
        radicand = x**2 + y**2 - linkConst**2
        raise ValueError(
            f"Invalid geometry: sqrt of negative value {radicand:.6f}. "
            "Target is too close to origin for given radius. "
            "(Point is inside the circle formed by the constant link)"
        ) from e

    if lenTangent > L2 + L3:
        raise ValueError("Target out of reach: lenTangent > L2 + L3",
                         lenTangent, L2 + L3)

    # Base-plane geometry (unchanged IK logic)
    d = math.sqrt(x**2 + y**2)       # distance from origin to target (horizontal)
    a = np.arctan2(y, x)              # angle from origin to target
    b = np.arccos(linkConst / d)      # angle to tangent point

    T1x = linkConst * np.cos(a + b)
    T1y = linkConst * np.sin(a + b)
    T2x = linkConst * np.cos(a - b)
    T2y = linkConst * np.sin(a - b)

    T2 = (T1x, T1y)
    T1 = (T2x, T2y)
    T  = T1

    theta_A = np.arctan2(home_linkConst[1], home_linkConst[0])
    theta_B = np.arctan2(T[1], T[0])
    theta1  = theta_B - theta_A
    if theta1 < 0:
        theta1 += 2 * np.pi

    # Relative target from the shifted linkConst end
    X = x - T[0]
    Y = y - T[1]
    Z = z - L1                        # height relative to top of L1

    L = np.sqrt(X**2 + Y**2 + Z**2)  # distance to target from shifted joint

    # Lower-leg angle (law of cosines)
    theta3 = -np.arccos((L**2 - L2**2 - L3**2) / (2 * L2 * L3))

    # Upper-leg angle
    beta   = np.arctan2(Z, np.sqrt(X**2 + Y**2))
    alpha  = np.arctan2(L3 * np.sin(theta3), L2 + L3 * np.cos(theta3))
    theta2 = beta - alpha

    if L > L2 + L3 + 1e-9:
        raise ValueError("Target out of reach for L2 and L3:", L, L2 + L3)

    # Uncomment to debug the top-view circle geometry:
    # visualize.top_circle_view(x, y, T1, T2, home_linkConst, linkConst, b, a, theta1)

    return theta1, theta2, theta3


# ─────────────────────────────────────────────────────────────────────────────
if __name__ == '__main__':

    # Link lengths (cm)
    L1        =  5.995   # vertical base link
    linkConst = -9.094   # constant horizontal offset (circle radius when base rotates)
    L2        = 22.0     # upper leg
    L3        = 21.5     # lower leg

    # ── Target in Y-up world frame ────────────────────────────────────────────
    # wx = X (horizontal), wy = Y (UP / height), wz = Z (horizontal)
    wx, wy, wz = -9.094, 6, 22
    print(f"Target  wx={wx}  wy={wy} (height)  wz={wz}")

    theta1, theta2, theta3 = inverse_kinematics(wx, wy, wz, L1, linkConst, L2, L3)
    print(f"Angles  θ1={math.degrees(theta1):.2f}°  "
          f"θ2={math.degrees(theta2):.2f}°  "
          f"θ3={math.degrees(theta3):.2f}°")

    visualize._3dof(L1, linkConst, L2, L3, theta1, theta2, theta3)