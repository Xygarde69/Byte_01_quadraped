# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/with_ik/_3dof_ik_with_shift_circle_method_old.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Previous IK revision.
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import matplotlib.pyplot as plt 
import math 
import visualization_functions as visualize


def inverse_kinematics(x, y, z, L1, linkConst, L2, L3):
    home_linkConst = (linkConst, 0, L1)

    # Calculate lenTangent and check reachability  
    try:
        lenTangent = math.sqrt(x**2 + y**2 - linkConst**2)
    except ValueError as e:
        radicand = x**2 + y**2 - linkConst**2
        raise ValueError(
            f"Invalid geometry: sqrt of negative value {radicand:.6f}. "
            "Target is too close to origin for given radius. (Point inside circle formed by constant link)"
        ) from e
    
    if lenTangent > L2 + L3:
        raise ValueError("Target is out of reach lenTangent > L2 + L3", lenTangent, L2 + L3)

    
    # Calculate d, a, b
    d = math.sqrt(x**2 + y**2) # distance from origin to target
    a = np.arctan2(y, x) # angle from origin to target
    b = np.arccos(linkConst / d) # angle from origin to tangent point

    # Calculate tangent points T1 and T2
    T1x = linkConst * np.cos(a + b)
    T1y = linkConst * np.sin(a + b)
    T2x = linkConst * np.cos(a - b)
    T2y = linkConst * np.sin(a - b) 

    T2 = (T1x, T1y)
    T1 = (T2x, T2y)

    T = T1

    # Calculate angle of points A and B from origin using atan2
    theta_A = np.arctan2(home_linkConst[1], home_linkConst[0])  # For A=(r,0), this is 0
    theta_B = np.arctan2(T[1], T[0]) # (y,x) order for atan2
    
    # Calculate angle difference in radians
    theta1 = theta_B - theta_A
     
    # Normalize angle to range 0 to 2*pi
    if theta1 < 0:
        theta1 += 2 * np.pi

    X = x - T[0]
    Y = y - T[1]
    Z = z - L1

    # Distance from SHIFTED linkConst end to target
    L = np.sqrt(X**2 + Y**2 + Z**2)

    # lower-leg angle using law of cosines
    theta3 = -np.arccos((L**2 - L2**2 - L3**2) / (2 * L2 * L3))

    # upper-leg angle calculation
    beta = np.arctan2(Z, np.sqrt(X**2 + Y**2)) 
    alpha = np.arctan2(L3 * np.sin(theta3), L2 + (L3 * np.cos(theta3)))
    theta2 = beta - alpha
 
    if L > L2 + L3 + 1e-9:
        raise ValueError("Target out of reach for links L2 and L3: ", L, L2 + L3) 
    
    # visualize.top_circle_view(x, y, T1, T2, home_linkConst, linkConst, b, a, theta1)
    return theta1, theta2, theta3



if __name__ == '__main__':

    # Define the inverse kinematics parameters

    # Link lengths in cm
    L1 = 5.995  # Link 1 length
    linkConst = -9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
    L2 = 22  # Link 2 length
    L3 = 21.5  # Link 3 length

    x, y, z = -9.094, 22, 6 # Target point

    print(x, y, z)


    theta1, theta2, theta3 = inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
    print(math.degrees(theta1), math.degrees(theta2), math.degrees(theta3)) 

    # theta1, theta2, theta3 = np.radians(282), np.radians(-5), np.radians(-5)
    # theta1, theta2, theta3 = 0,0,0


    visualize._3dof(L1, linkConst, L2, L3, theta1, theta2, theta3)

    

    