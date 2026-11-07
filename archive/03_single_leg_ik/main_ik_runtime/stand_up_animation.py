# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/main_ik_runtime/stand_up_animation.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# get_position(t): smooth trajectory used by the stand-up.
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import time
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import _3dof_ik_with_shift_circle_method as ik

def get_position(t, total_time=2.0):
    """
    Get smooth position along trajectory at time t.
    
    Parameters:
    -----------
    t : float
        Time in seconds (0 <= t <= total_time)
    total_time : float
        Total duration of movement in seconds
    
    Returns:
    --------
    tuple : (x, y, z) coordinates at time t
    """
    # Parameters
    x = 9.094
    z_start, z_end = 6, 6
    y_start, y_end = 22, 38

    # z_start, z_end = 6, -3 # right to left co-ordinates of bots leg for movement
    # y_start, y_end = 35, 27 # max up and max low of bots leg

    y_amplitude = abs(y_start - y_end)
    
    # Normalize time to [0, 1]
    t_normalized = t / total_time
    
    # Continuous equations
    x_t = x  # Constant
    z_t = z_start + (z_end - z_start) * t_normalized  # Linear
    # print(np.sin(t_normalized * np.pi))
    y_t = y_start + y_amplitude * abs(np.sin(t_normalized * np.pi))  # Sine curve
    
    return (x_t, y_t, z_t)




def animate_motion(x, y, z):
    # Link lengths in cm
    L1 = 5.995  # Link 1 length
    linkConst = -9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
    L2 = 22  # Link 2 length
    L3 = 21.5  # Link 3 length

    theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
    ik.visualize.animate_3dof(L1, linkConst, L2, L3, theta1, theta2, theta3, ax=ax)

def update(frame):
    ax.cla() 
    cur_time = time.time() - anim_at
    x,y,z = get_position(t=cur_time, total_time=total_anim_time)
    # print(f"Time: {cur_time:.2f}s, Target: ({x:.2f}, {y:.2f}, {z:.2f})")
    animate_motion(x, y, z)
    ax.set_title(f'Pose at {cur_time:.2f}s')



if __name__ == '__main__':
    fig = plt.figure(figsize=(8,6))
    ax = fig.add_subplot(111, projection='3d')


    anim_at = time.time()
    total_anim_time = 4.0
    interval_time = 10  # milliseconds

    ani = animation.FuncAnimation(fig, update,frames=int(total_anim_time*1000 / interval_time), interval=interval_time)
    plt.show()