# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/main_ik_runtime/visualization_functions.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Plot helpers (copy).
# ════════════════════════════════════════════════════════════════════════
import numpy as np
import matplotlib.pyplot as plt 
import math

def animate_3dof(L1, linkConst, L2, L3, theta1, theta2, theta3, ax=None):
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

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

    max_reach = L1 + linkConst + L2 + L3
    
    ax.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax.set_xlabel('X (cm)')
    ax.set_ylabel('Y (cm)')
    ax.set_zlabel('Z (cm)')
    ax.set_title('Dog Plane View')
    ax.set_box_aspect([1,1,1])
    ax.set_xlim([-max_reach, max_reach])
    ax.set_ylim([-max_reach, max_reach])
    ax.set_zlim([0, max_reach])
    # ax.view_init(elev=-82, azim=-54, roll=-37) #-2,0,-90
    ax.view_init(elev=-2, azim=0, roll=-90)
    ax.legend()

    # plt.tight_layout() 

def _3dof(L1, linkConst, L2, L3, theta1, theta2, theta3):
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

    max_reach = L1 + linkConst + L2 + L3
    
    fig = plt.figure(figsize=(12, 12))

    # YZ plane view (azim=0)
    ax1 = fig.add_subplot(2, 3, 1, projection='3d')
    ax1.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax1.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax1.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax1.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax1.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax1.set_xlabel('X (cm)')
    ax1.set_ylabel('Y (cm)')
    ax1.set_zlabel('Z (cm)')
    ax1.set_title('YZ Plane View')
    ax1.set_box_aspect([1,1,1])
    ax1.set_xlim([-max_reach, max_reach])
    ax1.set_ylim([-max_reach, max_reach])
    ax1.set_zlim([0, max_reach])
    ax1.view_init(elev=0, azim=0, roll=0)
    ax1.legend()

    # XZ plane view (azim=-90)
    ax2 = fig.add_subplot(2,3,2, projection='3d')
    ax2.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax2.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax2.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax2.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax2.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax2.set_xlabel('X (cm)')
    ax2.set_ylabel('Y (cm)')
    ax2.set_zlabel('Z (cm)')
    ax2.set_title('XZ Plane View')
    ax2.set_box_aspect([1,1,1])
    ax2.set_xlim([-max_reach, max_reach])
    ax2.set_ylim([-max_reach, max_reach])
    ax2.set_zlim([0, max_reach])
    ax2.view_init(elev=0, azim=-90, roll=0)
    ax2.legend()

    # XY plane view (azim=90)
    ax3 = fig.add_subplot(2,3,3, projection='3d')
    ax3.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax3.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax3.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax3.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax3.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax3.set_xlabel('X (cm)')
    ax3.set_ylabel('Y (cm)')
    ax3.set_zlabel('Z (cm)')
    ax3.set_title('XY Plane View')
    ax3.set_box_aspect([1,1,1])
    ax3.set_xlim([-max_reach, max_reach])
    ax3.set_ylim([-max_reach, max_reach])
    ax3.set_zlim([0, max_reach])
    ax3.view_init(elev=90, azim=90, roll=90)
    ax3.legend()

    # Isometric plane view (azim=90)
    ax4 = fig.add_subplot(2,3,4, projection='3d')
    ax4.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax4.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax4.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax4.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax4.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax4.set_xlabel('X (cm)')
    ax4.set_ylabel('Y (cm)')
    ax4.set_zlabel('Z (cm)')
    ax4.set_title('Isometric View')
    ax4.set_box_aspect([1,1,1])
    ax4.set_xlim([-max_reach, max_reach])
    ax4.set_ylim([-max_reach, max_reach])
    ax4.set_zlim([0, max_reach])
    ax4.view_init(elev=25, azim=25, roll=0)
    ax4.legend()

    # Isometric Dog view (azim=90)
    ax5 = fig.add_subplot(2,3,5, projection='3d')
    ax5.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax5.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax5.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax5.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax5.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax5.set_xlabel('X (cm)')
    ax5.set_ylabel('Y (cm)')
    ax5.set_zlabel('Z (cm)')
    ax5.set_title('Dog Leg View')
    ax5.set_box_aspect([1,1,1])
    ax5.set_xlim([-max_reach, max_reach])
    ax5.set_ylim([-max_reach, max_reach])
    ax5.set_zlim([0, max_reach])
    ax5.view_init(elev=-27, azim=-36, roll=-80) #0,0,-90
    ax5.view_init(elev=-34, azim=-46, roll=-60) #0,0,-90

    ax5.legend()

    # Dog plane view (azim=90)
    ax6 = fig.add_subplot(2,3,6, projection='3d')
    ax6.plot([O[0], J1[0]], [O[1], J1[1]], [O[2], J1[2]], 'k-', lw=4, label='L1')
    ax6.plot([J1[0], J_const[0]], [J1[1], J_const[1]], [J1[2], J_const[2]], 'y-', lw=4, label='Lc')
    ax6.plot([J_const[0], J2[0]], [J_const[1], J2[1]], [J_const[2], J2[2]], 'b-', lw=4, label='L2')
    ax6.plot([J2[0], J3[0]], [J2[1], J3[1]], [J2[2], J3[2]], 'r-', lw=4, label='L3')
    ax6.scatter([O[0], J1[0], J_const[0], J2[0], J3[0]], 
                [O[1], J1[1], J_const[1], J2[1], J3[1]], 
                [O[2], J1[2], J_const[2], J2[2], J3[2]], color='k', s=50)
    ax6.set_xlabel('X (cm)')
    ax6.set_ylabel('Y (cm)')
    ax6.set_zlabel('Z (cm)')
    ax6.set_title('Dog Plane View')
    ax6.set_box_aspect([1,1,1])
    ax6.set_xlim([-max_reach, max_reach])
    ax6.set_ylim([-max_reach, max_reach])
    ax6.set_zlim([0, max_reach])
    ax6.view_init(elev=-2, azim=0, roll=-90) #0,0,-90
    ax6.legend()

    plt.tight_layout()
    plt.show()

def top_circle_view(x, y, T1, T2, home_linkConst, linkConst, b, a, theta1):
    # # Plot the points, lines, and circles
    fig, ax = plt.subplots()

    # Plot linkConst circle
    circle = plt.Circle((0, 0), linkConst, color='b', fill=False, linestyle='--', label='linkConst circle')
    ax.add_artist(circle)

    def rotate_90ccw(x, y):
        return y, -x

    # Apply to target
    x_rot, y_rot = rotate_90ccw(x, y)

    # Apply to T1, T2, JC (home_linkConst)
    T1_rot = rotate_90ccw(*T1)
    T2_rot = rotate_90ccw(*T2)
    JC_rot = rotate_90ccw(home_linkConst[0], home_linkConst[1])

    # When plotting
    ax.plot(x_rot, y_rot, 'ro', label='Target (x,y)')
    circle = plt.Circle((0, 0), linkConst, color='b', fill=False, linestyle='--', label='linkConst circle')
    ax.add_artist(circle)
    ax.plot(*T1_rot, 'go', label='T1')
    ax.plot(*T2_rot, 'mo', label='T2')
    ax.plot(*JC_rot, 'bo', label='JC')

    # Lines (rotate both endpoints)
    ax.plot([0, T1_rot[0]], [0, T1_rot[1]], 'g-', label='Line to T1')
    ax.plot([0, T2_rot[0]], [0, T2_rot[1]], 'm-', label='Line to T2')
    ax.plot([0, x_rot], [0, y_rot], 'r-', label='Origin to Target')
    ax.plot([T1_rot[0], x_rot], [T1_rot[1], y_rot], 'b--', label='Tangent to Target')


    # Annotate - rotate label positions similarly
    ax.annotate(f'a={np.degrees(a):.1f}°', xy=(x_rot / 2, y_rot / 2), xytext=(x_rot / 2 + 0.5, y_rot / 2 + 0.5))
    ax.annotate(f'b={np.degrees(b):.1f}°', xy=T1_rot, xytext=(T1_rot[0] + 0.5, T1_rot[1] + 0.5))
    mid_x, mid_y = (T1[0]/2 + x/2, T1[1]/2 + y/2)
    mid_x_rot, mid_y_rot = rotate_90ccw(mid_x, mid_y)
    ax.annotate(f'theta1={np.degrees(theta1):.1f}°', xy=(mid_x_rot, mid_y_rot), xytext=(mid_x_rot + 0.5, mid_y_rot + 0.5))

    # Set plot limits and aspect ratio
    ax.set_aspect('equal', adjustable='box')
    # ax.set_xlim(-20, 40)
    # ax.set_ylim(-20, 20)
    ax.set_title('Inverse Kinematics Visualization')
    ax.legend()
    plt.grid(True)
    plt.show()