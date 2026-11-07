# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/stand_up.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Stand-up sequence: IK + S-curve timing, sends arrays over the socket
# to the motor controller.
# ════════════════════════════════════════════════════════════════════════
"""
Socket-based Sender for Motor Controller
Sends numpy arrays to receiver via TCP socket
"""

import numpy as np
import socket
import pickle
import time

import _3dof_ik_with_shift_circle_method as ik 
import matplotlib.pyplot as plt
import matplotlib.animation as animation

# S-curve timing parameters
BASEANGLE = 360.0
BASETIME = 3.0
MINTIME = 0.08
SMALLANGLETHRESH = 5.0

# Link lengths in cm
L1 = 5.995  # Link 1 length
linkConst = -9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length

# Trajectory parameters
x = -9.094
z = 6 # right to left co-ordinates of bots leg for movement
y_start, y_end = 20, 42 # max up and max low of bots leg
y_amplitude = y_start - y_end  # max amplitude of sine wave for movement
num_points = 5  # For smoothness

# Prepare trajectory points
ys = np.linspace(y_start, y_end, num_points) # Interpolate y from start to end (top to bottom)
ys_rev = ys[::-1]

ys = np.concatenate((ys, ys_rev))

coords = [(x, y, z) for y in ys] # Build the trajectory points



print("Length: ", len(coords))

reference_angles = np.array([0, 0, 0])
last_angles = np.array([0, 0, 0])
ref_updated = False

def send_array_to_motor(array, host='127.0.0.1', port=50000, timeout=2.0):
    """
    Send numpy array to motor controller via socket
    
    Returns:
        bool: True if successful, False otherwise
    """
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(timeout)
        sock.connect((host, port))
        
        # Serialize and send array
        data = pickle.dumps(array)
        sock.sendall(data)
        sock.close()
        
        return True
        
    except ConnectionRefusedError:
        print("❌ Connection refused - Is receiver running?")
        return False
    except socket.timeout:
        print("❌ Connection timeout")
        return False
    except Exception as e:
        print(f"❌ Send error: {e}")
        return False


def test_connection(host='127.0.0.1', port=50000):
    """Test if receiver is available"""
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(1.0)
        sock.connect((host, port))
        sock.close()
        return True
    except:
        return False


def on_complete(anim):
    plt.close(fig)
    print("Animation finished and program ended.")


def update(coords, interval=10):
    global ref_updated
    global reference_angles
    for coord in coords:
        x, y, z = coord
        theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
        max_time = interval/1000.0  # Initially given delay
        if not ref_updated:
            reference_angles[:] = np.array([theta1, theta2, theta3])
            ref_updated = True
            print("Reference angles set to:", np.degrees(reference_angles))
        else:
            # print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
            theta1 = theta1 - reference_angles[0]
            theta2 = theta2 - reference_angles[1]
            theta3 = theta3 - reference_angles[2]
            # abs_diff
            e1 = abs(np.degrees(theta1) - last_angles[1])
            e2 = abs(np.degrees(theta2) - last_angles[0])
            e3 = abs(np.degrees(theta3) - last_angles[2])
            max_time = max(e1,e2,e3) / (min(BASEANGLE/BASETIME, MINTIME))  # Assume max speed 180 deg/s
            max_time /= 1000.0
            last_angles[:] = np.array([np.degrees(theta2), np.degrees(theta1), np.degrees(theta3)])
            # print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
            destinations = np.array([np.degrees(theta2), np.degrees(theta1), np.degrees(theta3)])
            if send_array_to_motor(destinations, HOST, PORT):
                print(f"✅ Sent: {destinations.tolist()}°")
            else:
                print(f"📤 Send failed - {destinations.tolist()}°") 
                print("⚠️  Send failed - receiver may have disconnected")
        print(f"Waiting for {max_time:.2f} s before next command...")
        time.sleep(max_time)  # In seconds

def anim_update(num):
    global ref_updated
    global reference_angles
    ax.cla()
    x, y, z = coords[num]
    theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3)
    if not ref_updated:
        reference_angles[:] = np.array([theta1, theta2, theta3])
        ref_updated = True
        print("Reference angles set to:", np.degrees(reference_angles))
    else:
        ik.visualize.animate_3dof(L1, linkConst, L2, L3, theta1, theta2, theta3, ax=ax)
        # print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
        theta1 = theta1 - reference_angles[0]
        theta2 = theta2 - reference_angles[1]
        theta3 = theta3 - reference_angles[2]
        # print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
        destinations = np.array([np.degrees(theta2), np.degrees(theta1), np.degrees(theta3)])
        print(f"📤 Sending: {destinations.tolist()}°") 
        if send_array_to_motor(destinations, HOST, PORT):
            print(f"✅ Sent: {destinations.tolist()}°")
        else:
            print("⚠️  Send failed - receiver may have disconnected")

        ax.set_title(f'Pose {num+1}')



if __name__ == '__main__':
    HOST = '127.0.0.1'
    PORT = 50000
    
    print("="*70)
    print("Motor Controller Socket Sender")
    print("="*70)

    try: 
        # Test connection
        print(f"Testing connection to {HOST}:{PORT}...", end=" ")
        if test_connection(HOST, PORT):
            print("✅ Connected!")
        else:
            print("❌ Failed!")
            print("\n⚠️  Make sure motor_controller_receiver.py is running first!")
            print("   Start it in another terminal and try again.")
            
            retry = input("\nRetry connection? (y/n): ").strip().lower()
            if retry != 'y':
                exit(1)
            
            if not test_connection(HOST, PORT):
                print("❌ Still cannot connect. Exiting.")
                exit(1)
            print("✅ Connected on retry!")

        destinations = np.array([0.0, 0.0, 0.0])  # Use np.array from start
        print(f"📤 First Message Sending: {destinations.tolist()}°") 
        if send_array_to_motor(destinations, HOST, PORT):
            print(f"✅ First Message Sent: {destinations.tolist()}°")
        else:
            print("⚠️  First Message Send failed - receiver may have disconnected")
            exit(1)

        animation_mode = False

        while True:

            fig = plt.figure(figsize=(8,6))
            ax = fig.add_subplot(111, projection='3d')

            # Reset for new run
            ref_updated = False

            if animation_mode:
                # Create and run animation
                ani = animation.FuncAnimation(fig, anim_update, frames=len(coords), interval=10, repeat=False)
                plt.show(block=False)
                
                # Wait for animation to complete (rough timing)
                plt.pause(len(coords) * 0.4)
                
                # Close window
                plt.close(fig)
            else:
                update(coords = coords, interval=300)
            # Ask user
            rerun = input("\nRun animation again? (y/n): ").lower().strip()
            if rerun not in ['y', 'yes']:
                print("Program ended.")
                break
    except KeyboardInterrupt:
        print("\nProgram interrupted by user. Exiting...")
    except Exception as e:
        print(f"\n❌ An error occurred: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Sender closed.")