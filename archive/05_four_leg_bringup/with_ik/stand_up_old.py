# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/with_ik/stand_up_old.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Previous revision of stand_up.py.
# ════════════════════════════════════════════════════════════════════════
"""
Socket-based Sender for Motor Controller
Sends numpy arrays to receiver via TCP socket
"""

import numpy as np
import socket
import pickle
import time

import matplotlib.pyplot as plt
import matplotlib.animation as animation

import stand_up_animation as anim
import _3dof_ik_with_shift_circle_method as ik 


# Link lengths in cm
L1 = 5.995                  # Link 1 length

# Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
right_linkConst = -9.094    # for right leg
left_linkConst = 9.094      # for left leg, opposite sign to right leg

L2 = 22     # Link 2 length
L3 = 21.5   # Link 3 length

right_reference_angles = np.array([0, 0, 0])
left_reference_angles = np.array([0, 0, 0])

right_last_angles = np.array([0, 0, 0])
left_last_angles = np.array([0, 0, 0])
ref_updated = False

def wrap_to_180(angle_deg):
    # Wrap to [-180, 180)
    angle = (angle_deg + 180) % 360 - 180
    return angle

def deadband(angle_deg, eps=1e-2):
    return 0.0 if abs(angle_deg) < eps else angle_deg

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

def update(start_at, total_time, interval_time):
    global ref_updated
    global right_reference_angles, left_reference_angles

    # set right and left reference angles only once at the start of the animation
    right_reference_angles[:] = np.array([ik.inverse_kinematics(x, y, z, L1, left_linkConst, L2, L3) for x,y,z in [anim.get_position(t=0, total_time=total_time, leg='right')]])
    left_reference_angles[:] = np.array([ik.inverse_kinematics(x, y, z, L1, left_linkConst, L2, L3) for x,y,z in [anim.get_position(t=0, total_time=total_time, leg='left')]])
    ref_updated = True
    print("Reference angles set to:", np.degrees(left_reference_angles), np.degrees(right_reference_angles))
    
    while time.time() - start_at < total_time:
        cur_time = time.time() - start_at

        x,y,z = anim.get_position(t=cur_time, total_time=total_time, leg='left')
        print(f"Time: {cur_time:.2f}s, Target: ({x:.2f}, {y:.2f}, {z:.2f})") 
        left_theta1, left_theta2, left_theta3 =  ik.inverse_kinematics(x, y, z, L1, left_linkConst, L2, L3)

        x,y,z = anim.get_position(t=cur_time, total_time=total_time, leg='right')
        print(f"Time: {cur_time:.2f}s, Target: ({x:.2f}, {y:.2f}, {z:.2f})")
        right_theta1, right_theta2, right_theta3 = ik.inverse_kinematics(x, y, z, L1, right_linkConst, L2, L3)

        max_time = interval_time/1000.0  # Initially given delay 
        # print(np.degrees(right_theta1), np.degrees(right_theta2), np.degrees(right_theta3))
        right_theta1 = right_theta1 - right_reference_angles[0]
        right_theta2 = right_theta2 - right_reference_angles[1]
        right_theta3 = right_theta3 - right_reference_angles[2]

        left_theta1 = left_theta1 - left_reference_angles[0]
        left_theta2 = left_theta2 - left_reference_angles[1]
        left_theta3 = left_theta3 - left_reference_angles[2]

        # abs_diff
        # d1 = abs(np.degrees(right_theta1) - right_last_angles[1])
        # d2 = abs(np.degrees(right_theta2) - right_last_angles[0])
        # d3 = abs(np.degrees(right_theta3) - right_last_angles[2]) 

        right_last_angles[:] = np.array([np.degrees(right_theta1), np.degrees(right_theta2), np.degrees(right_theta3)])
        left_last_angles[:] = np.array([np.degrees(left_theta1), np.degrees(left_theta2), np.degrees(left_theta3)])

        r_deg1 = deadband(wrap_to_180(np.degrees(right_theta1)))
        r_deg2 = deadband(wrap_to_180(np.degrees(right_theta2)))
        r_deg3 = deadband(wrap_to_180(np.degrees(right_theta3)))
        
        l_deg1 = deadband(wrap_to_180(np.degrees(left_theta1)))
        l_deg2 = deadband(wrap_to_180(np.degrees(left_theta2)))
        l_deg3 = deadband(wrap_to_180(np.degrees(left_theta3)))

        # print(np.degrees(right_theta1), np.degrees(right_theta2), np.degrees(right_theta3))
        destinations = np.array([l_deg1, l_deg2, l_deg3, r_deg1, r_deg2, r_deg3]) 
        # destinations = np.array([l_deg1, l_deg2, l_deg3])
        if send_array_to_motor(destinations, HOST, PORT):
            print(f"✅ Sent: {destinations.tolist()}°")
        else:
            print(f"📤 Send failed - {destinations.tolist()}°") 
            print("⚠️  Send failed - receiver may have disconnected")

        print(f"Waiting for {max_time:.2f} s before next command...")
        time.sleep(max_time)  # In seconds

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

        destinations = np.array([[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]])  # Use np.array from start
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

            start_at = time.time()
            total_time = 4.0 # seconds
            interval_time = 10  # milliseconds

            update(start_at = start_at, total_time=total_time, interval_time=interval_time)
                
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