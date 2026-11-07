# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/03_single_leg_ik/main_ik_runtime/stand_up.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Stand-up sender for one leg (leg_side = 'left'/'right').
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

leg_side = 'left'  # 'left' or 'right'

# Link lengths in cm
L1 = 5.995  # Link 1 length
linkConst = -9.094 if leg_side == 'right' else 9.094  # Constant link between L1 and L2 (RADIUS OF CIRCLE WHEN L1 IS ROTATED ALONG Z AXIS)
L2 = 22  # Link 2 length
L3 = 21.5  # Link 3 length
 
reference_angles = np.array([0, 0, 0])
last_angles = np.array([0, 0, 0])
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

def update(start_at, total_time, interval_time):
    global ref_updated
    global reference_angles
    while time.time() - start_at < total_time:
        cur_time = time.time() - start_at
        x,y,z = anim.get_position(t=cur_time, total_time=total_time)
        print(f"Time: {cur_time:.2f}s, Target: ({x:.2f}, {y:.2f}, {z:.2f})")
        theta1, theta2, theta3 = ik.inverse_kinematics(x, y, z, L1, linkConst, L2, L3) 
        max_time = interval_time/1000.0  # Initially given delay
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
            d1 = abs(np.degrees(theta1) - last_angles[1])
            d2 = abs(np.degrees(theta2) - last_angles[0])
            d3 = abs(np.degrees(theta3) - last_angles[2]) 
            last_angles[:] = np.array([np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)])
            # print(np.degrees(theta1), np.degrees(theta2), np.degrees(theta3))
            # destinations = np.array([np.degrees(theta1), np.degrees(theta2), np.degrees(theta3)])

            deg1 = np.degrees(theta1)
            deg2 = np.degrees(theta2)
            deg3 = np.degrees(theta3)

            deg1 = deadband(wrap_to_180(deg1))
            deg2 = deadband(wrap_to_180(deg2))
            deg3 = deadband(wrap_to_180(deg3))

            destinations = np.array([deg1, deg2, deg3])


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

        destinations = np.array([0.0, 0.0, 0.0])  # Use np.array from start
        print(f"📤 First Message Sending: {destinations.tolist()}°") 
        if send_array_to_motor(destinations, HOST, PORT):
            print(f"✅ First Message Sent: {destinations.tolist()}°")
        else:
            print("⚠️  First Message Send failed - receiver may have disconnected")
            exit(1)

        while True: 
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