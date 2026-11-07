#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/02_multi_motor_socket_control/motor_controller_sender.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# send_array_to_motor(): sends numpy arrays to the receiver via TCP
# (default 127.0.0.1:50000) - an early form of the socket protocol the
# robot still uses.
# ════════════════════════════════════════════════════════════════════════
"""
Socket-based Sender for Motor Controller
Sends numpy arrays to receiver via TCP socket
"""

import numpy as np
import socket
import pickle
import time


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


if __name__ == '__main__':
    HOST = '127.0.0.1'
    PORT = 50000
    
    print("="*70)
    print("Motor Controller Socket Sender")
    print("="*70)
    
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

    print("\n📋 First enter Motor ID, then enter angle to be sent, then enter y to sent or n to continue.")
    print("   Current targets:", destinations.tolist())
    
    while True:
        

        motor_id = input("Enter Motor ID (1/2/3) (or 'q' to quit): ").strip()
        if motor_id.lower() == 'q':
            print("Exiting sender...")
            break

        if motor_id not in ['1', '2', '3']:
            print("❌ Invalid Motor ID. Please enter 1, 2, or 3.")
            continue

        try:
            motor_id = int(motor_id)
        except ValueError:
            print("❌ Invalid Motor ID. Please enter a numeric value.")
            continue

        angle_str = input("Enter angle in degrees(eg. 50.5) (or 'q' to quit): ").strip()
        if angle_str.lower() == 'q':
            print("Exiting sender...")
            break

        try:
            angle = float(angle_str)
            destinations[motor_id-1] = angle

            print(f"✅ Motor {motor_id} set to {angle:.1f}°")
            print(f"   Current targets: {destinations.tolist()}°")

        except ValueError:
            print("❌ Invalid angle. Please enter a numeric value.")
            continue
 

        send_cmd = input("Send command? (y/n): ").strip().lower()
        if send_cmd == 'n':
            continue
        elif send_cmd == 'y':
            print(f"📤 Sending: {destinations.tolist()}°") 
            if send_array_to_motor(destinations, HOST, PORT):
                print(f"✅ Sent: {destinations.tolist()}°")
            else:
                print("⚠️  Send failed - receiver may have disconnected")

        else:
            print("❌ Invalid input. Please enter 'y' or 'n'.")


    print("Sender closed.")


    
    # # Predefined destinations
    # destinations = {
    #     '1': np.array([0.0,0.0,0.0]),
    #     '2': np.array([40.0,20.0,30.0]),
    #     '3': np.array([60.0,30.0,40.0]),
    #     '4': np.array([-40.0,-20.0,-30.0]),
    #     '5': np.array([-60.0,-30.0,-40.0])
    # }
    
    # print("\n📋 Available commands:")
    # print("  1 - Move to [0°, 0°, 0°] (home)")
    # print("  2 - Move to [40°, 20°, 30°]")
    # print("  3 - Move to [60°, 30°, 40°]")
    # print("  4 - Move to [-40°, -20°, -30°]")
    # print("  5 - Move to [-60°, -30°, -40°]")
    # print("  q - Quit")
    # print()
    
    # while True:
    #     cmd = input("Cmd (1/2/3/q): ").strip()
        
    #     if cmd.lower() == 'q':
    #         print("Exiting sender...")
    #         break
            
    #     if cmd in destinations:
    #         array = destinations[cmd]
            
    #         if send_array_to_motor(array, HOST, PORT):
    #             print(f"✅ Sent: {array.tolist()}°")
    #         else:
    #             print("⚠️  Send failed - receiver may have disconnected")
                
    #     else:
    #         print("❌ Invalid command. Use 1/2/3/q")
    
    # print("Sender closed.")
