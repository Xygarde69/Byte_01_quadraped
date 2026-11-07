# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  multiprocess_code/all_4_legs/cur_test/BYTE_VISION/udp_receiver.py
# STATUS    ·  PROTOTYPE · vision (BYTE_VISION)
# ────────────────────────────────────────────────────────────────────────
# Debug listener: prints 'Target Lock' packets received on UDP
# 0.0.0.0:5005.
# ────────────────────────────────────────────────────────────────────────
# RUN       ·  python3 udp_receiver.py
# ════════════════════════════════════════════════════════════════════════
import socket
import json

UDP_IP = "0.0.0.0" # Listen on all network interfaces
UDP_PORT = 5005

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind((UDP_IP, UDP_PORT))

print(f"Listening for Target Lock data on Port {UDP_PORT}...")
print("-" * 50)

while True:
    try:
        # Buffer size is 1024 bytes
        data, addr = sock.recvfrom(1024) 
        msg = json.loads(data.decode('utf-8'))
        
        error_x = msg.get("error_x", 0)
        
        # Simple text visualization of the data
        if error_x < -20:
            direction = "<<< MOVE LEFT "
        elif error_x > 20:
            direction = " MOVE RIGHT >>>"
        else:
            direction = "=== LOCKED ==="
            
        print(f"Received Error: {error_x:+6.2f} px | Action: {direction}")
        
    except KeyboardInterrupt:
        print("\nReceiver stopped.")
        break
    except Exception as e:
        print(f"Error parsing packet: {e}")