# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/dual_motor_simple.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Simple dual-motor script: drives motor 1 and motor 2 simultaneously.
# ════════════════════════════════════════════════════════════════════════
"""
Simple Dual Motor Control Script
Controls Motor 1 (ID=1) and Motor 2 (ID=2) simultaneously
"""

from ak60_v3_control import AK60V3Motor
import time

def main():
    print("="*60)
    print("  Dual Motor Control - Simple Example")
    print("="*60)
    
    # STEP 1: Create motor objects
    print("\n1. Initializing motors...")
    motor1 = AK60V3Motor(motor_id=2, can_interface='can0')  # Motor ID = 1
    motor2 = AK60V3Motor(motor_id=3, can_interface='can0')  # Motor ID = 2
    print("   ✓ Both motors initialized")
    
    try:
        # STEP 2: Enable both motors
        print("\n2. Enabling motors...")
        motor1.enable()
        motor2.enable()
        time.sleep(1)
        print("   ✓ Both motors enabled")
        
        # STEP 3: Move both motors to same position
        print("\n3. Moving both motors to 1.57 rad (90 degrees)...")
        print("   Control parameters: Kp=50, Kd=2, Torque=0")
        print("\n   Press Ctrl+C to stop\n")
        
        for i in range(500):  # Run for 3 seconds (150 * 0.02s)
            # Send command to Motor 1
            motor1.send_mit_command(
                position=0,    # 90 degrees in radians
                velocity=0,
                kp=50,            # Position gain
                kd=4,             # Damping gain
                torque=0          # No feedforward torque
            )
            
            # Send command to Motor 2 (same parameters)
            motor2.send_mit_command(
                position=-3,    # Same position as Motor 1
                velocity=0,
                kp=5,
                kd=0.5,
                torque=10
            )
            
            # Read feedback from both motors
            fb1_ok = motor1.read_feedback(timeout=0.01)
            fb2_ok = motor2.read_feedback(timeout=0.01)
            
            # Display feedback
            if fb1_ok and fb2_ok:
                fb1 = motor1.get_feedback()
                fb2 = motor2.get_feedback()
                
                print(f"   Motor1: {fb1['position']:+.3f} rad | "
                      f"Motor2: {fb2['position']:+.3f} rad | "
                      f"Temps: {fb1['temperature']}°C / {fb2['temperature']}°C",
                      end='\r')
            
            time.sleep(0.02)  # 50Hz control rate
        
        print("\n\n4. Returning to zero position...")
        
        # Return both motors to zero
        for i in range(100):
            motor1.send_mit_command(position=0, velocity=0, kp=6, kd=2, torque=0)
            motor2.send_mit_command(position=0, velocity=0, kp=50, kd=20, torque=10)
            time.sleep(0.02)
        
        print("   ✓ Motors returned to zero")
        
        # STEP 4: Disable motors
        print("\n5. Disabling motors...")
        motor1.disable()
        motor2.disable()
        print("   ✓ Motors disabled")
        
    except KeyboardInterrupt:
        print("\n\n⚠ Stopped by user (Ctrl+C)")
        motor1.disable()
        motor2.disable()
        
    except Exception as e:
        print(f"\n✗ Error: {e}")
        motor1.disable()
        motor2.disable()
        
    finally:
        # STEP 5: Close connections
        print("\n6. Closing connections...")
        motor1.close()
        motor2.close()
        print("   ✓ Done!\n")

if __name__ == "__main__":
    main()
