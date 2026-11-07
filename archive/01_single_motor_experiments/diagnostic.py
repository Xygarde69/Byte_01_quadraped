#!/usr/bin/env python3
# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/01_single_motor_experiments/diagnostic.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# AK60-6 V3.0 + Waveshare CAN HAT diagnostic: checks every aspect of the
# CAN set-up (interface, bitrate, motor response). Same file as
# archive/02.../diagnostic.py.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 + Waveshare CAN HAT Diagnostic Script
Checks all aspects of your CAN setup
"""

import subprocess
import sys
import time

def print_header(text):
    """Print formatted header"""
    print("\n" + "=" * 60)
    print(f"  {text}")
    print("=" * 60)

def run_command(cmd, check=True):
    """Run shell command and return output"""
    try:
        result = subprocess.run(
            cmd, 
            shell=True, 
            capture_output=True, 
            text=True,
            check=check
        )
        return result.stdout, result.stderr, result.returncode
    except subprocess.CalledProcessError as e:
        return e.stdout, e.stderr, e.returncode

def check_system_info():
    """Check Ubuntu and hardware info"""
    print_header("1. System Information")
    
    stdout, _, _ = run_command("lsb_release -a", check=False)
    print(stdout)
    
    stdout, _, _ = run_command("uname -m")
    print(f"Architecture: {stdout.strip()}")
    
    stdout, _, _ = run_command("cat /proc/device-tree/model", check=False)
    print(f"Hardware: {stdout.strip()}")

def check_spi():
    """Check if SPI is enabled"""
    print_header("2. SPI Interface Check")
    
    stdout, _, returncode = run_command("lsmod | grep spi_bcm2835")
    if returncode == 0:
        print("✓ SPI driver loaded (spi_bcm2835)")
    else:
        print("✗ SPI driver NOT loaded")
        print("  Fix: Add 'dtparam=spi=on' to /boot/firmware/config.txt")
        return False
    
    stdout, _, returncode = run_command("ls /dev/spidev0.*", check=False)
    if returncode == 0:
        print(f"✓ SPI devices found:\n{stdout}")
    else:
        print("✗ No SPI devices found in /dev/")
        return False
    
    return True

def check_can_overlay():
    """Check if MCP2515 overlay is loaded"""
    print_header("3. MCP2515 CAN Controller Check")
    
    stdout, _, _ = run_command("dmesg | grep -i mcp2515", check=False)
    if "mcp2515" in stdout.lower() and "successfully initialized" in stdout.lower():
        print("✓ MCP2515 initialized successfully")
        print(f"  Details:\n{stdout[-500:]}")  # Last 500 chars
        return True
    else:
        print("✗ MCP2515 NOT found in kernel messages")
        print("  Fix: Add to /boot/firmware/config.txt:")
        print("       dtoverlay=mcp2515-can0,oscillator=12000000,interrupt=25")
        if stdout:
            print(f"  dmesg output:\n{stdout[-500:]}")
        return False

def check_can_interface():
    """Check if can0 interface exists and is UP"""
    print_header("4. CAN Interface Status")
    
    stdout, _, returncode = run_command("ip link show can0", check=False)
    if returncode != 0:
        print("✗ can0 interface NOT found")
        print("  Fix: Run setup script or manually:")
        print("       sudo ip link set can0 type can bitrate 1000000")
        print("       sudo ip link set can0 up")
        return False
    
    print("✓ can0 interface exists")
    print(stdout)
    
    if "UP" in stdout and "RUNNING" in stdout:
        print("✓ can0 is UP and RUNNING")
    elif "UP" in stdout:
        print("⚠ can0 is UP but not RUNNING (no CAN traffic yet)")
    else:
        print("✗ can0 is DOWN")
        print("  Fix: sudo ip link set can0 up type can bitrate 1000000")
        return False
    
    # Check bitrate
    stdout, _, _ = run_command("ip -details link show can0", check=False)
    if "bitrate 1000000" in stdout:
        print("✓ Bitrate correctly set to 1 Mbps")
    else:
        print("✗ Bitrate NOT set to 1 Mbps")
        print("  Fix: sudo ip link set can0 type can bitrate 1000000")
        return False
    
    return True

def check_can_utils():
    """Check if can-utils is installed"""
    print_header("5. CAN Utilities Check")
    
    tools = ['cansend', 'candump', 'cansequence']
    all_ok = True
    
    for tool in tools:
        stdout, _, returncode = run_command(f"which {tool}", check=False)
        if returncode == 0:
            print(f"✓ {tool} found at {stdout.strip()}")
        else:
            print(f"✗ {tool} NOT found")
            all_ok = False
    
    if not all_ok:
        print("  Fix: sudo apt install can-utils")
    
    return all_ok

def check_python_can():
    """Check if python-can is installed"""
    print_header("6. Python CAN Library Check")
    
    try:
        import can
        print(f"✓ python-can installed (version {can.__version__})")
        
        # Test socketcan interface
        try:
            bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
            print("✓ Successfully created CAN bus object")
            bus.shutdown()
        except Exception as e:
            print(f"✗ Failed to create CAN bus: {e}")
            return False
        
        return True
    except ImportError:
        print("✗ python-can NOT installed")
        print("  Fix: pip3 install python-can")
        return False

def check_can_statistics():
    """Check CAN interface statistics"""
    print_header("7. CAN Interface Statistics")
    
    stdout, _, returncode = run_command("ip -statistics link show can0", check=False)
    if returncode == 0:
        print(stdout)
        
        # Parse statistics
        if "RX: bytes" in stdout:
            lines = stdout.split('\n')
            for line in lines:
                if 'errors' in line.lower() or 'dropped' in line.lower():
                    if any(x in line for x in ['errors 0', 'dropped 0']):
                        continue
                    else:
                        print("⚠ Warning: CAN errors detected!")
        
        return True
    else:
        print("✗ Could not get statistics")
        return False

def test_can_loopback():
    """Test CAN with loopback message"""
    print_header("8. CAN Loopback Test")
    
    print("This test sends a CAN message and listens for it.")
    print("Note: This may not show anything if motor is not responding.")
    
    try:
        import can
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        
        # Send test message (extended frame)
        test_msg = can.Message(
            arbitration_id=0x00000501,  # Enable motor command, ID=1
            data=[0x00],
            is_extended_id=True
        )
        
        print(f"\nSending test message: ID=0x{test_msg.arbitration_id:08X}, Data={test_msg.data.hex()}")
        bus.send(test_msg)
        print("✓ Message sent successfully")
        
        # Try to receive (timeout 1 second)
        print("\nListening for response (1 second)...")
        msg = bus.recv(timeout=1.0)
        
        if msg:
            print(f"✓ Received CAN message!")
            print(f"  ID: 0x{msg.arbitration_id:08X} ({'Extended' if msg.is_extended_id else 'Standard'})")
            print(f"  Data: {msg.data.hex()}")
            print(f"  Length: {len(msg.data)} bytes")
        else:
            print("⚠ No response received within 1 second")
            print("  This is normal if motor is:")
            print("  - Not powered")
            print("  - In query-reply mode (not periodic feedback)")
            print("  - Wrong CAN ID")
        
        bus.shutdown()
        return True
        
    except Exception as e:
        print(f"✗ Test failed: {e}")
        return False

def check_motor_feedback():
    """Listen for motor feedback messages"""
    print_header("9. Motor Feedback Check")
    
    print("Listening for motor feedback for 3 seconds...")
    print("Motor must be:")
    print("  - Powered (18-52V)")
    print("  - Configured for 'Periodic Feedback' mode")
    print("  - Connected to CAN0 on Waveshare HAT")
    
    try:
        import can
        bus = can.interface.Bus(channel='can0', bustype='socketcan', bitrate=1000000)
        
        start_time = time.time()
        message_count = 0
        motor_ids = set()
        
        while time.time() - start_time < 3.0:
            msg = bus.recv(timeout=0.1)
            if msg:
                message_count += 1
                motor_id = msg.arbitration_id & 0xFF
                motor_ids.add(motor_id)
                
                if message_count == 1:  # Print first message
                    print(f"\n✓ Motor feedback detected!")
                    print(f"  ID: 0x{msg.arbitration_id:08X}")
                    print(f"  Motor ID: {motor_id}")
                    print(f"  Data: {msg.data.hex()}")
        
        bus.shutdown()
        
        if message_count > 0:
            print(f"\n✓ Received {message_count} messages from motor(s): {list(motor_ids)}")
            return True
        else:
            print("\n⚠ No motor feedback received")
            print("  Possible causes:")
            print("  - Motor not powered")
            print("  - Motor in query-reply mode (not periodic)")
            print("  - CAN wiring issue")
            print("  - Wrong bitrate")
            return False
            
    except Exception as e:
        print(f"✗ Error: {e}")
        return False

def print_summary(results):
    """Print summary of all checks"""
    print_header("DIAGNOSTIC SUMMARY")
    
    checks = [
        ("SPI Interface", results.get('spi', False)),
        ("MCP2515 Controller", results.get('mcp2515', False)),
        ("CAN Interface", results.get('can_interface', False)),
        ("CAN Utils", results.get('can_utils', False)),
        ("Python CAN", results.get('python_can', False)),
        ("CAN Statistics", results.get('statistics', False)),
        ("CAN Loopback", results.get('loopback', False)),
        ("Motor Feedback", results.get('motor', False)),
    ]
    
    passed = sum(1 for _, status in checks if status)
    total = len(checks)
    
    for name, status in checks:
        symbol = "✓" if status else "✗"
        print(f"  {symbol} {name}")
    
    print(f"\nResult: {passed}/{total} checks passed")
    
    if passed == total:
        print("\n🎉 ALL CHECKS PASSED! Your system is ready!")
        print("\nNext steps:")
        print("  1. Save the Python library: ak60_v3_control.py")
        print("  2. Run: python3 ak60_v3_control.py")
    else:
        print("\n⚠ Some checks failed. Review the errors above.")
        print("\nQuick fixes:")
        print("  • SPI/MCP2515 issues: Edit /boot/firmware/config.txt")
        print("  • CAN interface down: Run CAN setup script")
        print("  • Missing tools: sudo apt install can-utils")
        print("  • Python library: pip3 install python-can")
        print("  • Motor issues: Check power, wiring, and CubeMars config")

def main():
    """Run all diagnostics"""
    print("""
╔════════════════════════════════════════════════════════════╗
║   AK60-6 V3.0 + Waveshare CAN HAT Diagnostic Tool         ║
║   Checks: Ubuntu, SPI, MCP2515, CAN, Python, Motor        ║
╚════════════════════════════════════════════════════════════╝
""")
    
    if subprocess.os.geteuid() != 0:
        print("⚠ Warning: Not running as root. Some checks may require sudo.\n")
    
    results = {}
    
    check_system_info()
    results['spi'] = check_spi()
    results['mcp2515'] = check_can_overlay()
    results['can_interface'] = check_can_interface()
    results['can_utils'] = check_can_utils()
    results['python_can'] = check_python_can()
    results['statistics'] = check_can_statistics()
    results['loopback'] = test_can_loopback()
    results['motor'] = check_motor_feedback()
    
    print_summary(results)
    
    print("\nFor detailed setup instructions, see the complete guide.")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted by user.")
        sys.exit(0)
