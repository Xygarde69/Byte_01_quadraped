# ════════════════════════════════════════════════════════════════════════
# REPO MAP  ·  archive/05_four_leg_bringup/final/ak60_v3_control.py
# STATUS    ·  ARCHIVED · early project history, not used by robot/
# ────────────────────────────────────────────────────────────────────────
# Driver revision used by the first complete 4-leg program.
# ════════════════════════════════════════════════════════════════════════
"""
AK60-6 V3.0 Motor CAN Control Library for Raspberry Pi
Compatible with Extended CAN Frames (29-bit) and MCP2515
Based on CubeMars V3.0 Protocol Documentation
MODIFIED: Supports shared CAN bus for multiple motors
"""

import can
import struct
import time

class AK60V3Motor:
    """
    AK60-6 V3.0 Motor Controller for MIT Mode
    Uses 29-bit Extended CAN frames
    """
    
    # CAN ID Structure for V3.0 (29-bit Extended Frame)
    # Bits [7:0]   = Motor ID (Driver ID)
    # Bits [28:8]  = Control Mode ID
    
    # Control Mode IDs
    MODE_DUTY_CYCLE = 0x00
    MODE_CURRENT = 0x01
    MODE_CURRENT_BRAKE = 0x02
    MODE_VELOCITY = 0x03
    MODE_POSITION = 0x04
    MODE_SET_ORIGIN = 0x05
    MODE_POSITION_VELOCITY = 0x06
    MODE_MIT = 0x08  # Force Control Mode (MIT)
    
    # MIT Mode Limits for AK60-6
    P_MIN = -12.56   # Position min (rad)
    P_MAX = 12.56    # Position max (rad)
    V_MIN = -60.0    # Velocity min (rad/s)
    V_MAX = 60.0     # Velocity max (rad/s)
    KP_MIN = 0.0     # Kp min
    KP_MAX = 500.0   # Kp max
    KD_MIN = 0.0     # Kd min
    KD_MAX = 5.0     # Kd max
    T_MIN = -12.0    # Torque min (Nm)
    T_MAX = 12.0     # Torque max (Nm)
    
    def __init__(self, motor_id, can_interface='can0', bitrate=1000000, bus=None):
        """
        Initialize motor controller
        
        Args:
            motor_id: Motor ID (0-255)
            can_interface: CAN interface name (default: 'can0')
            bitrate: CAN bitrate (default: 1Mbps)
            bus: Shared CAN bus instance (optional). If provided, this motor
                 will use the shared bus instead of creating its own.
        """
        self.motor_id = motor_id
        self.can_interface = can_interface
        self.bitrate = bitrate

        self.current = 0.0  # in Amps
        
        # Motor feedback data
        self.position = 0.0
        self.velocity = 0.0
        self.torque = 0.0
        self.temperature = 0
        self.error = 0
        
        # ========== CHANGE 1: Support shared bus ==========
        if bus is None:
            # Create new bus if not provided (backward compatibility)
            try:
                self.bus = can.interface.Bus(
                    channel=can_interface,
                    bustype='socketcan',
                    bitrate=bitrate
                )
                self.owns_bus = True  # This instance owns the bus
                print(f"✓ CAN bus initialized on {can_interface} at {bitrate}bps")
            except Exception as e:
                print(f"✗ Failed to initialize CAN bus: {e}")
                raise
        else:
            # Use provided shared bus
            self.bus = bus
            self.owns_bus = False  # This instance doesn't own the bus
            # Don't print initialization message since bus already exists
    
    def _construct_can_id(self, control_mode):
        """
        Construct 29-bit Extended CAN ID
        
        CAN ID Structure:
        Bits [7:0]   = Motor ID
        Bits [28:8]  = Control Mode ID
        """
        can_id = (control_mode << 8) | self.motor_id
        return can_id
    
    def _constrain(self, value, min_val, max_val):
        """Constrain value to range"""
        return max(min_val, min(max_val, value))
    
    def _float_to_uint(self, x, x_min, x_max, bits):
        """Convert float to unsigned int for packing"""
        span = x_max - x_min
        offset = x - x_min
        return int((offset * ((1 << bits) - 1)) / span)
    
    def _uint_to_float(self, x_int, x_min, x_max, bits):
        """Convert unsigned int to float for unpacking"""
        span = x_max - x_min
        return (x_int * span) / ((1 << bits) - 1) + x_min
    
    def enable(self):
        """Enable motor (set origin temporarily)"""
        can_id = self._construct_can_id(self.MODE_SET_ORIGIN)
        data = [0x00]  # 0 = temporary origin
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)
        print(f"Motor {self.motor_id}: Enabled")
    
    def disable(self):
        """Disable motor (brake with 0 current)"""
        self.send_current_brake(0.0)
        print(f"Motor {self.motor_id}: Disabled")
    
    def set_zero_position(self, permanent=False):
        """
        Set current position as zero
        
        Args:
            permanent: If True, save to flash (default: False)
        """
        can_id = self._construct_can_id(self.MODE_SET_ORIGIN)
        data = [0x01 if permanent else 0x00]
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)
        print(f"Motor {self.motor_id}: Zero position set ({'permanent' if permanent else 'temporary'})")
    
    def send_mit_command(self, position=0.0, velocity=0.0, kp=0.0, kd=0.0, torque=0.0):
        """
        Send MIT mode command (Force Control Mode)
        
        Args:
            position: Desired position (rad)
            velocity: Desired velocity (rad/s)
            kp: Position gain (0-500)
            kd: Velocity gain (0-5)
            torque: Feedforward torque (Nm)
        
        This automatically operates in the appropriate mode based on parameters:
        - Position mode: Set position and kp, kd (velocity=0, torque=0)
        - Velocity mode: Set velocity and kd (position=0, kp=0, torque=0)
        - Torque mode: Set torque only (position=0, velocity=0, kp=0, kd=0)
        """
        # Constrain values to motor limits
        position = self._constrain(position, self.P_MIN, self.P_MAX)
        velocity = self._constrain(velocity, self.V_MIN, self.V_MAX)
        kp = self._constrain(kp, self.KP_MIN, self.KP_MAX)
        kd = self._constrain(kd, self.KD_MIN, self.KD_MAX)
        torque = self._constrain(torque, self.T_MIN, self.T_MAX)
        
        # Convert floats to unsigned ints
        p_int = self._float_to_uint(position, self.P_MIN, self.P_MAX, 16)
        v_int = self._float_to_uint(velocity, self.V_MIN, self.V_MAX, 12)
        kp_int = self._float_to_uint(kp, self.KP_MIN, self.KP_MAX, 12)
        kd_int = self._float_to_uint(kd, self.KD_MIN, self.KD_MAX, 12)
        t_int = self._float_to_uint(torque, self.T_MIN, self.T_MAX, 12)
        
        # Pack data into CAN frame (8 bytes)
        data = [
            (kp_int >> 4) & 0xFF,                           # Byte 0: KP[11:4]
            ((kp_int & 0x0F) << 4) | ((kd_int >> 8) & 0x0F), # Byte 1: KP[3:0], KD[11:8]
            kd_int & 0xFF,                                   # Byte 2: KD[7:0]
            (p_int >> 8) & 0xFF,                            # Byte 3: Position[15:8]
            p_int & 0xFF,                                    # Byte 4: Position[7:0]
            (v_int >> 4) & 0xFF,                            # Byte 5: Velocity[11:4]
            ((v_int & 0x0F) << 4) | ((t_int >> 8) & 0x0F),  # Byte 6: Velocity[3:0], Torque[11:8]
            t_int & 0xFF                                     # Byte 7: Torque[7:0]
        ]
        
        # Send CAN message
        can_id = self._construct_can_id(self.MODE_MIT)
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)
    
    def send_current_brake(self, current):
        """
        Send brake current command
        
        Args:
            current: Brake current in Amps (0-60A)
        """
        current = self._constrain(current, 0, 60.0)
        current_int = int(current * 1000.0)
        
        data = struct.pack('>i', current_int)  # Big-endian int32
        
        can_id = self._construct_can_id(self.MODE_CURRENT_BRAKE)
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        self.bus.send(msg)
    
    def read_feedback(self, timeout=0.1):
        """
        Read motor feedback from CAN bus
        
        Args:
            timeout: Timeout in seconds
            
        Returns:
            True if feedback received, False otherwise
        """
        msg = self.bus.recv(timeout=timeout)
        
        if msg is None:
            return False
        
        # Parse CAN ID to check if it's from our motor
        rx_motor_id = msg.arbitration_id & 0xFF
        
        if rx_motor_id != self.motor_id:
            return False
        
        # Parse feedback data (8 bytes)
        # Based on Section 4.3.1 of the manual
        if len(msg.data) >= 8:
            # Position (int16, 0.1 deg resolution)
            pos_int = struct.unpack('>h', msg.data[0:2])[0]
            self.position = pos_int * 0.1 * 3.14159 / 180.0  # Convert to radians
            
            # Velocity (int16, 10 ERPM resolution)
            spd_int = struct.unpack('>h', msg.data[2:4])[0]
            self.velocity = spd_int * 10.0
            
            # Current (int16, 0.01 A resolution)
            cur_int = struct.unpack('>h', msg.data[4:6])[0]
            self.current = round(cur_int * 0.01, 3)  # 0.01 A resolution, 3 decimals
            self.torque = cur_int * 0.01
            
            # Temperature (int8, °C)
            self.temperature = struct.unpack('b', msg.data[6:7])[0]
            
            # Error code (uint8)
            self.error = msg.data[7]
            
            return True
        
        return False
    
    def get_feedback(self):
        """
        Get latest feedback data
        
        Returns:
            Dictionary with position, velocity, torque, temperature, error
        """
        # return {
        #     'position': self.position,
        #     'velocity': self.velocity,
        #     'torque': self.torque,
        #     'temperature': self.temperature,
        #     'error': self.error
        # }

        return {
            'position': self.position,
            'velocity': self.velocity,
            'torque': self.torque,
            'current': self.current,
            'temperature': self.temperature,
            'error': self.error
        }
    
    def close(self):
        """
        Close CAN bus (only if this instance owns it)
        
        ========== CHANGE 2: Only close bus if we own it ==========
        """
        if self.owns_bus:
            self.bus.shutdown()
            print(f"Motor {self.motor_id}: CAN bus closed")
        else:
            # Don't close shared bus, just inform that motor is disconnected
            print(f"Motor {self.motor_id}: Disconnected from shared bus")


# ============================================================================
# EXAMPLE USAGE
# ============================================================================

if __name__ == "__main__":
    # Create motor instance (Motor ID = 1)
    motor = AK60V3Motor(motor_id=3, can_interface='can0')
    
    try:
        print("\n=== AK60-6 V3.0 MIT Mode Control Demo ===\n")
        
        # Enable motor
        motor.enable()
        time.sleep(0.5)
        
        # Example 3: Torque Control
        print("\n3. Torque Control (2 Nm)")
        motor.send_mit_command(position=3.14, velocity=0, kp=6, kd=4, torque=1.0)
        time.sleep(5)
        
        # Stop motor
        print("\n4. Stopping motor")
        motor.send_mit_command(position=0, velocity=0, kp=0, kd=5, torque=0)
        time.sleep(0.5)
        
        # Disable motor
        motor.disable()
        
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    finally:
        motor.close()
        print("\nDemo complete!")