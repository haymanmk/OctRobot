#!/usr/bin/env python3
"""
OctroBot Robot Arm - Host Test Script (Phase 3b)
Copyright (c) 2026 OctroBot Project

Interactive CLI for testing manual control and demo recording features.

Usage:
    python3 host_test.py [port]
    
Commands:
    ping              - Test servos
    jog <id> <dir> <deg>  - Jog joint (dir: +1/-1, deg: 1/5/10)
    set <id> <angle>  - Set joint angle in radians
    read              - Read current joint state
    loop              - Start continuous read loop (100Hz)
    stop_loop         - Stop read loop
    test <id> <start> <end> <cycles> - Oscillate joint
    stop              - Emergency stop
    record <id>       - Start demo recording
    waypoint <delay>  - Add waypoint with delay (ms)
    finish            - Finish recording
    play <id>         - Play demo
    clear <id>        - Clear demo
    quit              - Exit
"""

import serial
import struct
import sys
import time
from typing import Optional, Tuple

# Command definitions (must match firmware)
CMD_MOVE_JOINTS = 0x01
CMD_MOVE_CARTESIAN = 0x02
CMD_READ_STATE = 0x03
CMD_STOP = 0x04
CMD_HOME = 0x05
CMD_SET_PARAMS = 0x10

CMD_JOG_JOINT = 0x20
CMD_SET_JOINT_DIRECT = 0x21
CMD_START_READ_LOOP = 0x22
CMD_STOP_READ_LOOP = 0x23
CMD_SINGLE_JOINT_TEST = 0x24

CMD_PLAY_DEMO = 0x30
CMD_START_DEMO_RECORDING = 0x31
CMD_ADD_WAYPOINT = 0x32
CMD_FINISH_DEMO_RECORDING = 0x33
CMD_CLEAR_DEMO = 0x34

CMD_STATUS_REPORT = 0xF0
CMD_DEMO_RECORDING_STATUS = 0xF2

PACKET_START_BYTE = 0xAA


def crc8_compute(data: bytes) -> int:
    """
    Compute CRC8 checksum using Dallas/Maxim polynomial (0x31).
    
    Educational Note:
    This matches the firmware implementation exactly.
    """
    crc = 0x00
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x80:
                crc = (crc << 1) ^ 0x31
            else:
                crc = crc << 1
            crc &= 0xFF
    return crc


class Packet:
    """Packet builder/parser matching firmware protocol."""
    
    def __init__(self, cmd: int, payload: bytes = b''):
        self.cmd = cmd
        self.payload = payload
        
    def serialize(self) -> bytes:
        """Serialize packet to bytes for transmission."""
        payload_len = len(self.payload)
        # CRC over: CMD + LEN + PAYLOAD
        crc_data = bytes([self.cmd, payload_len]) + self.payload
        crc = crc8_compute(crc_data)
        
        # Build packet: START + CMD + LEN + PAYLOAD + CRC
        packet = bytes([PACKET_START_BYTE, self.cmd, payload_len]) + self.payload + bytes([crc])
        return packet
    
    @staticmethod
    def parse(data: bytes) -> Optional['Packet']:
        """Parse received packet."""
        if len(data) < 4:  # START + CMD + LEN + CRC minimum
            return None
            
        if data[0] != PACKET_START_BYTE:
            return None
            
        cmd = data[1]
        payload_len = data[2]
        
        expected_len = 3 + payload_len + 1  # HEADER + PAYLOAD + CRC
        if len(data) < expected_len:
            return None
            
        payload = data[3:3+payload_len]
        crc_received = data[3+payload_len]
        
        # Verify CRC
        crc_data = bytes([cmd, payload_len]) + payload
        crc_computed = crc8_compute(crc_data)
        
        if crc_received != crc_computed:
            print(f"CRC mismatch: expected {crc_computed:02X}, got {crc_received:02X}")
            return None
            
        return Packet(cmd, payload)


class OctrobotHost:
    """Host controller for OctroBot arm."""
    
    def __init__(self, port: str, baudrate: int = 115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        # Give USB device time to stabilize
        time.sleep(1)
        print(f"Connected to {port} at {baudrate} baud")
        
    def send_command(self, cmd: int, payload: bytes = b'') -> None:
        """Send command packet to robot."""
        pkt = Packet(cmd, payload)
        data = pkt.serialize()
        self.ser.write(data)
        print(f"Sent: CMD=0x{cmd:02X}, len={len(payload)} bytes")
        
    def receive_response(self, timeout: float = 1.0) -> Optional[Packet]:
        """Receive and parse response packet."""
        start_time = time.time()
        buffer = bytearray()
        
        while (time.time() - start_time) < timeout:
            if self.ser.in_waiting > 0:
                byte = self.ser.read(1)
                buffer.extend(byte)
                
                # Try to parse when we have enough bytes
                if len(buffer) >= 4:
                    # Check if we have a complete packet
                    if buffer[0] == PACKET_START_BYTE and len(buffer) >= 3:
                        expected_len = 3 + buffer[2] + 1  # HEADER + PAYLOAD + CRC
                        if len(buffer) >= expected_len:
                            pkt = Packet.parse(bytes(buffer[:expected_len]))
                            if pkt:
                                return pkt
                            # If parse failed, shift buffer and try again
                            buffer = buffer[1:]
            time.sleep(0.001)
        
        if len(buffer) > 0:
            print(f"Warning: Received {len(buffer)} bytes but no valid packet: {buffer.hex()}")
        return None
        
    def cmd_jog_joint(self, joint_id: int, direction: int, step_deg: int) -> None:
        """Jog a joint incrementally."""
        payload = struct.pack('BBB', joint_id, direction & 0xFF, step_deg)
        self.send_command(CMD_JOG_JOINT, payload)
        
    def cmd_set_joint_direct(self, joint_id: int, angle_rad: float) -> None:
        """Set joint to specific angle (bypass trajectory)."""
        payload = struct.pack('Bf', joint_id, angle_rad)
        self.send_command(CMD_SET_JOINT_DIRECT, payload)
        
    def cmd_read_state(self) -> None:
        """Request current robot state."""
        self.send_command(CMD_READ_STATE)
        print("Waiting for response...")
        resp = self.receive_response(timeout=2.0)
        if resp and resp.cmd == CMD_STATUS_REPORT:
            self.parse_status_report(resp.payload)
        elif resp:
            print(f"Received unexpected response: CMD=0x{resp.cmd:02X}")
        else:
            print("No response received (timeout)")
            print("Tips:")
            print("  - Check if firmware is running (look for heartbeat messages)")
            print("  - Verify serial port is correct")
            print("  - Try reflashing firmware")
            print("  - Check servo connections if firmware expects servos")
            
    def cmd_start_read_loop(self) -> None:
        """Start continuous position streaming at 100Hz."""
        self.send_command(CMD_START_READ_LOOP)
        print("Read loop started. Press Ctrl+C to stop and send stop command.")
        try:
            while True:
                resp = self.receive_response(timeout=0.02)
                if resp and resp.cmd == CMD_STATUS_REPORT:
                    self.parse_status_report(resp.payload)
        except KeyboardInterrupt:
            self.cmd_stop_read_loop()
            
    def cmd_stop_read_loop(self) -> None:
        """Stop continuous streaming."""
        self.send_command(CMD_STOP_READ_LOOP)
        print("Read loop stopped")
        
    def cmd_single_joint_test(self, joint_id: int, start_rad: float, end_rad: float, cycles: int) -> None:
        """Oscillate a single joint for testing."""
        payload = struct.pack('Bffb', joint_id, start_rad, end_rad, cycles)
        self.send_command(CMD_SINGLE_JOINT_TEST, payload)
        print(f"Testing joint {joint_id} for {cycles} cycles...")
        
    def cmd_stop(self) -> None:
        """Emergency stop."""
        self.send_command(CMD_STOP)
        print("Emergency stop sent!")
        
    def cmd_start_demo_recording(self, demo_id: int) -> None:
        """Start recording a demo sequence."""
        payload = struct.pack('B', demo_id)
        self.send_command(CMD_START_DEMO_RECORDING, payload)
        print(f"Started recording demo {demo_id}")
        
    def cmd_add_waypoint(self, delay_ms: int) -> None:
        """Add current position as waypoint."""
        payload = struct.pack('I', delay_ms)
        self.send_command(CMD_ADD_WAYPOINT, payload)
        print(f"Waypoint added (delay={delay_ms}ms)")
        time.sleep(0.1)
        # Check for demo status response
        resp = self.receive_response(timeout=0.5)
        if resp and resp.cmd == CMD_DEMO_RECORDING_STATUS:
            demo_id, waypoint_count, bytes_remaining = struct.unpack('BBH', resp.payload[:4])
            print(f"  Demo {demo_id}: {waypoint_count} waypoints, {bytes_remaining} bytes remaining")
            
    def cmd_finish_demo_recording(self) -> None:
        """Finish and save demo."""
        self.send_command(CMD_FINISH_DEMO_RECORDING)
        print("Demo recording finished and saved")
        
    def cmd_play_demo(self, demo_id: int) -> None:
        """Play a recorded demo."""
        payload = struct.pack('B', demo_id)
        self.send_command(CMD_PLAY_DEMO, payload)
        print(f"Playing demo {demo_id}...")
        
    def cmd_clear_demo(self, demo_id: int) -> None:
        """Clear a demo from storage."""
        payload = struct.pack('B', demo_id)
        self.send_command(CMD_CLEAR_DEMO, payload)
        print(f"Demo {demo_id} cleared")
        
    def parse_status_report(self, payload: bytes) -> None:
        """Parse and display status report."""
        # Joint angles: 6 × float32 = 24 bytes
        # Temperatures: 6 bytes
        # Voltages: 6 bytes
        # Loads: 6 × uint16 = 12 bytes
        # Flags: 1 byte
        # Total: 49 bytes
        
        if len(payload) < 49:
            print(f"Invalid status payload length: {len(payload)}")
            return
            
        angles = struct.unpack('6f', payload[0:24])
        temps = struct.unpack('6B', payload[24:30])
        voltages = struct.unpack('6B', payload[30:36])
        loads = struct.unpack('6H', payload[36:48])
        flags = payload[48]
        
        is_moving = flags & 0x01
        error_flags = flags >> 1
        
        print("\n--- Robot State ---")
        for i in range(6):
            print(f"Joint {i+1}: {angles[i]:7.3f} rad ({angles[i]*180/3.14159:6.1f}°)  "
                  f"Temp: {temps[i]:3d}°C  V: {voltages[i]/10:.1f}V  Load: {loads[i]:5d}")
        print(f"Moving: {'YES' if is_moving else 'NO'}  Errors: 0x{error_flags:02X}")
        print("-------------------\n")


def interactive_cli(host: OctrobotHost):
    """Interactive command-line interface."""
    print(__doc__)
    
    while True:
        try:
            cmd = input("\noctrobot> ").strip().split()
            if not cmd:
                continue
                
            if cmd[0] == 'quit' or cmd[0] == 'exit':
                break
            elif cmd[0] == 'ping':
                print("Use 'read' command to check servo status")
            elif cmd[0] == 'jog' and len(cmd) >= 4:
                joint_id = int(cmd[1])
                direction = int(cmd[2])
                step_deg = int(cmd[3])
                host.cmd_jog_joint(joint_id, direction, step_deg)
            elif cmd[0] == 'set' and len(cmd) >= 3:
                joint_id = int(cmd[1])
                angle_rad = float(cmd[2])
                host.cmd_set_joint_direct(joint_id, angle_rad)
            elif cmd[0] == 'read':
                host.cmd_read_state()
            elif cmd[0] == 'loop':
                host.cmd_start_read_loop()
            elif cmd[0] == 'stop_loop':
                host.cmd_stop_read_loop()
            elif cmd[0] == 'test' and len(cmd) >= 5:
                joint_id = int(cmd[1])
                start_rad = float(cmd[2])
                end_rad = float(cmd[3])
                cycles = int(cmd[4])
                host.cmd_single_joint_test(joint_id, start_rad, end_rad, cycles)
            elif cmd[0] == 'stop':
                host.cmd_stop()
            elif cmd[0] == 'record' and len(cmd) >= 2:
                demo_id = int(cmd[1])
                host.cmd_start_demo_recording(demo_id)
            elif cmd[0] == 'waypoint' and len(cmd) >= 2:
                delay_ms = int(cmd[1])
                host.cmd_add_waypoint(delay_ms)
            elif cmd[0] == 'finish':
                host.cmd_finish_demo_recording()
            elif cmd[0] == 'play' and len(cmd) >= 2:
                demo_id = int(cmd[1])
                host.cmd_play_demo(demo_id)
            elif cmd[0] == 'clear' and len(cmd) >= 2:
                demo_id = int(cmd[1])
                host.cmd_clear_demo(demo_id)
            else:
                print(f"Unknown command: {cmd[0]}")
                print("Type 'quit' for help")
                
        except KeyboardInterrupt:
            print("\nUse 'quit' to exit")
        except Exception as e:
            print(f"Error: {e}")


if __name__ == '__main__':
    port = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyACM0'
    
    try:
        host = OctrobotHost(port)
        interactive_cli(host)
    except serial.SerialException as e:
        print(f"Failed to open {port}: {e}")
        print("Try: python3 host_test.py /dev/ttyUSB0")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        print("Goodbye!")
