#!/usr/bin/env python3
"""
Hardware interface for multi-camera rig trigger controller.
Communicates with trigger hardware via USB serial interface.
"""

import serial
import time
from typing import Optional, Tuple


class TriggerHardwareInterface:
    """Interface for hardware trigger controller via USB serial."""
    
    def __init__(self, port: str = "/dev/ttyUSB0", baudrate: int = 9600):
        """
        Initialize the trigger hardware interface.
        
        Args:
            port: Serial port device path
            baudrate: Communication baud rate
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = 1.0  # 1 second timeout
        self.serial_conn: Optional[serial.Serial] = None
        self.video_running = False  # Track video state
        
    def open_connection(self) -> bool:
        """
        Open serial connection to trigger hardware.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            if self.serial_conn is not None and self.serial_conn.is_open:
                return True
                
            self.serial_conn = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout,
            )
            
            # Small delay for connection to stabilize
            time.sleep(0.05)
            return True
            
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.port}: {e}")
            return False
    
    def close_connection(self) -> None:
        """Close serial connection to trigger hardware."""
        if self.serial_conn is not None and self.serial_conn.is_open:
            self.serial_conn.close()
            self.serial_conn = None
    
    def test_connection(self) -> Tuple[bool, str]:
        """
        Test connection to trigger hardware.
        Sends 'i' and expects 'ih\\r\\n' response.
        
        Returns:
            Tuple of (success, message)
        """
        if not self.open_connection():
            return False, "Failed to open connection"
        
        try:
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            
            # Send info command ("i\n")
            self.serial_conn.write(b"i\n")
            self.serial_conn.flush()
            time.sleep(0.05)
            
            # Read response (expecting 'ih\r\n')
            response = self.serial_conn.read(10)  # Read up to 10 bytes
            self.close_connection()
            
            
            # Check for expected response
            if b'ih\r\n' in response:
                return True, f"{response.decode('ascii', errors='ignore')}"
            else:
                return False, f"{response.decode('ascii', errors='ignore')}"
                
        except Exception as e:
            self.close_connection()
            return False, f"{e}"
        
    def send_trigger(self) -> Tuple[bool, str]:
        """
        Send a single trigger pulse.
        Opens connection, sends 't', and closes connection.
        
        Returns:
            Tuple of (success, message)
        """
        if not self.open_connection():
            return False, "Failed to open connection"
        
        try:
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            # Send info command ("t\n")
            self.serial_conn.write(b"t\n")
            self.serial_conn.flush()
            time.sleep(0.05)

            # Read response (expecting "Single trigger\r\n")
            response = self.serial_conn.read(20)  # Read up to 20 bytes
            self.close_connection()
            
            if b'Single trigger\r\n' in response:
                return True, f"{response.decode('ascii', errors='ignore')}"
            else:
                return False, f"{response.decode('ascii', errors='ignore')}"

        except Exception as e:
            self.close_connection()
            return False, f"{e}"
    
    def start_video(self) -> Tuple[bool, str]:
        """
        Start continuous video recording/triggering.
        Opens connection, sends 'r', and closes connection.
        
        Returns:
            Tuple of (success, message)
        """
        if not self.open_connection():
            return False, "Failed to open connection"
        
        try:
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()
            # Send info command ("r\n")
            self.serial_conn.write(b"r\n")
            self.serial_conn.flush()
            time.sleep(0.05)

            # No response expected
            self.close_connection()
            self.video_running = True
            return True, ""

        except Exception as e:
            self.close_connection()
            return False, f"{e}"

    def stop_video(self) -> Tuple[bool, str]:
        """
        Stop continuous video recording/triggering.
        Opens connection, sends 's', and closes connection.
        
        Returns:
            Tuple of (success, message)
        """
        if not self.open_connection():
            return False, "Failed to open connection"
        
        try:
            # Clear any pending data
            self.serial_conn.reset_input_buffer()
            self.serial_conn.reset_output_buffer()

            # Send info command ("s\n")
            self.serial_conn.write(b"s\n")
            self.serial_conn.flush()
            time.sleep(0.05)

            # No response expected
            self.close_connection()
            self.video_running = False
            return True, ""

        except Exception as e:
            self.close_connection()
            return False, f"{e}"
    
    def set_flash_duration(self, duration_ms: int) -> Tuple[bool, str]:
        """
        Set flash duration in milliseconds.
        If video is running, stops it, makes the change, then restarts it.

        Args:
            duration_ms: Flash duration in milliseconds (0-300)

        Returns:
            Tuple of (success, message)
        """
        if not 0 <= duration_ms <= 300:
            return False, f"Flash duration {duration_ms} out of range (0-300 ms)"
        
        if not self.open_connection():
            return False, "Failed to open connection"

        # If a video is running, stop it first
        if self.video_running:
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.serial_conn.write(b"s\n")
                self.serial_conn.flush()
                time.sleep(0.05)
            except Exception as e:
                self.close_connection()
                return False, f"Failed to stop video before setting flash duration due to: {e}"

        # Clear any pending data
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        self.serial_conn.write(b"f\n")
        self.serial_conn.flush()
        time.sleep(0.05)

        # Clear any pending data
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        duration_str = f"{int(duration_ms)}\n"
        self.serial_conn.write(duration_str.encode("ascii"))
        self.serial_conn.flush()
        time.sleep(0.05)

        # Expecting response "Flash duration set to  10 microseconds\r\n"
        response = self.serial_conn.read(100)  # Read up to 100 bytes       

        if self.video_running:
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.serial_conn.write(b"r\n")
                self.serial_conn.flush()
                time.sleep(0.05)
            except Exception as e:
                self.close_connection()
                return False, f"Failed to start video again after setting flash duration due to: {e}"

        # Finally close connection
        self.close_connection()

        if b'Flash duration set to' in response and b'ERROR' not in response:
            return True, f"{response.decode('ascii', errors='ignore')}"
        else:
            return False, f"{response.decode('ascii', errors='ignore')}"
    
    def set_frame_rate(self, rate_hz: int) -> Tuple[bool, str]:
        """
        Set trigger frame rate in Hz (1–5).
        Matches the exact working behavior confirmed via test script.

        Args:
            rate_hz: Frame rate in Hz (1-5)

        Returns:
            Tuple of (success, message)
        """
        if not 1 <= rate_hz <= 5:
            return False, f"Frame rate {rate_hz} out of range (1–5 Hz)"
        
        if not self.open_connection():
            return False, "Failed to open connection"

        # If a video is running, stop it first
        if self.video_running:
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.serial_conn.write(b"s\n")
                self.serial_conn.flush()
                time.sleep(0.05)
            except Exception as e:
                self.close_connection()
                return False, f"Failed to stop video before setting frame rate due to: {e}"

        # Clear any pending data
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        self.serial_conn.write(b"c\n")
        self.serial_conn.flush()
        time.sleep(0.05)

        # Clear any pending data
        self.serial_conn.reset_input_buffer()
        self.serial_conn.reset_output_buffer()

        rate_str = f"{int(rate_hz)}\n"
        self.serial_conn.write(rate_str.encode("ascii"))
        self.serial_conn.flush()
        time.sleep(0.05)

        # Expecting response "I got...\r\n15\r\nFlash frequency set to 15Hz\r\nCamera trigger delay set to 66.67 milliseconds\r\n"
        response = self.serial_conn.read(100)  # Read up to 100 bytes       

        if self.video_running:
            try:
                self.serial_conn.reset_input_buffer()
                self.serial_conn.reset_output_buffer()
                self.serial_conn.write(b"r\n")
                self.serial_conn.flush()
                time.sleep(0.05)
            except Exception as e:
                self.close_connection()
                return False, f"Failed to start video again after setting flash duration due to: {e}"

        # Finally close connection
        self.close_connection()

        if b'Flash frequency set to' in response and b'ERROR' not in response:
            return True, f"{response.decode('ascii', errors='ignore')}"
        else:
            return False, f"{response.decode('ascii', errors='ignore')}"

    def __del__(self):
        """Cleanup: close connection on deletion."""
        self.close_connection()
