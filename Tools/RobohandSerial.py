#!/usr/bin/env python3
# RoboHand Serial - Serial communication handler for RoboHand GUI
# Author: Robert Fudge

import serial
import serial.tools.list_ports
import threading
import queue
import time
import re

class RoboHandSerial:
    """Handles serial communication with the RoboHand device"""
    
    def __init__(self, callback = None):
        """Initialize the serial handler
        
        Args:
            callback: Function to call when data is received (optional)
        """
        self.serial = None
        self.connected = False
        self.callback = callback
        self.cmd_queue = queue.Queue()
        self.response_buffer = ""
        
        # Communication threads
        self.reader_thread = None
        self.writer_thread = None
        self.reader_running = False
        self.writer_running = False
    
    def get_ports(self):
        """Get a list of available serial ports
        
        Returns:
            list: List of available port names
        """
        return [port.device for port in serial.tools.list_ports.comports()]
    
    def connect(self, port, baud=115200):
        """Connect to the specified serial port
        
        Args:
            port: Serial port name
            baud: Baud rate (default: 115200)
            
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            # Close any existing connection
            if self.connected:
                self.disconnect()
            
            # Open new connection
            self.serial = serial.Serial(port, baud, timeout=0.1)
            self.connected = True
            
            # Start communication threads
            self.reader_running = True
            self.writer_running = True
            
            self.reader_thread = threading.Thread(target=self._reader_task, daemon=True)
            self.writer_thread = threading.Thread(target=self._writer_task, daemon=True)
            
            self.reader_thread.start()
            self.writer_thread.start()
            
            return True
        
        except Exception as e:
            print(f"Connection error: {str(e)}")
            return False
    
    def disconnect(self):
        """Disconnect from the current serial port"""
        self.reader_running = False
        self.writer_running = False
        
        # Wait for threads to finish
        if self.reader_thread and self.reader_thread.is_alive():
            self.reader_thread.join(timeout=1.0)
        
        if self.writer_thread and self.writer_thread.is_alive():
            self.writer_thread.join(timeout=1.0)
        
        # Close serial connection
        if self.serial:
            self.serial.close()
            self.serial = None
        
        self.connected = False
    
    def send_command(self, command):
        """Send a command to the device
        
        Args:
            command: Command string to send
            
        Returns:
            bool: True if command was queued, False otherwise
        """
        if not self.connected:
            return False
        
        self.cmd_queue.put(command)
        return True
    
    def _reader_task(self):
        """Reader thread function to read data from the serial port"""
        while self.reader_running:
            if self.serial and self.serial.is_open:
                try:
                    self._read_data()
                
                except Exception as e:
                    print(f"Serial read error: {str(e)}")
                    time.sleep(0.1)
            
            time.sleep(0.01)
    
    def _writer_task(self):
        """Writer thread function to send commands to the serial port"""
        while self.writer_running:
            try:
                if not self.cmd_queue.empty() and self.serial and self.serial.is_open:
                    cmd = self.cmd_queue.get()
                    self.serial.write(f"{cmd}\r\n".encode('utf-8'))
                    time.sleep(0.1)  # Give device time to process
            except Exception as e:
                print(f"Serial write error: {str(e)}")
            
            time.sleep(0.01)
    
    def _process_line(self, line):
        """Process a complete line from the device
        
        This method can be extended to handle specific responses.
        """
        pass  # Basic implementation does nothing

    def _read_data(self):
        """Read data from the device
        
        This method is responsible for calling the _process_line method.
        """
        # Read available data
        data = self.serial.read(self.serial.in_waiting or 1)
        if data:
            text = data.decode('utf-8', errors='replace')
            self.response_buffer += text
                        
            # Call the callback if set
            if self.callback:
                self.callback(text)
                        
            # Process complete lines
            if '\n' in self.response_buffer:
                lines = self.response_buffer.split('\n')
                self.response_buffer = lines[-1]  # Keep the incomplete line
                            
                # Process complete lines
                for line in lines[:-1]:
                    self._process_line(line)
    
    def is_connected(self):
        """Check if the serial connection is active
        
        Returns:
            bool: True if connected, False otherwise
        """
        return self.connected and self.serial and self.serial.is_open


class RoboHandParser:
    """Parser for RoboHand serial responses"""
    
    @staticmethod
    def parse_sensor_data(text):
        """Parse sensor data from text
        
        Args:
            text: Text to parse
            
        Returns:
            dict: Dictionary of parsed sensor data or None if no data found
        """
        result = {}
        
        # Extract accelerometer data
        accel_match = re.search(r"Accelerometer \(g\):\s+X: ([-\d.]+)\s+Y: ([-\d.]+)\s+Z: ([-\d.]+)", text)
        if accel_match:
            result['accel'] = {
                'x': float(accel_match.group(1)),
                'y': float(accel_match.group(2)),
                'z': float(accel_match.group(3))
            }
        
        # Extract gyroscope data
        gyro_match = re.search(r"Gyroscope \(°/s\):\s+X: ([-\d.]+)\s+Y: ([-\d.]+)\s+Z: ([-\d.]+)", text)
        if gyro_match:
            result['gyro'] = {
                'x': float(gyro_match.group(1)),
                'y': float(gyro_match.group(2)),
                'z': float(gyro_match.group(3))
            }
        
        # Extract magnetometer data
        mag_match = re.search(r"Magnetometer \(µT\):\s+X: ([-\d.]+)\s+Y: ([-\d.]+)\s+Z: ([-\d.]+)", text)
        if mag_match:
            result['mag'] = {
                'x': float(mag_match.group(1)),
                'y': float(mag_match.group(2)),
                'z': float(mag_match.group(3))
            }
        
        # Extract ADC voltage data
        adc_match = re.search(r"ADC Voltages \(V\):\s+CH0: ([\d.]+)\s+CH1: ([\d.]+)\s+CH2: ([\d.]+)\s+CH3: ([\d.]+)\s+CH4: ([\d.]+)", text)
        if adc_match:
            result['adc'] = [
                float(adc_match.group(1)),
                float(adc_match.group(2)),
                float(adc_match.group(3)),
                float(adc_match.group(4)),
                float(adc_match.group(5))
            ]
        
        # Extract altitude data
        alt_match = re.search(r"Altitude: ([\d.]+) m", text)
        if alt_match:
            result['altitude'] = float(alt_match.group(1))
        
        return result if result else None
    
    @staticmethod
    def parse_status_data(text):
        """Parse system status data from text
        
        Args:
            text: Text to parse
            
        Returns:
            dict: Dictionary of parsed status data or None if no data found
        """
        result = {}
        
        # Core 0 Load
        core0_match = re.search(r"Core 0 Load: ([\d.]+) loops/ms", text)
        if core0_match:
            result['core0_load'] = float(core0_match.group(1))
        
        # Core 1 Load
        core1_match = re.search(r"Core 1 Load: ([\d.]+) loops/ms", text)
        if core1_match:
            result['core1_load'] = float(core1_match.group(1))
        
        # System OK
        sys_ok_match = re.search(r"System OK: (Yes|No)", text)
        if sys_ok_match:
            result['system_ok'] = sys_ok_match.group(1) == "Yes"
        
        # Emergency Stop
        estop_match = re.search(r"Emergency Stop: (Active|Inactive)", text)
        if estop_match:
            result['emergency_stop'] = estop_match.group(1) == "Active"
        
        # Last Watchdog
        watchdog_match = re.search(r"Last Watchdog: ([\d.]+) seconds ago", text)
        if watchdog_match:
            result['last_watchdog'] = float(watchdog_match.group(1))
        
        # Servo Status
        servo_matches = re.findall(r"Servo (\d+): Pos=(\d+) µs\s+Target=(\d+) µs\s+Moving=(Yes|No)", text)
        if servo_matches:
            result['servos'] = []
            for match in servo_matches:
                servo_id = int(match[0])
                current_pos = int(match[1])
                target_pos = int(match[2])
                moving = match[3] == "Yes"
                
                result['servos'].append({
                    'id': servo_id,
                    'current_position': current_pos,
                    'target_position': target_pos,
                    'is_moving': moving
                })
        
        return result if result else None