#!/usr/bin/env python3
# RoboHand GUI (Improved) - Control interface for the RoboHand Raspberry Pi Pico project
# Author: Claude (based on RoboHand by Robert Fudge)

import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import queue
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

# Import the serial handler
from RobohandSerial import RoboHandSerial, RoboHandParser

class RoboHandGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("RoboHand Controller")
        self.root.geometry("1000x700")
        self.root.minsize(900, 600)
        
        # Serial communication
        self.serial = RoboHandSerial(callback=self.on_serial_data)
        
        # Data storage
        self.sensor_data = {
            'accel': {'x': [], 'y': [], 'z': []},
            'gyro': {'x': [], 'y': [], 'z': []},
            'mag': {'x': [], 'y': [], 'z': []},
            'adc': [[] for _ in range(5)],
            'altitude': [],
            'timestamp': []
        }
        self.max_data_points = 100
        self.last_update_time = time.time()
        
        # Setup UI components
        self.create_notebook()
        self.create_connection_frame()
        self.create_sensor_frame()
        self.create_servo_frame()
        self.create_rgb_frame()
        self.create_terminal_frame()
        
        # Setup a periodic GUI update
        self.root.after(100, self.update_gui)
    
    def create_notebook(self):
        """Create the main notebook with tabs"""
        self.notebook = ttk.Notebook(self.root)
        self.notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create frames for each tab
        self.connection_frame = ttk.Frame(self.notebook)
        self.sensor_frame = ttk.Frame(self.notebook)
        self.servo_frame = ttk.Frame(self.notebook)
        self.rgb_frame = ttk.Frame(self.notebook)
        self.terminal_frame = ttk.Frame(self.notebook)
        
        # Add frames to notebook
        self.notebook.add(self.connection_frame, text="Connection")
        self.notebook.add(self.sensor_frame, text="Sensors")
        self.notebook.add(self.servo_frame, text="Servo Control")
        self.notebook.add(self.rgb_frame, text="RGB LED")
        self.notebook.add(self.terminal_frame, text="Terminal")
    
    def create_connection_frame(self):
        """Create the connection settings tab"""
        frame = ttk.LabelFrame(self.connection_frame, text="Serial Connection")
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Port selection
        ttk.Label(frame, text="Port:").grid(row=0, column=0, sticky=tk.W, padx=5, pady=5)
        self.port_var = tk.StringVar()
        self.port_combo = ttk.Combobox(frame, textvariable=self.port_var)
        self.port_combo.grid(row=0, column=1, sticky=tk.W, padx=5, pady=5)
        ttk.Button(frame, text="Refresh", command=self.refresh_ports).grid(row=0, column=2, padx=5, pady=5)
        
        # Baud rate selection
        ttk.Label(frame, text="Baud Rate:").grid(row=1, column=0, sticky=tk.W, padx=5, pady=5)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(frame, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200"])
        baud_combo.grid(row=1, column=1, sticky=tk.W, padx=5, pady=5)
        
        # Connect button
        self.connect_button = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_button.grid(row=2, column=0, columnspan=3, padx=5, pady=20)
        
        # Status display
        status_frame = ttk.LabelFrame(self.connection_frame, text="System Status")
        status_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Status labels
        self.status_labels = {}
        status_items = [
            "Connection Status", "Core 0 Load", "Core 1 Load", 
            "System OK", "Emergency Stop", "Last Watchdog"
        ]
        
        for i, item in enumerate(status_items):
            ttk.Label(status_frame, text=f"{item}:").grid(row=i, column=0, sticky=tk.W, padx=5, pady=2)
            self.status_labels[item] = ttk.Label(status_frame, text="N/A")
            self.status_labels[item].grid(row=i, column=1, sticky=tk.W, padx=5, pady=2)
        
        # Request status button
        ttk.Button(status_frame, text="Update Status", command=lambda: self.send_command("status")).grid(
            row=len(status_items), column=0, columnspan=2, padx=5, pady=10
        )
        
        # Populate port list on startup
        self.refresh_ports()
    
    def create_sensor_frame(self):
        """Create the sensor visualization tab"""
        # Create the figure for plotting
        self.fig = plt.Figure(figsize=(10, 8), dpi=100)
        self.canvas = FigureCanvasTkAgg(self.fig, master=self.sensor_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Create subplots
        self.ax1 = self.fig.add_subplot(321)  # Accelerometer
        self.ax2 = self.fig.add_subplot(322)  # Gyroscope
        self.ax3 = self.fig.add_subplot(323)  # Magnetometer
        self.ax4 = self.fig.add_subplot(324)  # ADC values
        self.ax5 = self.fig.add_subplot(325)  # Altitude
        
        # Set titles and labels
        self.ax1.set_title("Accelerometer (g)")
        self.ax2.set_title("Gyroscope (°/s)")
        self.ax3.set_title("Magnetometer (µT)")
        self.ax4.set_title("ADC Voltages (V)")
        self.ax5.set_title("Altitude (m)")
        
        # Add legends
        self.ax1.legend(['X', 'Y', 'Z'])
        self.ax2.legend(['X', 'Y', 'Z'])
        self.ax3.legend(['X', 'Y', 'Z'])
        self.ax4.legend(['CH0', 'CH1', 'CH2', 'CH3', 'CH4'])
        
        # Set auto layout
        self.fig.tight_layout()
        
        # Controls frame
        controls_frame = ttk.Frame(self.sensor_frame)
        controls_frame.pack(fill=tk.X, padx=10, pady=5)
        
        # Button to request sensor data
        ttk.Button(controls_frame, text="Read Sensors", command=lambda: self.send_command("sensors")).pack(side=tk.LEFT, padx=5)
        
        # Checkbox for continuous monitoring
        self.continuous_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(controls_frame, text="Continuous Monitoring", variable=self.continuous_var, 
                        command=self.toggle_monitoring).pack(side=tk.LEFT, padx=5)
        
        # Sample interval for continuous monitoring
        ttk.Label(controls_frame, text="Interval (ms):").pack(side=tk.LEFT, padx=5)
        self.interval_var = tk.StringVar(value="1000")
        ttk.Entry(controls_frame, textvariable=self.interval_var, width=6).pack(side=tk.LEFT, padx=5)
    
    def create_servo_frame(self):
        """Create the servo control tab"""
        # Main frame for servo controls
        main_frame = ttk.Frame(self.servo_frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Create servo control sliders
        servo_frame = ttk.LabelFrame(main_frame, text="Servo Control")
        servo_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # We'll create 5 servo controls as mentioned in the Robohand project
        self.servo_vars = []
        self.servo_sliders = []
        
        for i in range(5):
            frame = ttk.Frame(servo_frame)
            frame.pack(fill=tk.X, padx=5, pady=5)
            
            ttk.Label(frame, text=f"Servo {i}:").pack(side=tk.LEFT, padx=5, pady=5)
            
            # Position value
            pos_var = tk.IntVar(value=1500)  # Default center position
            self.servo_vars.append(pos_var)
            
            # Position slider (500-2500 µs)
            slider = ttk.Scale(frame, from_=500, to=2500, orient=tk.HORIZONTAL, 
                              variable=pos_var, length=300)
            slider.pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)
            self.servo_sliders.append(slider)
            
            # Position display
            ttk.Label(frame, textvariable=pos_var).pack(side=tk.LEFT, padx=5, pady=5)
            ttk.Label(frame, text="µs").pack(side=tk.LEFT, pady=5)
            
            # Move button
            ttk.Button(frame, text="Move", 
                      command=lambda i=i: self.move_servo(i)).pack(side=tk.LEFT, padx=5, pady=5)
        
        # Duration control
        duration_frame = ttk.Frame(servo_frame)
        duration_frame.pack(fill=tk.X, padx=5, pady=5)
        
        ttk.Label(duration_frame, text="Duration:").pack(side=tk.LEFT, padx=5, pady=5)
        self.duration_var = tk.IntVar(value=1000)  # Default 1000ms
        ttk.Scale(duration_frame, from_=100, to=5000, orient=tk.HORIZONTAL, 
                 variable=self.duration_var, length=300).pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)
        ttk.Label(duration_frame, textvariable=self.duration_var).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Label(duration_frame, text="ms").pack(side=tk.LEFT, pady=5)
    
    def create_rgb_frame(self):
        """Create the RGB LED control tab"""
        frame = ttk.LabelFrame(self.rgb_frame, text="RGB LED Control")
        frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Color sliders
        self.rgb_vars = [tk.IntVar(value=0) for _ in range(3)]
        colors = [("Red", 0), ("Green", 1), ("Blue", 2)]
        
        for i, (color, idx) in enumerate(colors):
            color_frame = ttk.Frame(frame)
            color_frame.pack(fill=tk.X, padx=5, pady=5)
            
            ttk.Label(color_frame, text=f"{color}:").pack(side=tk.LEFT, padx=5, pady=5)
            ttk.Scale(color_frame, from_=0, to=255, orient=tk.HORIZONTAL, 
                     variable=self.rgb_vars[idx], length=300).pack(side=tk.LEFT, padx=5, pady=5, fill=tk.X, expand=True)
            ttk.Label(color_frame, textvariable=self.rgb_vars[idx]).pack(side=tk.LEFT, padx=5, pady=5)
        
        # Color preview
        self.color_preview = tk.Canvas(frame, width=100, height=100, bg="black")
        self.color_preview.pack(padx=10, pady=10)
        
        # Apply button
        ttk.Button(frame, text="Apply Color", command=self.apply_rgb_color).pack(padx=5, pady=5)
        
        # Blink controls
        blink_frame = ttk.LabelFrame(self.rgb_frame, text="Blink Control")
        blink_frame.pack(fill=tk.X, padx=10, pady=10)
        
        # Blink checkbox
        self.blink_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(blink_frame, text="Enable Blinking", variable=self.blink_var).pack(side=tk.LEFT, padx=5, pady=10)
        
        # Blink interval
        ttk.Label(blink_frame, text="Interval:").pack(side=tk.LEFT, padx=5, pady=10)
        self.blink_interval_var = tk.IntVar(value=500)
        ttk.Scale(blink_frame, from_=50, to=2000, orient=tk.HORIZONTAL, 
                 variable=self.blink_interval_var, length=200).pack(side=tk.LEFT, padx=5, pady=10, fill=tk.X, expand=True)
        ttk.Label(blink_frame, textvariable=self.blink_interval_var).pack(side=tk.LEFT, padx=5, pady=10)
        ttk.Label(blink_frame, text="ms").pack(side=tk.LEFT, pady=10)
        
        # Apply blink button
        ttk.Button(blink_frame, text="Apply Blink", command=self.apply_blink).pack(side=tk.LEFT, padx=5, pady=10)
        
        # Bind color update to preview
        for var in self.rgb_vars:
            var.trace_add("write", self.update_color_preview)
    
    def create_terminal_frame(self):
        """Create the terminal tab for direct command input"""
        # Command history display
        history_frame = ttk.LabelFrame(self.terminal_frame, text="Command Output")
        history_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Text widget with scrollbar
        self.terminal_text = tk.Text(history_frame, wrap=tk.WORD, bg="black", fg="green")
        self.terminal_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        
        scrollbar = ttk.Scrollbar(history_frame, command=self.terminal_text.yview)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.terminal_text.config(yscrollcommand=scrollbar.set)
        
        # Command input
        input_frame = ttk.Frame(self.terminal_frame)
        input_frame.pack(fill=tk.X, padx=10, pady=5)
        
        ttk.Label(input_frame, text="Command:").pack(side=tk.LEFT, padx=5, pady=5)
        self.cmd_entry = ttk.Entry(input_frame)
        self.cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5, pady=5)
        self.cmd_entry.bind("<Return>", lambda e: self.send_manual_command())
        
        ttk.Button(input_frame, text="Send", command=self.send_manual_command).pack(side=tk.LEFT, padx=5, pady=5)
        ttk.Button(input_frame, text="Clear Terminal", command=self.clear_terminal).pack(side=tk.LEFT, padx=5, pady=5)
    
    # ===== Connection functions =====
    
    def refresh_ports(self):
        """Refresh the list of available serial ports"""
        ports = self.serial.get_ports()
        self.port_combo['values'] = ports
        if ports:
            self.port_combo.current(0)
    
    def toggle_connection(self):
        """Connect to or disconnect from the selected port"""
        if not self.serial.is_connected():
            port = self.port_var.get()
            baud = int(self.baud_var.get())
            
            if self.serial.connect(port, baud):
                self.connect_button.config(text="Disconnect")
                self.status_labels["Connection Status"].config(text="Connected")
                self.terminal_log(f"Connected to {port} at {baud} baud")
                
                # Get initial status
                self.send_command("status")
            else:
                messagebox.showerror("Connection Error", "Failed to connect to the device")
                self.status_labels["Connection Status"].config(text="Connection Failed")
        else:
            # Disconnect
            self.serial.disconnect()
            self.connect_button.config(text="Connect")
            self.status_labels["Connection Status"].config(text="Disconnected")
            self.terminal_log("Disconnected from device")
    
    def send_command(self, cmd):
        """Send a command to the device"""
        if self.serial.is_connected():
            self.serial.send_command(cmd)
            return True
        else:
            messagebox.showwarning("Not Connected", "Please connect to the device first.")
            return False
    
    def send_manual_command(self):
        """Send the command from the entry field"""
        cmd = self.cmd_entry.get().strip()
        if cmd:
            if self.send_command(cmd):
                self.terminal_log(f"> {cmd}")
                self.cmd_entry.delete(0, tk.END)
    
    def on_serial_data(self, data):
        """Handle incoming data from the serial port"""
        # Add to terminal
        self.terminal_log(data, newline=False)
        
        # Check for sensor data
        sensor_data = RoboHandParser.parse_sensor_data(data)
        if sensor_data:
            self.process_sensor_data(sensor_data)
        
        # Check for status data
        status_data = RoboHandParser.parse_status_data(data)
        if status_data:
            self.process_status_data(status_data)
    
    def process_sensor_data(self, data):
        """Process parsed sensor data"""
        # Add timestamp
        curr_time = time.time()
        self.sensor_data['timestamp'].append(curr_time)
        if len(self.sensor_data['timestamp']) > self.max_data_points:
            self.sensor_data['timestamp'] = self.sensor_data['timestamp'][-self.max_data_points:]
        
        # Process accelerometer data
        if 'accel' in data:
            self.add_sensor_point('accel', 'x', data['accel']['x'])
            self.add_sensor_point('accel', 'y', data['accel']['y'])
            self.add_sensor_point('accel', 'z', data['accel']['z'])
        
        # Process gyroscope data
        if 'gyro' in data:
            self.add_sensor_point('gyro', 'x', data['gyro']['x'])
            self.add_sensor_point('gyro', 'y', data['gyro']['y'])
            self.add_sensor_point('gyro', 'z', data['gyro']['z'])
        
        # Process magnetometer data
        if 'mag' in data:
            self.add_sensor_point('mag', 'x', data['mag']['x'])
            self.add_sensor_point('mag', 'y', data['mag']['y'])
            self.add_sensor_point('mag', 'z', data['mag']['z'])
        
        # Process ADC data
        if 'adc' in data:
            for i, value in enumerate(data['adc']):
                self.add_sensor_point('adc', i, value)
        
        # Process altitude data
        if 'altitude' in data:
            self.add_sensor_point('altitude', 0, data['altitude'])
        
        # Update plots
        self.update_plots()
    
    def process_status_data(self, data):
        """Process parsed status data"""
        # Update status labels
        if 'core0_load' in data:
            self.status_labels["Core 0 Load"].config(text=f"{data['core0_load']:.2f} loops/ms")
        
        if 'core1_load' in data:
            self.status_labels["Core 1 Load"].config(text=f"{data['core1_load']:.2f} loops/ms")
        
        if 'system_ok' in data:
            self.status_labels["System OK"].config(text="Yes" if data['system_ok'] else "No")
        
        if 'emergency_stop' in data:
            self.status_labels["Emergency Stop"].config(text="Active" if data['emergency_stop'] else "Inactive")
        
        if 'last_watchdog' in data:
            self.status_labels["Last Watchdog"].config(text=f"{data['last_watchdog']:.1f} seconds ago")
    
    # ===== Servo Control Functions =====
    
    def move_servo(self, servo_index):
        """Send command to move a servo to the specified position"""
        position = self.servo_vars[servo_index].get()
        duration = self.duration_var.get()
        
        command = f"servo {servo_index} {position} {duration}"
        if self.send_command(command):
            self.terminal_log(f"Moving servo {servo_index} to {position}µs over {duration}ms")
    
    # ===== RGB Control Functions =====
    
    def update_color_preview(self, *args):
        """Update the color preview canvas"""
        r = self.rgb_vars[0].get()
        g = self.rgb_vars[1].get()
        b = self.rgb_vars[2].get()
        
        # Update canvas color
        color = f"#{r:02x}{g:02x}{b:02x}"
        self.color_preview.config(bg=color)
    
    def apply_rgb_color(self):
        """Apply the selected RGB color"""
        r = self.rgb_vars[0].get()
        g = self.rgb_vars[1].get()
        b = self.rgb_vars[2].get()
        
        command = f"rgb {r} {g} {b}"
        if self.send_command(command):
            self.terminal_log(f"Setting RGB color to ({r}, {g}, {b})")
    
    def apply_blink(self):
        """Apply the blink settings"""
        state = "on" if self.blink_var.get() else "off"
        interval = self.blink_interval_var.get()
        
        command = f"blink {state} {interval}"
        if self.send_command(command):
            if state == "on":
                self.terminal_log(f"Enabling LED blink with {interval}ms interval")
            else:
                self.terminal_log("Disabling LED blink")
    
    # ===== Sensor Data and Plotting Functions =====
    
    def add_sensor_point(self, sensor_type, axis, value):
        """Add a data point to the sensor data arrays"""
        if sensor_type == 'accel' or sensor_type == 'gyro' or sensor_type == 'mag':
            self.sensor_data[sensor_type][axis].append(value)
            if len(self.sensor_data[sensor_type][axis]) > self.max_data_points:
                self.sensor_data[sensor_type][axis] = self.sensor_data[sensor_type][axis][-self.max_data_points:]
        
        elif sensor_type == 'adc':
            self.sensor_data[sensor_type][axis].append(value)
            if len(self.sensor_data[sensor_type][axis]) > self.max_data_points:
                self.sensor_data[sensor_type][axis] = self.sensor_data[sensor_type][axis][-self.max_data_points:]
        
        elif sensor_type == 'altitude':
            self.sensor_data[sensor_type].append(value)
            if len(self.sensor_data[sensor_type]) > self.max_data_points:
                self.sensor_data[sensor_type] = self.sensor_data[sensor_type][-self.max_data_points:]
    
    def update_plots(self):
        """Update all sensor plots with the latest data"""
        # Clear all plots
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        self.ax5.clear()
        
        # Set titles
        self.ax1.set_title("Accelerometer (g)")
        self.ax2.set_title("Gyroscope (°/s)")
        self.ax3.set_title("Magnetometer (µT)")
        self.ax4.set_title("ADC Voltages (V)")
        self.ax5.set_title("Altitude (m)")
        
        # Create x-axis values (just indices for now)
        x = range(len(self.sensor_data['timestamp']))
        
        # Plot data if available
        if x:
            # Accelerometer
            if self.sensor_data['accel']['x']:
                self.ax1.plot(self.sensor_data['accel']['x'], label='X')
                self.ax1.plot(self.sensor_data['accel']['y'], label='Y')
                self.ax1.plot(self.sensor_data['accel']['z'], label='Z')
                self.ax1.legend()
            
            # Gyroscope
            if self.sensor_data['gyro']['x']:
                self.ax2.plot(self.sensor_data['gyro']['x'], label='X')
                self.ax2.plot(self.sensor_data['gyro']['y'], label='Y')
                self.ax2.plot(self.sensor_data['gyro']['z'], label='Z')
                self.ax2.legend()
            
            # Magnetometer
            if self.sensor_data['mag']['x']:
                self.ax3.plot(self.sensor_data['mag']['x'], label='X')
                self.ax3.plot(self.sensor_data['mag']['y'], label='Y')
                self.ax3.plot(self.sensor_data['mag']['z'], label='Z')
                self.ax3.legend()
            
            # ADC values
            for i in range(5):
                if self.sensor_data['adc'][i]:
                    self.ax4.plot(self.sensor_data['adc'][i], label=f'CH{i}')
            self.ax4.legend()
            
            # Altitude
            if self.sensor_data['altitude']:
                self.ax5.plot(self.sensor_data['altitude'], 'k-', label='Altitude')
                self.ax5.legend()
        
        # Update the canvas
        self.fig.tight_layout()
        self.canvas.draw()
    
    def toggle_monitoring(self):
        """Toggle continuous sensor monitoring"""
        if self.continuous_var.get():
            # Start monitoring
            interval = int(self.interval_var.get())
            if interval < 100:
                interval = 100  # Minimum interval
            
            self.send_command(f"telemetry on {interval}")
            self.terminal_log(f"Started continuous monitoring with {interval}ms interval")
        else:
            # Stop monitoring
            self.send_command("telemetry off")
            self.terminal_log("Stopped continuous monitoring")
    
    # ===== Terminal Functions =====
    
    def terminal_log(self, text, newline=True):
        """Add text to the terminal output"""
        self.terminal_text.config(state=tk.NORMAL)
        if newline:
            text = text + "\n"
        self.terminal_text.insert(tk.END, text)
        self.terminal_text.see(tk.END)
        self.terminal_text.config(state=tk.DISABLED)
    
    def clear_terminal(self):
        """Clear the terminal output"""
        self.terminal_text.config(state=tk.NORMAL)
        self.terminal_text.delete(1.0, tk.END)
        self.terminal_text.config(state=tk.DISABLED)
    
    # ===== Update Function =====
    
    def update_gui(self):
        """Periodic update function for the GUI"""
        # Check if we need to request sensor data
        if self.serial.is_connected() and self.continuous_var.get():
            current_time = time.time()
            if not hasattr(self, 'last_sensor_request') or current_time - self.last_sensor_request > 2:
                self.send_command("sensors")
                self.last_sensor_request = current_time
        
        # Schedule the next update
        self.root.after(100, self.update_gui)


# Entry point
if __name__ == "__main__":
    root = tk.Tk()
    app = RoboHandGUI(root)
    root.mainloop()