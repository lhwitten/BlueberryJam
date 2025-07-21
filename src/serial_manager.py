import serial
import serial.tools.list_ports
import threading
import time
import tkinter as tk
from tkinter import ttk


class SerialManager:
    """Manages serial port communication for the blueberry sorter."""
    
    def __init__(self, root):
        self.root = root
        self.serial_port = None
        self.serial_port_addr = ""
        self.serial_thread = None
        self.serial_thread_running = False
        
        # PH trigger setup
        self.ph_triggers_received = set()
        self.expected_ph_triggers = {"1", "2", "3"}
        
        # Callbacks
        self.on_trigger_complete = None  # Callback when all triggers received
        self.on_status_change = None    # Callback when connection status changes
        
        # Initialize with available ports
        self.refresh_available_ports()
        
    def refresh_available_ports(self):
        """Get list of available serial ports."""
        ports = serial.tools.list_ports.comports()
        self.available_ports = [port.device for port in ports]
        for port in sorted(ports, key=lambda p: p.device):
            print("{}: {} [{}]".format(port.device, port.description, port.hwid))
        if self.available_ports:
            self.serial_port_addr = self.available_ports[0]
        return self.available_ports
    
    def connect(self, port_addr=None):
        """Connect to the specified serial port."""
        try:
            # Close existing connection if any
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.serial_port = None
            
            # Use provided port or stored address
            if port_addr:
                self.serial_port_addr = port_addr
            
            if not self.serial_port_addr:
                self._notify_status_change("No port selected", False)
                return False
                
            self.serial_port = serial.Serial(self.serial_port_addr, baudrate=9600, timeout=1)
            self._notify_status_change("Connected", True)
            return True
            
        except serial.SerialException as e:
            self._notify_status_change("Disconnected", False)
            print(f"Failed to open serial port: {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the serial port."""
        self.stop_serial_thread()
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None
        self._notify_status_change("Disconnected", False)
    
    def start_serial_thread(self):
        """Start a separate thread for serial listening."""
        if self.serial_thread is None:
            self.serial_thread_running = True
            self.serial_thread = threading.Thread(target=self.serial_listener_thread, daemon=True)
            self.serial_thread.start()

    def stop_serial_thread(self):
        """Stop the serial listening thread."""
        self.serial_thread_running = False
        if self.serial_thread:
            if threading.current_thread() != self.serial_thread:
                self.serial_thread.join()
            self.serial_thread = None

    def serial_listener_thread(self):
        """Thread function for listening to serial data."""
        while self.serial_thread_running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting > 0:
                        try:
                            line = self.serial_port.readline().decode('utf-8').strip()
                        except Exception as e:
                            print(f"Serial read exception: {e}")
                            line = ""
                        if line in self.expected_ph_triggers:
                            print(f"Received trigger: {line}")
                            self.ph_triggers_received.add(line)
                            # Schedule UI update on main thread
                            self.root.after(0, self._check_triggers_complete)
                except Exception as e:
                    print(f"Serial read error: {e}")
            else:
                time.sleep(0.05)  # Sleep briefly to reduce CPU usage when port is not open
    
    def _check_triggers_complete(self):
        """Check if all triggers have been received and notify callback."""
        if self.ph_triggers_received == self.expected_ph_triggers:
            if self.on_trigger_complete:
                self.on_trigger_complete()
            self.ph_triggers_received.clear()
    
    def send_eject_command(self, eject_array, conveyor_speed=5, num_shakes=3, advance_steps=2):
        """
        Send eject command to Arduino.
        
        Args:
            eject_array: List of 3 integers [0,1,0] for each eject port
            conveyor_speed: Speed setting (0-9)
            num_shakes: Number of shakes (1-10)
        """
        if self.serial_port and self.serial_port.is_open:
            try:
                # Convert eject_array (e.g., [0, 0, 1]) to string "001"
                data = ''.join(str(x) for x in eject_array) + '1' + str(num_shakes) + str(conveyor_speed) +'\n'
                self.serial_port.write(data.encode('utf-8'))
                print(f"Sent eject command: {data.strip()}")
                return True, data.strip()
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                return False, "error"
        return False, "error"

    def send_next_command(self):
        """Send NEXT command to Arduino."""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"NEXT\n")
                print("Sent NEXT command")
                return True
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                return False
        return False
    
    def is_connected(self):
        """Check if serial port is connected."""
        return self.serial_port and self.serial_port.is_open
    
    def get_received_triggers(self):
        """Get set of currently received triggers."""
        return self.ph_triggers_received.copy()
    
    def clear_triggers(self):
        """Clear all received triggers."""
        self.ph_triggers_received.clear()
    
    def set_trigger_callback(self, callback):
        """Set callback function for when all triggers are received."""
        self.on_trigger_complete = callback
    
    def set_status_callback(self, callback):
        """Set callback function for status changes."""
        self.on_status_change = callback
    
    def _notify_status_change(self, status_text, is_connected):
        """Notify status change callback if set."""
        if self.on_status_change:
            self.on_status_change(status_text, is_connected)
    
    def __del__(self):
        """Cleanup on object destruction."""
        self.disconnect()


class SerialControlWidget:
    """Widget for controlling serial connection in the GUI."""
    
    def __init__(self, parent_frame, serial_manager):
        self.serial_manager = serial_manager
        self.parent_frame = parent_frame
        
        # Create serial control frame
        self.serial_frame = tk.Frame(parent_frame)
        self.serial_frame.pack(fill=tk.X, pady=2)
        tk.Label(self.serial_frame, text="Serial:").pack(side=tk.LEFT)
        
        # Port selection dropdown
        self.serial_port_var = tk.StringVar(value=serial_manager.serial_port_addr)
        self.serial_port_dropdown = ttk.Combobox(self.serial_frame, textvariable=self.serial_port_var, width=15)
        self.serial_port_dropdown['values'] = serial_manager.available_ports
        if serial_manager.available_ports:
            self.serial_port_dropdown.set(serial_manager.available_ports[0])
        self.serial_port_dropdown.pack(side=tk.LEFT, padx=2)
        
        # Control buttons
        self.btn_refresh_ports = ttk.Button(self.serial_frame, text="Refresh", command=self.refresh_ports, width=8)
        self.btn_refresh_ports.pack(side=tk.LEFT, padx=2)
        
        self.btn_connect_serial = ttk.Button(self.serial_frame, text="Connect", command=self.toggle_connection, width=8)
        self.btn_connect_serial.pack(side=tk.LEFT, padx=2)
        
        # Set up status callback
        serial_manager.set_status_callback(self.on_status_change)
    
    def toggle_connection(self):
        """Toggle between connecting and disconnecting the serial port."""
        if self.serial_manager.is_connected():
            self.serial_manager.disconnect()
            self.btn_connect_serial.config(text="Connect")
        else:
            self.connect_serial()
            if self.serial_manager.is_connected():
                self.btn_connect_serial.config(text="Disconnect")   
    
    def refresh_ports(self):
        """Refresh the list of available ports."""
        port_list = self.serial_manager.refresh_available_ports()
        self.serial_port_dropdown['values'] = port_list
        if port_list:
            self.serial_port_var.set(port_list[0])
        else:
            self.serial_port_var.set('')
    
    def connect_serial(self):
        """Connect to the selected serial port."""
        port_addr = self.serial_port_var.get()
        self.serial_manager.connect(port_addr)

    def disconnect_serial(self):
        """Disconnect from the serial port."""
        self.serial_manager.disconnect()
    
    def on_status_change(self, status_text, is_connected):
        """Handle status changes from serial manager."""
        # This can be overridden by the parent to update UI elements
        print(f"Serial status: {status_text}")


class PHIndicatorWidget:
    """Widget for displaying PH trigger status."""
    
    def __init__(self, parent_frame, serial_manager):
        self.serial_manager = serial_manager
        self.parent_frame = parent_frame
        
        # Status label for connection
        self.serial_status_var = tk.StringVar(value="Disconnected")
        self.serial_status_label = tk.Label(parent_frame, textvariable=self.serial_status_var, fg="red", anchor="w", justify="left")
        self.serial_status_label.grid(row=0, column=0, sticky="w")
        
        # PH trigger emoji indicators
        self.ph_emoji_labels = {}
        self.ph_frame = tk.Frame(parent_frame)
        self.ph_frame.grid(row=0, column=1, sticky="w", padx=(10,0))
        
        # Set up status callback
        serial_manager.set_status_callback(self.update_status)
        
        # Initial display
        self.update_ph_emoji_labels()
    
    def update_status(self, status_text, is_connected):
        """Update the connection status display."""
        self.serial_status_var.set(status_text)
        self.serial_status_label.config(fg="green" if is_connected else "red")
    
    def update_ph_emoji_labels(self):
        """Update the PH trigger emoji indicators."""
        # Clear existing labels
        for widget in self.ph_frame.winfo_children():
            widget.destroy()
        
        received_triggers = self.serial_manager.get_received_triggers()
        expected_triggers = self.serial_manager.expected_ph_triggers
        
        for i, ph in enumerate(expected_triggers):
            # Show sun if received, moon if not
            emoji = "🌞" if ph in received_triggers else "🌚"
            lbl = tk.Label(self.ph_frame, text=emoji, font=("Arial", 16))
            lbl.grid(row=0, column=2 * i)
            tk.Label(self.ph_frame, text=ph, font=("Arial", 10)).grid(row=0, column=2 * i + 1, padx=(0, 8))
