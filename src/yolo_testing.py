import tkinter as tk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO
import numpy as np
from tkinter import simpledialog
import serial
import json
import time
import sys
import glob
import serial.tools.list_ports
from collections import deque, Counter
import torch
import threading


class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO Webcam Segmentation")
        
        # Load YOLO model
        self.model = YOLO("/Users/jasper/Desktop/blueberry sorter/BlueberryJam/runs/detect/train4/weights/best.pt")
        
        # Initialize webcam
        cv2.OPENCV_VIDEOIO_DEBUG=1
        self.camera_index = 0
        self.cap_backend = cv2.CAP_AVFOUNDATION  # Use AVFoundation backend for Mac
        self.cap = cv2.VideoCapture(self.camera_index, self.cap_backend)
        
        if not self.cap.isOpened():
            raise Exception("Webcam not accessible")
        
        # get camera type
        camera_type = self.cap.get(cv2.CAP_PROP_BACKEND)
        print(f"Using unknown backend: {camera_type}")

        # Set manual exposure mode (disable auto exposure)
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual, 0.75 = auto
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # auto mode
        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode

        time.sleep(1)

        print (self.cap.get(cv2.CAP_PROP_EXPOSURE))

        # Define target resolution
        self.camera_x = 1920
        self.camera_y = 1080
        self.scale_feed = 0.5
        self.target_res = (round(self.camera_x*self.scale_feed), round(self.camera_y*self.scale_feed))
        self.exposure_unsupported = False  # Now we try to support exposure
        self.exposure = 0.5  # Default exposure value (0-1 range)
        self.capture_device = None

        # Define bounding boxes [[TopCornerX, TopCornerY, BottomCornerX, BottomCornerY], ...]
        self.bboxes = [
            [468, 36, 812, 534], # Box 1
            [204, 40, 444, 530], # Box 2
            [15, 202, 190, 537], # Box 3
        ]
        
        # Initialize state for each bounding box
        self.bbox_states = [[] for _ in self.bboxes]

        # keyboard bindings
        self.root.bind("<Escape>", lambda e: self.root.quit())
        self.root.bind("<Delete>", self.delete_selected_bbox)
        self.root.bind("<BackSpace>", self.delete_selected_bbox)
        self.root.bind("<Control-d>", lambda e: self.toggle_draw_mode())
        self.root.bind("<Control-e>", lambda e: self.toggle_edit_mode())
        self.root.bind("<Control-c>", lambda e: self.capture())
        self.root.bind("<Control-r>", lambda e: self.clear())
        self.root.bind("<Control-a>", lambda e: self.toggle_auto_capture())
        
        # GUI layout: two columns (controls, feedback/state)
        main_frame = tk.Frame(root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Controls column
        controls_frame = tk.LabelFrame(main_frame, text="Controls", padx=10, pady=10)
        controls_frame.grid(row=0, column=0, sticky="nsw", padx=10, pady=10)
        controls_frame.grid_columnconfigure(0, weight=1)

        # Feedback/State column
        feedback_frame = tk.LabelFrame(main_frame, text="Feedback / State", padx=10, pady=10)
        feedback_frame.grid(row=0, column=1, sticky="nsew", padx=10, pady=10)
        main_frame.grid_columnconfigure(0, weight=0)
        main_frame.grid_columnconfigure(1, weight=1)
        main_frame.grid_rowconfigure(0, weight=1)

        # Canvas (video feed) at the top of feedback column
        canvas_label = tk.Label(feedback_frame, text="Camera Feed")
        canvas_label.grid(row=1, column=0, sticky="w")
        self.canvas = tk.Canvas(feedback_frame, width=self.target_res[0], height=self.target_res[1])
        self.canvas.grid(row=2, column=0, pady=(0, 10))

        # --- Controls (stacked vertically, let widgets wrap/expand as needed) ---

        serial_frame = tk.Frame(controls_frame)
        serial_frame.pack(fill=tk.X, pady=2)
        tk.Label(serial_frame, text="Serial:").pack(side=tk.LEFT)

        # --- Serial Status Container ---
        serial_status_container = tk.Frame(feedback_frame)
        serial_status_container.grid(row=0, column=0, sticky="nw")

        # Status label for connection
        self.serial_status_var = tk.StringVar(value="Disconnected")
        self.serial_status_label = tk.Label(serial_status_container, textvariable=self.serial_status_var, fg="red", anchor="w", justify="left")
        self.serial_status_label.grid(row=0, column=0, sticky="w")

        # PH trigger setup
        self.ph_triggers_received = set()
        self.expected_ph_triggers = {"PH1", "PH2", "PH3"}
        #self.root.after(100, self.serial_listener)

        # --- PH trigger emoji indicators ---
        self.ph_states = {"PH1": False, "PH2": False, "PH3": False}
        self.ph_emoji_labels = {}
        self.ph_frame = tk.Frame(serial_status_container)
        self.ph_frame.grid(row=0, column=1, sticky="w", padx=(10,0))

        self.update_ph_emoji_labels()

        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        for port in sorted(ports, key=lambda p: p.device):
            print("{}: {} [{}]".format(port.device, port.description, port.hwid))
        self.serial_port_addr = port_list[0] if port_list else ''
        self.serial_port_var = tk.StringVar(value=self.serial_port_addr)

        self.serial_port_dropdown = tk.OptionMenu(serial_frame, self.serial_port_var, *port_list)
        self.serial_port_dropdown.config(width=25)
        self.serial_port_dropdown.pack(side=tk.LEFT, padx=2)

        self.btn_refresh_ports = tk.Button(serial_frame, text="Refresh", command=self.refresh_ports)
        self.btn_refresh_ports.pack(side=tk.LEFT, padx=2)

        self.btn_connect_serial = tk.Button(serial_frame, text="Connect", command=self.connect_serial)
        self.btn_connect_serial.pack(side=tk.LEFT, padx=2)

        self.serial_port = None
        #self.connect_serial()

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Actions
        btn_frame = tk.Frame(controls_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        tk.Label(btn_frame, text="Actions:").pack(side=tk.LEFT)
        self.btn_capture = tk.Button(btn_frame, text="Capture & Segment", command=self.capture)
        self.btn_capture.pack(side=tk.LEFT, padx=2)
        self.btn_clear = tk.Button(btn_frame, text="Clear", command=self.clear)
        self.btn_clear.pack(side=tk.LEFT, padx=2)

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Bounding Boxes
        bbox_frame = tk.Frame(controls_frame)
        bbox_frame.pack(fill=tk.X, pady=2)
        tk.Label(bbox_frame, text="Bounding Boxes:").pack(side=tk.LEFT)

        # Toggle switches using Checkbutton widgets
        self.draw_mode = False
        self.edit_mode = False

        def draw_toggle_command():
            self.draw_mode = not self.draw_mode
            if self.draw_mode:
                self.edit_mode = False
                self.edit_toggle.deselect()

        def edit_toggle_command():
            self.edit_mode = not self.edit_mode
            if self.edit_mode:
                self.draw_mode = False
                self.draw_toggle.deselect()

        self.draw_toggle = tk.Checkbutton(
            bbox_frame, text="Draw", indicatoron=True,
            command=draw_toggle_command
        )
        self.draw_toggle.pack(side=tk.LEFT, padx=2)

        self.edit_toggle = tk.Checkbutton(
            bbox_frame, text="Edit", indicatoron=True,
            command=edit_toggle_command
        )
        self.edit_toggle.pack(side=tk.LEFT, padx=2)

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Exposure
        exposure_frame = tk.Frame(controls_frame)
        exposure_frame.pack(fill=tk.X, pady=2)
        tk.Label(exposure_frame, text="Exposure:").pack(side=tk.LEFT)
        self.exposure_label = tk.Label(exposure_frame, text="Supported" if not self.exposure_unsupported else "Unsupported")
        self.exposure_label.pack(side=tk.LEFT, padx=2)
        self.exposure_scale = tk.Scale(exposure_frame, from_=0.0, to=1.0, resolution=0.1, orient=tk.HORIZONTAL, 
                  command=self.update_exposure, length=120)
        self.exposure_scale.set(self.exposure)
        self.exposure_scale.pack(side=tk.LEFT, padx=2)
        if self.exposure_unsupported:
            self.exposure_scale.config(state='disabled')

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Auto Capture
        auto_frame = tk.Frame(controls_frame)
        auto_frame.pack(fill=tk.X, pady=2)
        tk.Label(auto_frame, text="Auto Capture:").grid(row=0, column=0, sticky="w")
        self.auto_capture_running = False
        self.auto_capture_interval = tk.IntVar(value=5)
        self.auto_capture_dir = tk.StringVar(value="/Users/jasper/Desktop/blueberry sorter/BlueberryJam/auto_captures")
        self.entry_interval = tk.Entry(auto_frame, textvariable=self.auto_capture_interval, width=5)
        self.entry_interval.grid(row=0, column=1, padx=2)
        tk.Label(auto_frame, text="seconds").grid(row=0, column=2, sticky="w")
        self.entry_dir = tk.Entry(auto_frame, textvariable=self.auto_capture_dir, width=30)
        self.entry_dir.grid(row=1, column=0, columnspan=3, sticky="we", padx=2, pady=(2,0))
        self.btn_auto_capture = tk.Button(auto_frame, text="Start Auto Capture", command=self.toggle_auto_capture)
        self.btn_auto_capture.grid(row=0, column=3, rowspan=2, padx=2, sticky="ns")
        auto_frame.grid_columnconfigure(0, weight=0)
        auto_frame.grid_columnconfigure(1, weight=0)
        auto_frame.grid_columnconfigure(2, weight=0)
        auto_frame.grid_columnconfigure(3, weight=0)

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Motor Control
        motor_frame = tk.Frame(controls_frame)
        motor_frame.pack(fill=tk.X, pady=2)
        tk.Label(motor_frame, text="Conveyor Speed:").grid(row=0, column=0, sticky="w")
        self.conveyor_speed = tk.IntVar(value=5)
        self.slider_conveyor = tk.Scale(
            motor_frame, from_=0, to=9, orient=tk.HORIZONTAL, variable=self.conveyor_speed,
            showvalue=True, length=120
        )
        self.slider_conveyor.grid(row=0, column=1, padx=2, sticky="w")
        motor_frame.grid_columnconfigure(0, weight=0)
        motor_frame.grid_columnconfigure(1, weight=1)

        tk.Frame(controls_frame, height=2, bg="#cccccc").pack(fill=tk.X, pady=4)  # Separator

        # Carousel and ejection ports
        self.carousel_slots = deque([None]*20, maxlen=20)  # 20-slot queue
        self.eject_ports = {"RIPE": 5, "UNDERRIPE": 8, "OVERRIPE": 11}
        self.class_labels = ["RIPE", "UNDERRIPE", "OVERRIPE"]

        # --- Add Carousel Status Canvas at the bottom of controls column ---
        self.carousel_canvas = tk.Canvas(controls_frame, width=400, height=400, highlightthickness=0)
        self.carousel_canvas.pack(side=tk.BOTTOM, pady=(16, 0))
        tk.Label(controls_frame, text="Carousel Slots:").pack(side=tk.BOTTOM, pady=(0, 2))
        self.draw_carousel_status()  # Initial draw

        # --- Feedback/State output (row by row) ---
        state_label = tk.Label(feedback_frame, text="Bounding Box States:")
        state_label.grid(row=3, column=0, sticky="w", pady=(10, 0))
        self.label_states = tk.Label(feedback_frame, text="[]", wraplength=600, anchor="w", justify="left")
        self.label_states.grid(row=4, column=0, sticky="w")
        
        # State variables
        self.show_segmented = False
        self.segmented_image = None
        self.start_x = None
        self.start_y = None
        self.temp_bbox = None
        self.temp_rect_id = None
        self.bbox_items = []  # Store canvas rectangle and text IDs
        self.image_id = None  # Store canvas image ID
        self.selected_bbox_idx = None
        self.drag_mode = None  # 'move', 'top-left', 'top-right', 'bottom-left', 'bottom-right'
        self.proximity = 10  # Pixels for edge/corner detection
        
        # Bind mouse events for drawing and editing
        self.canvas.bind("<Button-1>", self.start_action)
        self.canvas.bind("<B1-Motion>", self.update_action)
        self.canvas.bind("<ButtonRelease-1>", self.end_action)
        
        # Display initial webcam feed
        self.update_feed()

        self.serial_thread = None
        self.serial_thread_running = False
        self.start_serial_thread()

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
                            self.root.after(0, self.update_ph_emoji_labels)  # Schedule UI update
                            if self.ph_triggers_received == self.expected_ph_triggers:
                                self.root.after(0, self.classify_and_update_queue)  # Schedule classification
                                self.ph_triggers_received.clear()
                except Exception as e:
                    print(f"Serial read error: {e}")
            else:
                time.sleep(0.05)  # Sleep briefly to reduce CPU usage when port is not open

    # Get available serial ports (populate once, refresh only on button click)
    def refresh_ports(self):
        ports = serial.tools.list_ports.comports()
        port_list = [port.device for port in ports]
        menu = self.serial_port_dropdown["menu"]
        menu.delete(0, "end")
        for port in port_list:
            menu.add_command(label=port, command=lambda value=port: self.serial_port_var.set(value))
        if port_list:
            self.serial_port_var.set(port_list[0])
        else:
            self.serial_port_var.set('')
    
    def connect_serial(self):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
                self.serial_port = None
            self.serial_port_addr = self.serial_port_var.get()
            if not self.serial_port_addr:
                self.serial_status_var.set("No port selected")
                self.serial_status_label.config(fg="red")
                self.serial_port = None
                return
            self.serial_port = serial.Serial(self.serial_port_addr, baudrate=9600, timeout=1)
            self.serial_status_var.set("Connected")
            self.serial_status_label.config(fg="green")
        except serial.SerialException as e:
            self.serial_status_var.set("Disconnected")
            self.serial_status_label.config(fg="red")
            print(f"Failed to open serial port: {e}")
    
    def toggle_auto_capture(self):
        if not self.auto_capture_running:
            self.auto_capture_running = True
            self.btn_auto_capture.config(text="Stop Auto Capture")
            self.auto_capture()
        else:
            self.auto_capture_running = False
            self.btn_auto_capture.config(text="Start Auto Capture")

    def auto_capture(self):
        if not self.auto_capture_running:
            return
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, self.target_res)
            directory = self.auto_capture_dir.get()
            import os
            os.makedirs(directory, exist_ok=True)
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"{directory}/capture_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Saved: {filename}")
        interval = self.auto_capture_interval.get()
        self.root.after(max(1000, int(interval * 1000)), self.auto_capture)

    # def update_uvc_control(self, control_name, value):
    #     if self.uvc_device and control_name in self.uvc_controls:
    #         try:
    #             self.uvc_controls[control_name].value = int(value)
    #         except Exception as e:
    #             print(f"Failed to set {control_name}: {e}")
    
    # def check_exposure_support_AVF(self):
    #             # Initialize AVFoundation for exposure control
    #     if AVFOUNDATION_AVAILABLE:
    #         devices = AVCaptureDevice.devices()
    #         print(f"Found {len(devices)} AVFoundation devices")
    #         for device in devices:
    #             if device.hasMediaType_('vide'):
    #                 self.capture_device = device
    #                 print(f"Using AVFoundation device: {device.localizedName()}")
    #                 break
    #         if self.capture_device:
    #             try:
    #                 self.set_exposure_avf(self.exposure)
    #                 # Check if exposure adjustment is supported
    #                 if self.capture_device.isExposureModeSupported_(0):  # AVCaptureExposureModeContinuousAutoExposure
    #                     print("Exposure adjustment lock supported")
    #                     self.exposure_unsupported = False
    #                 elif self.capture_device.isExposureModeSupported_(1):
    #                     print("autoExpose supported")
    #                     self.exposure_unsupported = False
    #                 elif self.capture_device.isExposureModeSupported_(2):
    #                     print("ContinuousAutoExpose supported")
    #                     self.exposure_unsupported = False
    #                 elif self.capture_device.isExposureModeSupported_(3):
    #                     print("custom exposure supported")
    #                     self.exposure_unsupported = False
    #                 else:
    #                     print("Exposure adjustment not supported")
    #                     #self.exposure_unsupported = True
    #             except:
    #                 pass

    # def set_exposure_avf(self, value):
    #     if self.capture_device and not self.exposure_unsupported:
    #         try:
    #             # Lock device for configuration
    #             self.capture_device.lockForConfiguration_(None)
    #             # Map 0-1 range to camera's exposure range
    #             min_exposure = self.capture_device.minExposureTargetBias
    #             max_exposure = self.capture_device.maxExposureTargetBias
    #             exposure_value = min_exposure + (max_exposure - min_exposure) * value
    #             self.capture_device.setExposureTargetBias_(exposure_value)
    #             self.capture_device.unlockForConfiguration()
    #         except:
    #             self.exposure_unsupported = True
    #             self.exposure_label.config(text="Exposure: Unsupported")
    #             self.exposure_scale.config(state='disabled')
    
    def update_exposure(self, value):
        self.exposure = float(value)
        # Also set exposure via OpenCV (if supported)
        if hasattr(self, 'cap') and self.cap is not None:
            # Map normalized value (0-1) to a typical exposure range, e.g., -8 to -1 for many webcams
            min_exp, max_exp = 0.01, 1
            exp_val = float(min_exp + (max_exp - min_exp) * self.exposure)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exp_val)

    def send_serial_data(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                # convert bbox_states to json and send
                data = json.dumps(self.bbox_states)+'\n'
                self.serial_port.write(data.encode('utf-8'))
            except serial.SerialException as e:
                print(f"serial write error: {e}")
        # Schedule next transmission
        # self.root.after(3000, self.send_serial_data)
    
    def start_action(self, event):
        if self.draw_mode:
            self.start_x, self.start_y = event.x, event.y
            self.temp_bbox = [self.start_x, self.start_y, self.start_x, self.start_y]
            self.temp_rect_id = self.canvas.create_rectangle(self.temp_bbox, outline="red", width=2)
        elif self.edit_mode:
            self.selected_bbox_idx, self.drag_mode = self.find_bbox_edge_or_corner(event.x, event.y)
            if self.selected_bbox_idx is not None:
                self.start_x, self.start_y = event.x, event.y
    
    def find_bbox_edge_or_corner(self, x, y):
        for i, bbox in enumerate(self.bboxes):
            x1, y1, x2, y2 = bbox
            # Check corners
            if abs(x - x1) <= self.proximity and abs(y - y1) <= self.proximity:
                return i, "top-left"
            if abs(x - x2) <= self.proximity and abs(y - y1) <= self.proximity:
                return i, "top-right"
            if abs(x - x1) <= self.proximity and abs(y - y2) <= self.proximity:
                return i, "bottom-left"
            if abs(x - x2) <= self.proximity and abs(y - y2) <= self.proximity:
                return i, "bottom-right"
            # Check edges
            if abs(y - y1) <= self.proximity and x1 <= x <= x2:
                return i, "move"
            if abs(y - y2) <= self.proximity and x1 <= x <= x2:
                return i, "move"
            if abs(x - x1) <= self.proximity and y1 <= y <= y2:
                return i, "move"
            if abs(x - x2) <= self.proximity and y1 <= y <= y2:
                return i, "move"
            # Check center region (set as move if clicked inside bbox but not near edge/corner)
            center_margin_x = (x2 - x1) * 0.2
            center_margin_y = (y2 - y1) * 0.2
            if (x1 + center_margin_x < x < x2 - center_margin_x and
                y1 + center_margin_y < y < y2 - center_margin_y):
                return i, "move"
        return None, None
    
    def update_action(self, event):
        if self.draw_mode and self.start_x is not None:
            self.temp_bbox[2], self.temp_bbox[3] = event.x, event.y
            self.canvas.coords(self.temp_rect_id, self.temp_bbox)
        elif self.edit_mode and self.selected_bbox_idx is not None:
            dx, dy = event.x - self.start_x, event.y - self.start_y
            bbox = self.bboxes[self.selected_bbox_idx]
            if self.drag_mode == "move":
                # Move entire box
                bbox[0] += dx
                bbox[1] += dy
                bbox[2] += dx
                bbox[3] += dy
            elif self.drag_mode == "top-left":
                bbox[0] += dx
                bbox[1] += dy
            elif self.drag_mode == "top-right":
                bbox[2] += dx
                bbox[1] += dy
            elif self.drag_mode == "bottom-left":
                bbox[0] += dx
                bbox[3] += dy
            elif self.drag_mode == "bottom-right":
                bbox[2] += dx
                bbox[3] += dy
            # Ensure coordinates are within canvas
            bbox[0] = max(0, min(bbox[0], self.target_res[0]))
            bbox[1] = max(0, min(bbox[1], self.target_res[1]))
            bbox[2] = max(0, min(bbox[2], self.target_res[0]))
            bbox[3] = max(0, min(bbox[3], self.target_res[1]))
            # Ensure x1 <= x2 and y1 <= y2
            if bbox[0] > bbox[2]:
                bbox[0], bbox[2] = bbox[2], bbox[0]
            if bbox[1] > bbox[3]:
                bbox[1], bbox[3] = bbox[3], bbox[1]
            self.bboxes[self.selected_bbox_idx] = bbox
            self.draw_bboxes()
            self.start_x, self.start_y = event.x, event.y
            # Print all bbox coordinates after adjustment
    
    def end_action(self, event):
        if self.draw_mode and self.start_x is not None:
            end_x, end_y = event.x, event.y
            x1, x2 = min(self.start_x, end_x), max(self.start_x, end_x)
            y1, y2 = min(self.start_y, end_y), max(self.start_y, end_y)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(self.target_res[0], x2), min(self.target_res[1], y2)
            new_bbox = [x1, y1, x2, y2]
            index = simpledialog.askinteger("Input", "Enter index for this bounding box (0 to append, 1 to insert at start, etc.):", 
                                          parent=self.root, minvalue=0)
            if index is not None:
                if index == 0 or index >= len(self.bboxes):
                    self.bboxes.append(new_bbox)
                    self.bbox_states.append([])
                else:
                    self.bboxes.insert(index, new_bbox)
                    self.bbox_states.insert(index, [])
                self.draw_bboxes()
            self.canvas.delete(self.temp_rect_id)
            self.start_x = None
            self.start_y = None
            self.temp_bbox = None
            self.temp_rect_id = None
            self.toggle_draw_mode()
        elif self.edit_mode and self.selected_bbox_idx is not None:
            self.selected_bbox_idx = None
            self.drag_mode = None
            self.start_x = None
            self.start_y = None
        
        if self.edit_mode or self.draw_mode:
            print("Updated bounding boxes:")
            for i, b in enumerate(self.bboxes):
                print(f"{b}, #Box {i}")
    
    def draw_bboxes(self):
        # Clear existing bounding box items
        for rect_id, text_id, hover_id in getattr(self, "bbox_items", []):
            self.canvas.delete(rect_id)
            self.canvas.delete(text_id)
            self.canvas.delete(hover_id)
        self.bbox_items = []

        for i, bbox in enumerate(self.bboxes):
            # Highlight selected bbox in yellow, others in green
            outline_color = "yellow" if i == self.selected_bbox_idx else "green"
            rect_id = self.canvas.create_rectangle(bbox, outline=outline_color, width=2, tags=f"bbox_{i}_rect")
            text_id = self.canvas.create_text(bbox[2] - 10, bbox[3] - 10, text=str(i), fill="white", anchor="se", tags=f"bbox_{i}_text")
            hover_id = self.canvas.create_rectangle(
                bbox,
                outline="", fill="",
                tags=f"bbox_{i}_hover"
            )
            self.canvas.tag_raise(text_id)
            self.canvas.tag_raise(rect_id, hover_id)
            self.bbox_items.append((rect_id, text_id, hover_id))

            # Bind hover and click events to the transparent rectangle
            self.canvas.tag_bind(f"bbox_{i}_hover", "<Enter>", lambda e, idx=i: self.on_bbox_hover(idx, True))
            self.canvas.tag_bind(f"bbox_{i}_hover", "<Leave>", lambda e, idx=i: self.on_bbox_hover(idx, False))
            self.canvas.tag_bind(f"bbox_{i}_hover", "<Button-1>", lambda e, idx=i: self.on_bbox_click(idx))

    def on_bbox_hover(self, index, enter):
        # print(f"{'Entered' if enter else 'Exited'} bbox {index}")
        # Only highlight yellow if not already selected
        rect_id, _, _ = self.bbox_items[index]
        if index == self.selected_bbox_idx:
            color = "yellow"
        else:
            color = "yellow" if enter else "green"
        self.canvas.itemconfig(rect_id, outline=color)
        
        if self.draw_mode or self.edit_mode:
            self.selected_bbox_idx = index

    def on_bbox_click(self, index):
        # Only allow selection in draw or edit mode
        # print(f"Clicked on bbox {index}")
        if self.draw_mode or self.edit_mode:
            self.selected_bbox_idx = index
            # Redraw to update highlight
            self.draw_bboxes()

    def delete_selected_bbox(self, event=None):
        if self.draw_mode or self.edit_mode and self.selected_bbox_idx is not None:
            idx = self.selected_bbox_idx
            if 0 <= idx < len(self.bboxes):
                del self.bboxes[idx]
                del self.bbox_states[idx]
                self.selected_bbox_idx = None
                self.draw_bboxes()
    
    # def on_bbox_click(self, index):
    #     # Set the clicked bbox as the selected bbox and redraw
    #     self.selected_bbox_idx = index
    #     self.draw_bboxes()
    
    def update_feed(self):
        if not self.show_segmented:
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, self.target_res)
                frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                self.photo = ImageTk.PhotoImage(image=Image.fromarray(frame))
                if self.image_id is None:
                    self.image_id = self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
                else:
                    self.canvas.itemconfig(self.image_id, image=self.photo)
                self.draw_bboxes()
        elif self.segmented_image is not None:
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(self.segmented_image))
            if self.image_id is None:
                self.image_id = self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            else:
                self.canvas.itemconfig(self.image_id, image=self.photo)
            self.draw_bboxes()
        self.root.after(10, self.update_feed)
    
    def capture(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, self.target_res)
            self.bbox_states = [[] for _ in self.bboxes]
            results = self.model(frame)
            for result in results:
                if result.boxes and result.boxes.xyxy is not None:
                    boxes = result.boxes.xyxy.cpu().numpy()
                    classes = result.boxes.cls.cpu().numpy()
                    class_names = result.names
                    for box, cls in zip(boxes, classes):
                        x1, y1, x2, y2 = box[:4]
                        centroid_x = (x1 + x2) / 2
                        centroid_y = (y1 + y2) / 2
                        for i, bbox in enumerate(self.bboxes):
                            bx1, by1, bx2, by2 = bbox
                            if bx1 <= centroid_x <= bx2 and by1 <= centroid_y <= by2:
                                label = class_names[int(cls)]
                                if label not in self.bbox_states[i]:
                                    self.bbox_states[i].append(label)
            
            state_text = "Bounding Box States: " + str(self.bbox_states)
            self.label_states.config(text=state_text)
            
            annotated_frame = results[0].plot()
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            annotated_frame = cv2.resize(annotated_frame, self.target_res)
            self.segmented_image = annotated_frame
            self.show_segmented = True
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(self.segmented_image))
            if self.image_id is None:
                self.image_id = self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            else:
                self.canvas.itemconfig(self.image_id, image=self.photo)
            self.draw_bboxes()
            self.classify_and_update_queue()
    
    def clear(self):
        self.show_segmented = False
        self.segmented_image = None
        self.bbox_states = [[] for _ in self.bboxes]
        #self.label_states.config(text="Bounding Box States: []")
        self.draw_bboxes()
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        self.stop_serial_thread()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

    def draw_carousel_status(self):
        """Draws the circular carousel status display."""
        self.carousel_canvas.delete("all")
        n = len(self.carousel_slots)
        cx, cy = 200, 200  # Center of the canvas
        r = 150            # Radius of the circle
        circle_r = 10     # Radius of each slot circle

        color_map = {
            "RIPE": "#b3e0ff",        # Light blue
            "UNDERRIPE": "#baffc9",   # Light green
            "OVERRIPE": "#ffb3b3",    # Light red
            None: "#eeeeee"           # Empty slot
        }
        outline_map = {
            "RIPE": "#3399ff",
            "UNDERRIPE": "#33cc66",
            "OVERRIPE": "#ff6666",
            None: "#cccccc"
        }

        for i, label in enumerate(self.carousel_slots):
            angle = 2 * np.pi * i / n - np.pi/2  # Start at top
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            fill = color_map.get(label, "#eeeeee")
            outline = outline_map.get(label, "#cccccc")
            self.carousel_canvas.create_oval(
                x - circle_r, y - circle_r, x + circle_r, y + circle_r,
                fill=fill, outline=outline, width=2
            )
            self.carousel_canvas.create_text(
                x, y, text=str(i+1), fill="#222", font=("Arial", 10, "bold")
            )
            # Optionally, show label as small text below
            if label:
                self.carousel_canvas.create_text(
                    x, y + circle_r + 8, text=label[0], fill="#555", font=("Arial", 8)
                )
        
        # Draw eject port triangles
        triangle_size = 18
        port_colors = {
            "RIPE": "#3399ff",        # Blue
            "UNDERRIPE": "#33cc66",   # Green
            "OVERRIPE": "#ff6666",    # Red
        }
        for label, pos in self.eject_ports.items():
            idx = pos - 1  # 0-based
            angle = 2 * np.pi * idx / n - np.pi/2
            # Triangle tip is triangle_size away from the slot circle edge
            tip_dist = r + circle_r + triangle_size
            base_dist = r + circle_r + triangle_size * 0.4  # base closer to slot
            angle_offset = np.pi / 72 # base width
            x_tip = cx + tip_dist * np.cos(angle)
            y_tip = cy + tip_dist * np.sin(angle)
            x_base1 = cx + base_dist * np.cos(angle - angle_offset)
            y_base1 = cy + base_dist * np.sin(angle - angle_offset)
            x_base2 = cx + base_dist * np.cos(angle + angle_offset)
            y_base2 = cy + base_dist * np.sin(angle + angle_offset)
            self.carousel_canvas.create_polygon(
            [(x_tip, y_tip), (x_base1, y_base1), (x_base2, y_base2)],
            fill=port_colors[label], outline="#222", width=2
            )

    def classify_and_update_queue(self):
        # 1. Take picture and classify slots 0, 1, 2
        ret, frame = self.cap.read()
        if not ret:
            print("Camera read failed")
            return

        frame = cv2.resize(frame, self.target_res)
        results = self.model(frame)
        slot_classifications = [None, None, None]

        # For each bbox (slots 0,1,2), get the most confident class
        for i, bbox in enumerate(self.bboxes[:3]):
            slot_classifications[i] = self.classify_bbox(results, bbox)

        # 2. Average for slot 3 (weighted)
        if all(slot_classifications):
            # Example: slot 1 gets double weight
            counts = Counter()
            counts[slot_classifications[0]] += 1
            counts[slot_classifications[1]] += 2
            counts[slot_classifications[2]] += 1
            best_class = counts.most_common(1)[0][0]
        else:
            best_class = None

        # 3. Add to queue at position 3 (simulate rotation)
        self.carousel_slots.append(best_class)

        # 4. Check ejection ports
        eject_array = [0, 0, 0]
        for idx, label in enumerate(self.class_labels):
            port_pos = self.eject_ports[label]
            slot_val = self.carousel_slots[port_pos-1]  # -1 because 0-based
            if slot_val == label:
                eject_array[idx] = 1

        # 5. Send serial commands
        self.send_eject_serial(eject_array)
        #self.send_next_serial()

        print(f"Classified: {slot_classifications}, Added: {best_class}, Queue: {list(self.carousel_slots)}")
        print(f"Eject: {eject_array}")
        self.draw_carousel_status()

    def classify_bbox(self, results, bbox):
        # Find the most confident class in bbox
        for result in results:
            if result.boxes and result.boxes.xyxy is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                class_names = result.names
                for box, cls, score in zip(boxes, classes, scores):
                    x1, y1, x2, y2 = box[:4]
                    centroid_x = (x1 + x2) / 2
                    centroid_y = (y1 + y2) / 2
                    bx1, by1, bx2, by2 = bbox
                    if bx1 <= centroid_x <= bx2 and by1 <= centroid_y <= by2:
                        return class_names[int(cls)]
        return None

    def send_eject_serial(self, eject_array):
        if self.serial_port and self.serial_port.is_open:
            try:
                # Convert eject_array (e.g., [0, 0, 1]) to string "001"
                speed = self.conveyor_speed.get() #if hasattr(self.conveyor_speed, "get") else self.conveyor_speed
                data = ''.join(str(x) for x in eject_array) + '1' + str(speed) + str(speed) + '\n'
                self.serial_port.write(data.encode('utf-8'))
                print(f"Sent eject command: {data.strip()}")
            except serial.SerialException as e:
                print(f"serial write error: {e}")

    def send_next_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"NEXT\n")
            except serial.SerialException as e:
                print(f"serial write error: {e}")

    def serial_listener(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting:
                    line = self.serial_port.readline().decode('utf-8').strip()
                    if line in self.expected_ph_triggers:
                        print(f"Received trigger: {line}")
                        self.ph_triggers_received.add(line)
                        if self.ph_triggers_received == self.expected_ph_triggers:
                            self.classify_and_update_queue()
                            self.ph_triggers_received.clear()
            except Exception as e:
                print(f"Serial read error: {e}")
        # self.root.after(100, self.serial_listener)
    
    def update_ph_emoji_labels(self):
        for i, ph in enumerate(self.expected_ph_triggers):
            # Show sun if received, moon if not
            emoji = "🌞" if ph in self.ph_triggers_received else "🌚"
            lbl = tk.Label(self.ph_frame, text=emoji, font=("Arial", 16))
            lbl.grid(row=0, column=2 * i)
            tk.Label(self.ph_frame, text=ph, font=("Arial", 10)).grid(row=0, column=2 * i + 1, padx=(0, 8))

if __name__ == "__main__":
    root = tk.Tk()
    app = WebcamApp(root)
    root.mainloop()