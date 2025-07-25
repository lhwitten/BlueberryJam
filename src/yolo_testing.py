import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO
import numpy as np
import time
import sys
from collections import deque, Counter
import torch

# Import our custom modules
from bbox_manager import BoundingBoxManager
from serial_manager import SerialManager, SerialControlWidget, PHIndicatorWidget
from carousel_widget import CarouselStatusWidget, Blueberry, StatisticsWidget

class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO Webcam Segmentation")
        
        # Load YOLO model
        # self.model = YOLO("/Users/jasper/Desktop/blueberry sorter/BlueberryJam/runs/detect/train4/weights/best.pt")
        self.model = YOLO("/Users/jasper/Desktop/blueberry sorter/BlueberryJam/runs/segment/train5/weights/best.pt")
        
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
        self.scale_feed = 0.45
        self.target_res = (round(self.camera_x*self.scale_feed), round(self.camera_y*self.scale_feed))
        self.exposure_unsupported = False  # Now we try to support exposure
        self.exposure = 0.5  # Default exposure value (0-1 range)
        self.capture_device = None

        # Initialize classification tracking
        self.class_labels = ["RIPE", "UNDERRIPE", "OVERRIPE"]
        self.slot_classification_history = deque(maxlen=3)  # Track 3 positions by default
        self.current_slot_classifications = []

        # keyboard bindings
        self.root.bind("<Escape>", lambda e: self.root.quit())
        self.root.bind("<Delete>", self.delete_selected_bbox)
        self.root.bind("<BackSpace>", self.delete_selected_bbox)
        self.root.bind("<Control-d>", lambda e: self.toggle_draw_mode())
        self.root.bind("<Control-e>", lambda e: self.toggle_edit_mode())
        self.root.bind("<Control-c>", lambda e: self.classify_and_update_queue())
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

        # Initialize modules
        self.bbox_manager = BoundingBoxManager(self.canvas, self.target_res)
        self.bbox_states = [[] for _ in range(self.bbox_manager.get_bbox_count())]  # Initialize with empty state per bbox
        self.serial_manager = SerialManager(root)
        
        # Set up callbacks
        # remove for now for maula control
        # self.serial_manager.set_trigger_callback(self.classify_and_update_queue)
        self.bbox_manager.set_bbox_count_changed_callback(self.on_bbox_count_changed)

        # --- Controls (stacked vertically, let widgets wrap/expand as needed) ---

        # Serial controls
        self.serial_control_widget = SerialControlWidget(controls_frame, self.serial_manager)

        # --- Serial Status Container ---
        serial_status_container = tk.Frame(feedback_frame)
        serial_status_container.grid(row=0, column=0, sticky="nw")

        # PH indicator widget
        self.ph_indicator = PHIndicatorWidget(serial_status_container, self.serial_manager)

        # Start serial thread
        self.serial_manager.start_serial_thread()

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # Actions
        btn_frame = tk.Frame(controls_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        tk.Label(btn_frame, text="Actions:").pack(side=tk.LEFT)
        self.btn_capture = ttk.Button(btn_frame, text="Capture", command=self.capture)
        self.btn_capture.pack(side=tk.LEFT, padx=2)
        self.btn_capture = ttk.Button(btn_frame, text="Live", command=self.toggle_live)
        self.btn_capture.pack(side=tk.LEFT, padx=2)
        self.btn_capture = ttk.Button(btn_frame, text="Step Sort", command=self.classify_and_update_queue)
        self.btn_capture.pack(side=tk.LEFT, padx=2)
        self.btn_clear = ttk.Button(btn_frame, text="clear", command=self.clear)
        self.btn_clear.pack(side=tk.LEFT, padx=2)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)


        # Bounding Boxes
        bbox_frame = tk.Frame(controls_frame)
        bbox_frame.pack(fill=tk.X, pady=2)
        tk.Label(bbox_frame, text="Bounding Boxes:").pack(side=tk.LEFT)

        # Toggle switches using Checkbutton widgets
        def draw_toggle_command():
            draw_mode = self.bbox_manager.toggle_draw_mode()
            if draw_mode:
                self.edit_toggle.deselect()

        def edit_toggle_command():
            edit_mode = self.bbox_manager.toggle_edit_mode()
            if edit_mode:
                self.draw_toggle.deselect()

        # Replace tk Checkbuttons with ttk Checkbuttons
        self.draw_toggle = ttk.Checkbutton(
            bbox_frame, text="Draw",
            command=draw_toggle_command
        )
        self.draw_toggle.pack(side=tk.LEFT, padx=2)

        self.edit_toggle = ttk.Checkbutton(
            bbox_frame, text="Edit",
            command=edit_toggle_command
        )
        self.edit_toggle.pack(side=tk.LEFT, padx=2)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # # Exposure
        # exposure_frame = tk.Frame(controls_frame)
        # exposure_frame.pack(fill=tk.X, pady=2)
        # tk.Label(exposure_frame, text="Exposure:").pack(side=tk.LEFT)
        # self.exposure_label = tk.Label(exposure_frame, text="Supported" if not self.exposure_unsupported else "Unsupported")
        # self.exposure_label.pack(side=tk.LEFT, padx=2)
        # self.exposure_value_label = tk.Label(exposure_frame, text="0.5")
        # self.exposure_value_label.pack(side=tk.RIGHT, padx=2)
        # # Replace tk Scale with ttk Scale for exposure
        # self.exposure_scale = ttk.Scale(exposure_frame, from_=0.0, to=1.0, orient=tk.HORIZONTAL,
        #                               command=self.update_exposure, length=120)
        # self.exposure_scale.set(self.exposure)
        # self.exposure_scale.pack(side=tk.LEFT, padx=2)
        # if self.exposure_unsupported:
        #     self.exposure_scale.config(state='disabled')

        # ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # Auto Capture
        auto_frame = tk.Frame(controls_frame)
        auto_frame.pack(fill=tk.X, pady=2)
        tk.Label(auto_frame, text="Auto Capture:").grid(row=0, column=0, sticky="w")
        self.auto_capture_running = False
        self.auto_capture_interval = tk.IntVar(value=5)
        self.auto_capture_dir = tk.StringVar(value="/Users/jasper/Desktop/blueberry sorter/BlueberryJam/auto_captures")
        self.entry_interval = tk.Entry(auto_frame, textvariable=self.auto_capture_interval, width=5)
        self.entry_interval.grid(row=0, column=1, padx=2, sticky="ew")
        tk.Label(auto_frame, text="seconds").grid(row=0, column=2, sticky="w")
        self.entry_dir = tk.Entry(auto_frame, textvariable=self.auto_capture_dir)
        self.entry_dir.grid(row=2, column=0, columnspan=4, sticky="ew", padx=2, pady=(2, 0))
        auto_frame.grid_columnconfigure(0, weight=1)
        auto_frame.grid_columnconfigure(1, weight=1)
        auto_frame.grid_columnconfigure(2, weight=1)
        auto_frame.grid_columnconfigure(3, weight=1)
        # Replace auto capture button
        self.btn_auto_capture = ttk.Button(auto_frame, text="Start Auto Capture", command=self.toggle_auto_capture)
        self.btn_auto_capture.grid(row=0, column=3, padx=2, sticky="ns")

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # Motor Control
        motor_frame = tk.Frame(controls_frame)
        motor_frame.pack(fill=tk.X, pady=2)

        # SHAKES:
        motor_frame.pack(fill=tk.X, pady=2)
        tk.Label(motor_frame, text="shake count:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_1 = tk.IntVar(value=5)
        self.spinbox_c1 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_1, width=1)
        self.spinbox_c1.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="PPS:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_2 = tk.IntVar(value=5)
        self.spinbox_c2 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_2, width=1)
        self.spinbox_c2.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="N:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_3 = tk.IntVar(value=5)
        self.spinbox_c3 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_3, width=1)
        self.spinbox_c3.pack(side=tk.LEFT, padx=2)

        #DROP off controlls:
        tk.Label(motor_frame, text="drop PPS:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_4 = tk.IntVar(value=5)
        self.spinbox_c4 = ttk.Spinbox( motor_frame, from_=1, to=10, textvariable=self.conveyor_param_4, width=1)
        self.spinbox_c4.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="N:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_5 = tk.IntVar(value=5)
        self.spinbox_c5 = ttk.Spinbox( motor_frame, from_=1, to=10, textvariable=self.conveyor_param_5, width=1)
        self.spinbox_c5.pack(side=tk.LEFT, padx=2)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

         # run sort
        btn_frame = tk.Frame(controls_frame)
        btn_frame.pack(fill=tk.X, pady=2)
        tk.Label(btn_frame, text="Auto-Sort rate: (s)").pack(side=tk.LEFT, padx=2)
        self.classify_interval = tk.DoubleVar(value=10.1)
        self.spinbox_classify_interval = ttk.Spinbox(
            btn_frame,
            from_=0.1,
            to=60.0,
            increment=0.1,
            textvariable=self.classify_interval,
            format="%.1f",
            width=5
        )
        self.spinbox_classify_interval.pack(side=tk.LEFT, padx=2)

        self.classify_timer_running = False

        # Auto-classify timer functionality
        def start_classify_timer():
            if not self.classify_timer_running:
                self.classify_timer_running = True
                update_button_appearance(True)
                schedule_classify()
            else:
                self.classify_timer_running = False
                update_button_appearance(False)
                self.clear()

        def update_button_appearance(is_running):
            """Update button appearance using canvas drawing for guaranteed color display"""
            if is_running:
                # Running state - Red with "Stop"
                self.btn_canvas.delete("all")
                self.btn_canvas.create_rectangle(0, 0, 120, 50, fill="#ff0000", outline="#cc0000", width=2)
                self.btn_canvas.create_text(60, 25, text="STOP", font=("Arial", 12, "bold"), fill="white")
            else:
                # Stopped state - Green with "Start"
                self.btn_canvas.delete("all")
                self.btn_canvas.create_rectangle(0, 0, 120, 50, fill="#00ff00", outline="#00cc00", width=2)
                self.btn_canvas.create_text(60, 25, text="START", font=("Arial", 12, "bold"), fill="black")

        def on_button_click(event):
            start_classify_timer()

        def schedule_classify():
            if self.classify_timer_running:
                self.classify_and_update_queue()
                interval_ms = int(self.classify_interval.get() * 1000)
                self.root.after(interval_ms, schedule_classify)

        # Create a Canvas-based button that will definitely show colors
        self.btn_canvas = tk.Canvas(btn_frame, width=120, height=50, highlightthickness=0)
        self.btn_canvas.pack(side=tk.LEFT, padx=2, pady=5)
        self.btn_canvas.bind("<Button-1>", on_button_click)
        # self.btn_canvas.bind("<Enter>", lambda e: self.btn_canvas.config(cursor="hand2"))
        # self.btn_canvas.bind("<Leave>", lambda e: self.btn_canvas.config(cursor=""))
        
        # Initialize button appearance
        update_button_appearance(False)
        #add a checkbox to enable image saving during automatic classification
        self.save_images_var = tk.BooleanVar(value=False)
        self.save_images_checkbox = ttk.Checkbutton(
            btn_frame, text="Save Images", variable=self.save_images_var
        )
        self.save_images_checkbox.pack(side=tk.LEFT, padx=2)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # --- Add Carousel Status Widget at the bottom of controls column ---
        self.carousel_widget = CarouselStatusWidget(controls_frame)

        # --- Feedback/State output (row by row) ---
        self.label_states = tk.Label(feedback_frame, text="Bounding box states: \n \n \n",wraplength=800, anchor="w", justify="left")
        self.label_states.grid(row=3, column=0, sticky="w")

        self.label_eject_array = tk.Label(feedback_frame, text="Eject array: []", anchor="w", justify="left")
        self.label_eject_array.grid(row=4, column=0, sticky="w", pady=(5,0))

        self.label_serial_status = tk.Label(feedback_frame, text="Last Serial Command: None", wraplength=600, anchor="w", justify="left")
        self.label_serial_status.grid(row=5, column=0, sticky="w", pady=(5,0))
        
        # Add Statistics Widget
        self.statistics_widget = StatisticsWidget(feedback_frame, self.carousel_widget, width=feedback_frame.winfo_width())
        self.statistics_widget.grid(row=6, column=0, sticky="ew", pady=(10,0))

        # add clear statistics button
        self.btn_clear_stats = ttk.Button(feedback_frame, text="Reset Statistics", command=self.carousel_widget.reset_statistics)
        self.btn_clear_stats.grid(row=7, column=0, sticky="w", pady=(5,0)) 
        
        # State variables
        self.show_segmented = False
        self.show_live = False
        self.segmented_image = None
        self.image_id = None  # Store canvas image ID
        
        # Display initial webcam feed
        self.update_feed()

    def toggle_draw_mode(self):
        """Toggle draw mode and update UI."""
        self.bbox_manager.toggle_draw_mode()
        if self.bbox_manager.edit_mode:
            self.edit_toggle.deselect()

    def toggle_edit_mode(self):
        """Toggle edit mode and update UI."""
        self.bbox_manager.toggle_edit_mode()
        if self.bbox_manager.draw_mode:
            self.draw_toggle.deselect()

    def delete_selected_bbox(self, event=None):
        """Delete the currently selected bounding box."""
        # The bbox manager will handle the deletion and call our callback
        self.bbox_manager.delete_selected_bbox()
    
    def on_bbox_count_changed(self, new_count):
        """Callback when bbox count changes - update classification history length."""
        print(f"Bbox count changed to {new_count}, updating classification history maxlen")
        new_history = deque(self.slot_classification_history, maxlen=new_count)
        self.slot_classification_history = new_history
    
    def toggle_auto_capture(self):
        if not self.auto_capture_running:
            self.auto_capture_running = True
            self.btn_auto_capture.config(text="Stop Auto Capture")
            self.auto_capture()
        else:
            self.auto_capture_running = False
            self.btn_auto_capture.config(text="Start Auto Capture")

    def save_image(self, frame):
        """Save the given frame as an image in the specified directory."""
        directory = self.auto_capture_dir.get()
        import os
        os.makedirs(directory, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{directory}/capture_{timestamp}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved: {filename}")

    def auto_capture(self):
        """Automatically capture images at regular intervals."""
        if not self.auto_capture_running:
            return
        ret, frame = self.cap.read()
        # Send test eject command during auto capture
        self.send_eject_command_with_ui_values([1,1,1]);
        if ret:
            frame = cv2.resize(frame, self.target_res)
            self.save_image(frame)
        interval = self.auto_capture_interval.get()
        self.root.after(max(1000, int(interval * 1000)), self.auto_capture)
    
    def update_exposure(self, value):
        # Update exposure method to handle string value from ttk Scale
        self.exposure = float(value)
        self.exposure_value_label.config(text=f"{float(value):.1f}")
        # Also set exposure via OpenCV (if supported)
        if hasattr(self, 'cap') and self.cap is not None:
            # Map normalized value (0-1) to a typical exposure range, e.g., -8 to -1 for many webcams
            min_exp, max_exp = 0.01, 1
            exp_val = float(min_exp + (max_exp - min_exp) * self.exposure)
            self.cap.set(cv2.CAP_PROP_EXPOSURE, exp_val)

    # def update_conveyor_param_1(self, value):
    #     self.conveyor_param_1_label.config(text=str(int(float(value))))
    
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
                self.bbox_manager.draw_bboxes()
        elif self.segmented_image is not None:
            self.photo = ImageTk.PhotoImage(image=Image.fromarray(self.segmented_image))
            if self.image_id is None:
                self.image_id = self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
            else:
                self.canvas.itemconfig(self.image_id, image=self.photo)
            self.bbox_manager.draw_bboxes()
        self.root.after(10, self.update_feed)
    
    def capture(self):
        ret, frame = self.cap.read()
        if ret:
            frame = cv2.resize(frame, self.target_res)
            results = self.model(frame)
            self.classify_and_update_bboxes(results, add_to_carousel=False);
            self.draw_yolo_results(results)

    def reset_bbox_states(self):
        """Reset bounding box states to empty lists."""
        self.bbox_states = [[] for _ in range(self.bbox_manager.get_bbox_count())]

    def add_bboxes_states(self, index, state):
        """Update the label displaying bounding box states."""
        self.bbox_states[index].append(state)
        state_text = "Bounding Box States: " + str(self.bbox_states)
        self.label_states.config(text=state_text)

    def set_bboxes_states(self, index, state):
        """
        Update the label displaying bounding box states.
        Displays formatted class, score, and centroid for each detection.
        Example: BOX1: UNDERRIPE (0.90 x:12 y:15) | RIPE (0.70 x:11 y:13)
        """
        self.bbox_states[index] = state

        def format_detection(det):
            cls = det.get('class', 'N/A')
            score = det.get('score', 0)
            cx, cy = det.get('centroid', (0, 0))
            return f"{cls} ({score:.2f} x:{int(round(cx))} y:{int(round(cy))})"

        lines = []
        for i, detections in enumerate(self.bbox_states):
            if detections:
                det_strs = [format_detection(det) for det in detections]
                line = f"BOX{i+1}: " + " | ".join(det_strs)
            else:
                line = f"BOX{i+1}: None"
            lines.append(line)

        state_text = "Bounding Box States:\n" + "\n".join(lines)
        self.label_states.config(text=state_text)

    def draw_yolo_results(self, results):
        """Draw YOLO segmentation and box results on the canvas."""
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
        
        self.bbox_manager.draw_bboxes()

    def toggle_live(self):
        self.show_live = not self.show_live
        if self.show_live:
            self.show_live_yolo_results()

    def show_live_yolo_results(self):
        if self.show_live:
            """Continuously show YOLO results on the canvas."""
            ret, frame = self.cap.read()
            if ret:
                frame = cv2.resize(frame, self.target_res)
                results = self.model(frame)
                self.draw_yolo_results(results)
            self.root.after(10, self.show_live_yolo_results)

    def clear(self):
        """Clear the canvas and stop live view."""
        self.show_segmented = False
        self.segmented_image = None
        self.show_live = False
        self.bbox_manager.clear_states()
        self.bbox_manager.draw_bboxes()
        # self.carousel_widget.reset_statistics()
        self.statistics_widget.update_display()
        self.image_id = None  # Reset image ID to stop live view updates
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        if hasattr(self, 'serial_manager'):
            self.serial_manager.disconnect()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

    def classify_and_update_queue(self):
        # 1. Take picture and classify slots 0, 1, 2
        ret, frame = self.cap.read()
        if not ret:
            print("Camera read failed")
            return

        frame = cv2.resize(frame, self.target_res)
        results = self.model(frame)
        self.draw_yolo_results(results)

        if self.save_images_var.get():
            self.save_image(frame)
        
        # Get current classifications for visible slots
        self.classify_and_update_bboxes(results, add_to_carousel=True)

        # 2. Check ejection ports and send serial commands
        eject_array = self.carousel_widget.get_eject_array()

        self.label_eject_array.config(text=f"Eject array: {eject_array}")
        
        # 3. Send serial commands
        self.send_eject_command_with_ui_values(eject_array)
        
        # Update PH indicator display
        self.ph_indicator.update_ph_emoji_labels()
        
        # Update statistics display
        self.statistics_widget.update_display()

    def classify_and_update_bboxes(self, results, add_to_carousel=True):

        self.reset_bbox_states()  # Reset states before classifying
        bboxes = self.bbox_manager.get_bboxes()
        for i, bbox in enumerate(bboxes):
            classification = self.classify_bbox(i, results, bbox)
            if not add_to_carousel:
                continue
            if i == 0:
                print(f"add to carousel: Slot {i} classification: {classification}")
                # For the first slot, create a new Blueberry object and add it to the carousel
                self.carousel_widget.add_to_carousel(classification)
            else:
                # For subsequent slots, update the history of the existing Blueberry object
                blueberry = self.carousel_widget.carousel_slots[i]
                if blueberry:
                    print(f"update slot {i} classification: {classification}")
                    blueberry.add_classification_attempt(classification)

    def send_eject_command_with_ui_values(self, eject_array):
        conveyor_param_1 = self.conveyor_param_1.get()
        conveyor_param_2 = self.conveyor_param_2.get()
        conveyor_param_3 = self.conveyor_param_3.get()
        conveyor_param_4 = self.conveyor_param_4.get()
        conveyor_param_5 = self.conveyor_param_5.get()
        success, message = self.serial_manager.send_eject_command(eject_array, conveyor_param_1, conveyor_param_2, conveyor_param_3, conveyor_param_4, conveyor_param_5)
        self.label_serial_status.config(text=f"Last Serial Command: {message if success else 'Error'}")
    
    def classify_bbox(self, i, results, bbox):
        """
        Classify the bounding box using YOLO results, considering overlapping boxes and multiple berries.
        Uses centroid proximity to filter overlapping detections.
        """
        bx1, by1, bx2, by2 = bbox
        detections = []

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
                    if bx1 <= centroid_x <= bx2 and by1 <= centroid_y <= by2:
                        detections.append({
                            'class': class_names[int(cls)],
                            'score': score,
                            #'box': box,
                            'centroid': (centroid_x, centroid_y)
                        })

        # Filter overlapping detections by centroid proximity (e.g., within 10 pixels)
        filtered_detections = []
        used = set()
        for idx, det in enumerate(detections):
            if idx in used:
                continue
            cx1, cy1 = det['centroid']
            best_det = det
            for jdx, other in enumerate(detections):
                if jdx == idx or jdx in used:
                    continue
                cx2, cy2 = other['centroid']
                if abs(cx1 - cx2) <= 10 and abs(cy1 - cy2) <= 10:
                    # Keep the one with higher score
                    if other['score'] > best_det['score']:
                        best_det = other
                    used.add(jdx)
            filtered_detections.append(best_det)
            used.add(idx)

                # Remove detections under a score threshold
        SCORE_THRESHOLD = 0.5
        filtered_detections = [det for det in detections if det['score'] >= SCORE_THRESHOLD]

        self.set_bboxes_states(i, filtered_detections)
        print(f"Slot {i} detections: {detections}")
        print(f"Filtered detections: {filtered_detections}")

        berry_classes = [det['class'] for det in filtered_detections]
        if not berry_classes:
            return None

        # Multiple berry rules
        if len(berry_classes) > 1:
            if "OVERRIPE" in berry_classes:
                return "OVERRIPE"
            elif all(cls == "RIPE" for cls in berry_classes):
                return "RIPE"
            elif "UNDERRIPE" or "UNDERRIPE-GREEN" in berry_classes and "RIPE" in berry_classes:
                return "RIPE"
            elif "UNDERRIPE" or "UNDERRIPE-GREEN" in berry_classes:
                return "UNDERRIPE"
        else:
            if berry_classes[0] == "UNDERRIPE-GREEN":
                return "UNDERRIPE"
            else:
                return berry_classes[0]

if __name__ == "__main__":
    root = tk.Tk()
    app = WebcamApp(root)
    root.mainloop()