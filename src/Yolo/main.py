import tkinter as tk
from tkinter import ttk
from PIL import Image, ImageTk
import cv2
from ultralytics import YOLO
import time
from collections import deque
from cv2_enumerate_cameras import enumerate_cameras
from cv2_enumerate_cameras import supported_backends
from cv2.videoio_registry import getBackendName
import os

# Import our custom modules
from bbox_manager import BoundingBoxManager
from serial_manager import SerialManager, SerialControlWidget, PHIndicatorWidget
from carousel_widget import CarouselStatusWidget, Blueberry, CroppedImageWidget
from image_pipeline import ImagePipelineManager

class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO Webcam Segmentation")
        
        # Load YOLO model using a robust relative path
        model_path = os.path.join(os.path.dirname(__file__), "models", "best-train7-1_42_aug_9.pt")
        self.model = YOLO(model_path)
        # self.model = YOLO("best.pt")

        # Initialize classification tracking
        self.class_labels = ["RIPE", "UNDERRIPE", "OVERRIPE"]
        self.slot_classification_history = deque(maxlen=3)  # Track 3 positions by default
        self.current_slot_classifications = []
        
        # Image pipeline will be initialized after camera setup

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

        # --- Controls (stacked vertically, let widgets wrap/expand as needed) ---

        # Initialize webcam
        cv2.OPENCV_VIDEOIO_DEBUG=1
        self.camera_index = tk.IntVar(value=0)
        self.cap_backend = tk.IntVar(value=cv2.CAP_DSHOW)  # DirectShow backend for Windows
        self.cap = cv2.VideoCapture(self.camera_index.get(), self.cap_backend.get())

        # set frame size 1280x720
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)    

        # check camera aspect ratio
        self.camera_x = tk.IntVar(value=int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH)))
        self.camera_y = tk.IntVar(value=int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
        self.camera_aspect_ratio = self.camera_x.get() / self.camera_y.get()
        print(f"Camera resolution: {self.camera_x.get()}x{self.camera_y.get()}")
        print(f"Camera aspect ratio: {self.camera_aspect_ratio}")

        if not self.cap.isOpened():
            raise Exception("Webcam not accessible")

        # Define target resolution
        # self.camera_x.get() = 1920
        # self.camera_y.get() = 1080
        self.scale_feed = tk.DoubleVar(value=0.65)
        self.target_res = (round(self.camera_x.get() * self.scale_feed.get()), round(self.camera_y.get() * self.scale_feed.get()))
        self.scale_feed_model = tk.DoubleVar(value=0.65)
        self.target_res_model = (round(self.camera_x.get()*self.scale_feed_model.get()), round(self.camera_y.get()*self.scale_feed_model.get()))
        self.exposure_unsupported = False  # Now we try to support exposure
        # Create exposure as a tk variable with the current camera exposure value
        self.exposure = tk.DoubleVar(value=-4.8) # tk.DoubleVar(value=self.cap.get(cv2.CAP_PROP_EXPOSURE))
        
        print('exposure:', self.exposure.get())

        # Set manual exposure mode (disable auto exposure)
        # https://github.com/opencv/opencv/issues/9738

        self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 = manual, 0.75 = auto
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3) # auto mode
        # self.cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1) # manual mode

        # =====================================================================
        # ========================= CAMERA CONTROLS ===========================
        # =====================================================================
        

        camera_controls_frame = tk.Frame(controls_frame)
        camera_controls_frame.pack(fill=tk.X, pady=2)


        tk.Label(camera_controls_frame, text="Camera:").pack(side=tk.LEFT)

        # ========================= CAMERA SELECTION ==========================

        self.available_cameras = [1]  # Placeholder for available cameras

        self.camera_dropdown = ttk.Combobox(camera_controls_frame, textvariable=self.camera_index,width=1, state='readonly')
        self.camera_dropdown['values'] = self.available_cameras
        self.camera_dropdown.pack(side=tk.LEFT, padx=2)

        def populate_camera_dropdown(event=None):
            # Use cv2_enumerate_cameras to get available cameras
            self.available_cameras = []
            for camera_info in enumerate_cameras(self.cap_backend.get()):
                if camera_info.index not in self.available_cameras:
                    print(f"Found camera: {camera_info.name} at index {camera_info.index}")
                self.available_cameras.append(camera_info.index)
            self.camera_dropdown['values'] = self.available_cameras

        populate_camera_dropdown()

        # Update camera list every time the dropdown is clicked/opened
        self.camera_dropdown.bind("<Button-1>", populate_camera_dropdown)

        
        # ========================= BACKEND SELECTION =========================

        # tk.Label(camera_controls_frame, text="Backend:").pack(side=tk.LEFT)

        # combobox for camera backends
        self.available_backends = [cv2.CAP_DSHOW]
        self.backend_dropdown = ttk.Combobox(camera_controls_frame, textvariable=self.cap_backend, width=4, state='readonly')
        self.backend_dropdown['values'] =  self.available_backends #[getBackendName(backend) for backend in self.available_backends]
        self.backend_dropdown.pack(side=tk.LEFT, padx=2)

        # Function to update camera backend
        def update_camera_backend(event=None):
            self.available_backends = []
            for backend in supported_backends:
                self.available_backends.append(backend)
            self.backend_dropdown['values'] = self.available_backends # 

        update_camera_backend()

        # Update backend list every time the dropdown is clicked/opened
        self.backend_dropdown.bind("<Button-1>", update_camera_backend)

        # ========================= RECONNECT BUTTON ==========================

        # add a button with ascii play icon / emoji to trigger camera reconnection
        self.btn_reconnect_camera = ttk.Button(camera_controls_frame, text="▶", command=lambda: self.reconnect_camera(), width=2)
        self.btn_reconnect_camera.pack(side=tk.LEFT, padx=2)

        # ========================= EXPOSURE CONTROL ==========================
        
        # spinbox for exposure
        tk.Label(camera_controls_frame, text="Exposure:").pack(side=tk.LEFT)
        self.exposure_spinbox = tk.Spinbox(camera_controls_frame, from_=-10, to=100, increment=.05, textvariable=self.exposure,width=4, state='normal')
        self.exposure_spinbox.pack(side=tk.LEFT, padx=2)

        # function to update exposure
        def update_exposure(event=None):
            try:
                # Get the exposure value from the spinbox
                exp_value = float(self.exposure_spinbox.get())
                self.cap.set(cv2.CAP_PROP_EXPOSURE, exp_value)
                print(f"Exposure set to: {exp_value}")
            except ValueError:
                print("Invalid exposure value")

        # update exposure when spinbox value changes
        self.exposure_spinbox.bind("<Return>", update_exposure)
        self.exposure_spinbox.bind("<FocusOut>", update_exposure)
        self.exposure_spinbox.bind("<Button-1>", update_exposure)

        # # ========================== GAMMA CONTROL =========================

        # self.gamma = tk.DoubleVar(value=self.cap.get(cv2.CAP_PROP_GAMMA))
        # print('gamma:', self.gamma.get())

        # tk.Label(camera_controls_frame, text="Gamma:").pack(side=tk.LEFT)
        # self.gamma_spinbox = tk.Spinbox(camera_controls_frame, from_=0, to=100, increment=.01, textvariable=self.gamma, width=3, state='normal')
        # self.gamma_spinbox.pack(side=tk.LEFT, padx=2)

        # # function to update gamma
        # def update_gamma(event=None):
        #     try:
        #         # Get the gamma value from the spinbox
        #         gamma_value = float(self.gamma_spinbox.get())
        #         self.cap.set(cv2.CAP_PROP_GAMMA, gamma_value)
        #         print(f"Gamma set to: {gamma_value}")
        #     except ValueError:
        #         print("Invalid gamma value")

        # # update gamma when spinbox value changes
        # self.gamma_spinbox.bind("<Return>", update_gamma)
        # self.gamma_spinbox.bind("<FocusOut>", update_gamma)

        # ========================== GAIN CONTROL =========================

        self.gain = tk.DoubleVar(value=3.0) #  tk.DoubleVar(value=self.cap.get(cv2.CAP_PROP_GAIN))
        print('gain:', self.gain.get())

        tk.Label(camera_controls_frame, text="Gain:").pack(side=tk.LEFT)
        self.gain_spinbox = tk.Spinbox(camera_controls_frame, from_=1, to=9, increment=.5, textvariable=self.gain, width=3, state='normal')
        self.gain_spinbox.pack(side=tk.LEFT, padx=2)

        # function to update gain
        def update_gain(event=None):
            try:
                # Get the gain value from the spinbox
                gain_value = float(self.gain_spinbox.get())
                self.cap.set(cv2.CAP_PROP_GAIN, gain_value)
                print(f"Gain set to: {gain_value}")
            except ValueError:
                print("Invalid gain value")

        # update gain when spinbox value changes
        self.gain_spinbox.bind("<Return>", update_gain)
        self.gain_spinbox.bind("<FocusOut>", update_gain)
        self.gain_spinbox.bind("<Button-1>", update_gain)  # Bind to mouse click

        # ========================== SCALE FEED =========================

        tk.Label(camera_controls_frame, text="Scale:").pack(side=tk.LEFT)
        self.scale_feed_spinbox = tk.Spinbox(camera_controls_frame, from_=0.1, to=1.0, increment=0.05, textvariable=self.scale_feed, width=4)
        self.scale_feed_spinbox.pack(side=tk.LEFT, padx=2)

        # Function to update target resolution based on scale feed
        def update_target_resolution(event=None):
            try:
                scale_value = float(self.scale_feed_spinbox.get())
                self.target_res = (round(self.camera_x.get() * scale_value), round(self.camera_y.get() * scale_value))
                print(f"Target resolution set to: {self.target_res[0]}x{self.target_res[1]}")
                # Update canvas size to match new target resolution
                self.canvas.config(width=self.target_res[0], height=self.target_res[1])
            except ValueError:
                print("Invalid scale value")

        # Update target resolution when scale feed value changes
        self.scale_feed_spinbox.bind("<Return>", update_target_resolution)
        self.scale_feed_spinbox.bind("<Button-1>", update_target_resolution)

        # =====================================================================
        # ========================= SERIAL PORT CONTROLS ======================
        # =====================================================================

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # Serial controls
        self.serial_manager = SerialManager(root)
        self.serial_control_widget = SerialControlWidget(controls_frame, self.serial_manager)

        # PH indicator widget
        # self.ph_indicator = PHIndicatorWidget(serial_status_container, self.serial_manager)

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
        self.btn_clear = ttk.Button(btn_frame, text="Clear", command=self.clear)
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
                self.edit_toggle.state(['!selected'])

        def edit_toggle_command():
            edit_mode = self.bbox_manager.toggle_edit_mode()
            if edit_mode:
                self.draw_toggle.state(['!selected'])

        self.draw_toggle = ttk.Checkbutton(
            bbox_frame, text="Draw",
            command=draw_toggle_command,
            state=['!selected']  # Start with draw mode off

        )
        self.draw_toggle.pack(side=tk.LEFT, padx=2)

        self.edit_toggle = ttk.Checkbutton(
            bbox_frame, text="Edit",
            command=edit_toggle_command,
            state='normal'  # Start with edit mode off
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
        self.auto_capture_dir = tk.StringVar(value="C:\\Users\\jdavis\\Desktop\\blueberry\\BlueberryJam\\captures")
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

        # =============================================================================
        # ========================= MOTOR CONTROLS ====================================
        # =============================================================================

        # Motor Control
        motor_frame = tk.Frame(controls_frame)
        motor_frame.pack(fill=tk.X, pady=2)

        # SHAKES:
        motor_frame.pack(fill=tk.X, pady=2)
        tk.Label(motor_frame, text="shake count:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_1 = tk.IntVar(value=0)
        self.spinbox_c1 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_1, width=2)
        self.spinbox_c1.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="PPS:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_2 = tk.IntVar(value=4)
        self.spinbox_c2 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_2, width=2)
        self.spinbox_c2.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="N:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_3 = tk.IntVar(value=3)
        self.spinbox_c3 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_3, width=2)
        self.spinbox_c3.pack(side=tk.LEFT, padx=2)

        #DROP off controlls:
        tk.Label(motor_frame, text="Drop:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_4 = tk.IntVar(value=4)
        self.spinbox_c4 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_4, width=2)
        self.spinbox_c4.pack(side=tk.LEFT, padx=2)

        tk.Label(motor_frame, text="VIB:").pack(side=tk.LEFT, padx=2)
        self.conveyor_param_5 = tk.IntVar(value=1)
        self.spinbox_c5 = ttk.Spinbox( motor_frame, from_=0, to=9, textvariable=self.conveyor_param_5, width=2)
        self.spinbox_c5.pack(side=tk.LEFT, padx=2)

        # add a slim canvas element, 10px x 450px, that shows a square wave representing the motor conrol pattern
        # conveyor_param_1 = shake count - number of initial waves
        # conveyor_param_2 = PPS (pulses per second) - represented by the frequency of the square wave
        # conveyor_param_3 = N (motor pulses) - represented by the amplitude of the square wave
        # conveyor_param_4 = Drop (motor pulses for drop) - represented by a single trailing pulse, also amplitude
        # conveyor_param_5 = VIB (vibration strength) 
        
        motor_sequence = tk.Canvas(controls_frame, width=200, height=10, bg="white")
        motor_sequence.pack(fill=tk.X, pady=2)
        motor_sequence.create_rectangle(0, 0, 200, 50, fill="white", outline="black")
        motor_sequence.create_text(100, 25, text="Motor Sequence", font=("Arial", 10, "bold"), fill="black")
        # Draw the square wave based on the parameters
        def draw_motor_sequence():
            motor_sequence.delete("all")
            width = 200
            height = 10
            shake_count = self.conveyor_param_1.get()
            pps = self.conveyor_param_2.get() + 1
            n = self.conveyor_param_3.get() + 1
            drop = self.conveyor_param_4.get()
            vib = self.conveyor_param_5.get()

            # Calculate the width of each pulse based on PPS
            pulse_width = width / (pps * 10)
            if pulse_width < 1:
                pulse_width = 1
            # Calculate the height of the pulse based on N
            pulse_height = height / 2 * (n / 10)
            if pulse_height < 1:
                pulse_height = 1
            # Calculate the drop pulse width
            drop_width = width / 10
            if drop_width < 1:
                drop_width = 1
            # Calculate the vibration pulse width
            vib_width = width / 10
            if vib_width < 1:
                vib_width = 1
            # Draw the square wave
            for i in range(shake_count):
                x_start = i * (pulse_width * 2)
                # Draw the pulse
                motor_sequence.create_rectangle(x_start, height - pulse_height, x_start + pulse_width, height, fill="blue", outline="blue")
                # Draw the drop pulse
                if i == shake_count - 1:
                    motor_sequence.create_rectangle(x_start + pulse_width, height - drop, x_start + pulse_width + drop_width, height, fill="red", outline="red")
                    motor_sequence.create_rectangle(x_start + pulse_width + drop_width, height - vib, x_start + pulse_width + drop_width + vib_width, height, fill="green", outline="green")
            if shake_count == 0:
                motor_sequence.create_rectangle(pulse_width, height - drop, pulse_width + drop_width, height, fill="red", outline="red")
                motor_sequence.create_rectangle(pulse_width + drop_width, height - vib, pulse_width + drop_width + vib_width, height, fill="green", outline="green")

            # Draw the remaining space
            if shake_count * (pulse_width * 2) < width:
                motor_sequence.create_rectangle(shake_count * (pulse_width * 2), 0, width, height, fill="white", outline="white")
            # Update the canvas
            motor_sequence.update()
        draw_motor_sequence()
        # Bind the draw function to the spinboxes
        self.spinbox_c1.bind("<Button-1>", lambda e: draw_motor_sequence())
        self.spinbox_c2.bind("<Button-1>", lambda e: draw_motor_sequence())
        self.spinbox_c3.bind("<Button-1>", lambda e: draw_motor_sequence())
        self.spinbox_c4.bind("<Button-1>", lambda e: draw_motor_sequence())
        self.spinbox_c5.bind("<Button-1>", lambda e: draw_motor_sequence())


        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)
        
        # Motor Control
        shake_frame = tk.Frame(controls_frame)
        shake_frame.pack(fill=tk.X, pady=2)

        self.current_sort_count = 0;
        self.shake = True

        # number of sorts between shakes control
        tk.Label(shake_frame, text="Number of sorts between Bucket vibration:").pack(side=tk.LEFT, padx=2)
        self.shake_interval = tk.IntVar(value=65)
        self.spinbox_shake_interval = ttk.Spinbox(shake_frame, from_=1, to=100, textvariable=self.shake_interval, width=3)
        self.spinbox_shake_interval.pack(side=tk.LEFT, padx=2)

        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        #=============================================================================
        # =================== AUTOMATIC CLASSIFICATION CONTROLS ======================
        #=============================================================================

        btn_frame = tk.Frame(controls_frame)
        btn_frame.pack(fill=tk.X, pady=2)

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
                self.btn_canvas.create_text(60, 25, text="STOP SORT", font=("Arial", 12, "bold"), fill="white")
            else:
                # Stopped state - Green with "Start"
                self.btn_canvas.delete("all")
                self.btn_canvas.create_rectangle(0, 0, 120, 50, fill="#00ff00", outline="#00cc00", width=2)
                self.btn_canvas.create_text(60, 25, text="START SORT", font=("Arial", 12, "bold"), fill="black")

        def on_button_click(event):
            start_classify_timer()

        def schedule_classify():
            if self.classify_timer_running:
                self.classify_and_update_queue()
                if self.shake:
                    interval_ms = int( (self.conveyor_param_5.get() + 5 ) * 1000)  # Add 1 second for shake
                else:
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

        tk.Label(btn_frame, text="rate: (s)").pack(side=tk.LEFT, padx=2)
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

        #add a checkbox to enable image saving during automatic classification
        tk.Label(btn_frame, text="Save Images:").pack(side=tk.LEFT, padx=2)
        self.save_images_var = tk.BooleanVar(value=False)
        self.save_images_checkbox = ttk.Checkbutton(btn_frame, variable=self.save_images_var)
        self.save_images_checkbox.pack(side=tk.LEFT, padx=2)


        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)

        # ==========================================================================================================
        # ======================================= CLASSIFICATION THRESHOLD =========================================
        # ==========================================================================================================

        # Classification threshold controls
        threshold_frame = tk.Frame(controls_frame)
        threshold_frame.pack(fill=tk.X, pady=2)
        tk.Label(threshold_frame, text="Threshold Ripe:").pack(side=tk.LEFT, padx=2)
        self.classification_threshold_ripe = tk.DoubleVar(value=0.5)
        self.input_classification_threshold_ripe = ttk.Spinbox(
            threshold_frame,
            from_=0.0,
            to=1.0,
            increment=0.01,
            textvariable=self.classification_threshold_ripe,
            format="%.2f",
            width=5
        )
        self.input_classification_threshold_ripe.pack(side=tk.LEFT, padx=2)

        tk.Label(threshold_frame, text="Underripe:").pack(side=tk.LEFT, padx=2)
        self.classification_threshold_underripe = tk.DoubleVar(value=0.5)
        self.input_classification_threshold_underripe = ttk.Spinbox(
            threshold_frame,
            from_=0.0,
            to=1.0,
            increment=0.01,
            textvariable=self.classification_threshold_underripe,
            format="%.2f",
            width=5
        )
        self.input_classification_threshold_underripe.pack(side=tk.LEFT, padx=2)

        tk.Label(threshold_frame, text="Overripe:").pack(side=tk.LEFT, padx=2)
        self.classification_threshold_overripe = tk.DoubleVar(value=0.8)
        self.input_classification_threshold_overripe = ttk.Spinbox(
            threshold_frame,
            from_=0.0,
            to=1.0,
            increment=0.01,
            textvariable=self.classification_threshold_overripe,
            format="%.2f",
            width=5
        )
        self.input_classification_threshold_overripe.pack(side=tk.LEFT, padx=2)
        # Bind the spinboxes to update the thresholds
        
        ttk.Separator(controls_frame, orient='horizontal').pack(fill=tk.X, pady=4)


        # ===========================================================================================================
        # ======================================== CAROUSEL STATUS WIDGET ============================================
        # ==================================================================================================

        # --- Add Carousel Status Widget at the bottom of controls column ---
        self.carousel_widget = CarouselStatusWidget(controls_frame)

        # ==========================================================================================================
        # ======================================== FEEDBACK / STATE OUTPUT ========================================
        # ==========================================================================================================

        # Canvas (video feed) at the top of feedback column
        self.canvas = tk.Canvas(feedback_frame, width=self.target_res[0], height=self.target_res[1])
        self.canvas.grid(row=0, column=0, pady=(0, 10))

        # Initialize modules
        self.bbox_manager = BoundingBoxManager(self.canvas, self.target_res)
        self.bbox_states = [[] for _ in range(self.bbox_manager.get_bbox_count())]  # Initialize with empty state per bbox
        
        # Set up callbacks
        self.bbox_manager.set_bbox_count_changed_callback(self.on_bbox_count_changed)
        
        # Initialize the image pipeline manager
        self.image_pipeline = ImagePipelineManager(
            camera=self.cap,
            model=self.model,
            target_res=self.target_res,
            target_res_model=self.target_res_model,
            canvas=self.canvas
        )


        self.label_states = tk.Label(feedback_frame, text="Bounding box states: \n \n \n",wraplength=800, anchor="w", justify="left")
        self.label_states.grid(row=3, column=0, sticky="w")

        self.label_eject_array = tk.Label(feedback_frame, text="Eject array: []", anchor="w", justify="left")
        self.label_eject_array.grid(row=4, column=0, sticky="w", pady=(5,0))

        self.label_serial_status = tk.Label(feedback_frame, text="Last Serial Command: None", wraplength=600, anchor="w", justify="left")
        self.label_serial_status.grid(row=5, column=0, sticky="w", pady=(5,0))

        # add cropped image widget
        self.cropped_image_widget = CroppedImageWidget(feedback_frame, self.carousel_widget ) # , width=feedback_frame.winfo_width())
        self.cropped_image_widget.grid(row=6, column=0, sticky="ew", pady=(0,0))   
        
        # # Add Statistics Widget
        # self.statistics_widget = StatisticsWidget(feedback_frame, self.carousel_widget, width=feedback_frame.winfo_width())
        # self.statistics_widget.grid(row=6, column=0, sticky="ew", pady=(10,0))
        
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
            
        # Use the image pipeline for more consistent behavior
        self.image_pipeline.show_segmented = True
        success, _ = self.image_pipeline.execute_pipeline(
            bbox_manager=self.bbox_manager,
            save_image=True,
            save_dir=self.auto_capture_dir.get()
        )
        
        # Send test eject command during auto capture
        self.send_eject_command_with_ui_values([1,1,1], self.shake)
        
        # Schedule next capture
        interval = self.auto_capture_interval.get()
        self.root.after(max(1000, int(interval * 1000)), self.auto_capture)

    
    def update_feed(self):
        """Update the video feed on the canvas using the image pipeline."""
        if not self.show_live:  # Only update feed if not in live YOLO mode
            # Use the image pipeline to capture and display a frame
            self.image_pipeline.show_segmented = False  # Regular camera view (no segmentation)
            success = self.image_pipeline.capture_frame() and self.image_pipeline.process_frame()
            
            if success:
                self.image_pipeline.create_display_image()
                self.image_pipeline.update_display()
                self.bbox_manager.draw_bboxes()
            
            self.root.after(10, self.update_feed)
        #         self.canvas.itemconfig(self.image_id, image=self.photo)
        #     self.bbox_manager.draw_bboxes()
    
    def capture(self):
        
        """Capture a frame and classify using the image pipeline."""
        # Stop regular feed updates and enable YOLO display
        self.show_live = True
        self.show_segmented = True
        
        # Enable segmentation display and run the full pipeline
        self.image_pipeline.show_segmented = True
        success, classifications = self.image_pipeline.execute_pipeline(
            bbox_manager=self.bbox_manager,
            save_image=False
        )
        
        # Make sure bounding boxes are drawn
        if success:
            self.bbox_manager.draw_bboxes()
            print("YOLO capture completed and displayed - results will remain until cleared")

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

    def toggle_live(self):
        """Toggle live mode to continuously show YOLO results."""
        self.show_live = not self.show_live
        if self.show_live:
            # Start live YOLO mode
            self.show_segmented = True
            self.image_pipeline.show_segmented = True
            self.show_live_yolo_results()
        else:
            # Exit live mode and return to normal camera view
            self.show_segmented = False
            self.image_pipeline.show_segmented = False
            self.clear()

    def show_live_yolo_results(self):
        """Continuously show YOLO results on the canvas."""
        if not self.show_live:
            return
        
        # Force segmentation mode to be active
        self.show_segmented = True
        
        # Use the image pipeline to capture, process, analyze and display
        self.image_pipeline.show_segmented = True
        
        # Run the full pipeline with analysis
        success = self.image_pipeline.capture_frame()
        if success:
            self.image_pipeline.process_frame()
            self.image_pipeline.analyze_frame()
            self.image_pipeline.create_display_image()
            self.image_pipeline.update_display()
            self.bbox_manager.draw_bboxes()
            
        # Continue the loop if still in live mode
        self.root.after(10, self.show_live_yolo_results)

    def clear(self):
        """Clear the canvas and restart the regular camera feed."""
        # Reset UI state first to prevent further processing of old frames
        self.show_segmented = False
        self.show_live = False
        self.reset_bbox_states()
        
        # Temporarily stop all image processing
        self.root.update_idletasks()
        
        # Reset all image pipeline state
        self.image_pipeline.clear_all()
        
        # Clear the camera buffer by reading a few frames and discarding them
        for _ in range(3):
            self.cap.read()
            
        # Immediately capture and show a fresh frame
        ret, frame = self.cap.read()
        if ret:
            # Process and display this frame directly
            processed = cv2.resize(frame.copy(), self.image_pipeline.target_res)
            rgb_frame = cv2.cvtColor(processed, cv2.COLOR_BGR2RGB)
            photo = ImageTk.PhotoImage(image=Image.fromarray(rgb_frame))
            self.image_pipeline.photo = photo
            self.image_pipeline.analyzed_frame = rgb_frame
            self.image_pipeline.update_display()
        
        # Update UI components
        self.bbox_manager.draw_bboxes()
        self.cropped_image_widget.update_display()
        
        # Always restart the feed when explicitly clearing
        self.update_feed()
    
    def __del__(self):
        """Ensure resources are cleaned up."""
        if hasattr(self, 'serial_manager'):
            self.serial_manager.disconnect()
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()

    def reconnect_camera(self):
        """Attempt to reconnect the camera if it was lost."""
        self.target_res = (round(self.camera_x.get()*self.scale_feed.get()), round(self.camera_y.get()*self.scale_feed.get()))
        self.target_res_model = (round(self.camera_x.get()*self.scale_feed_model.get()), round(self.camera_y.get()*self.scale_feed_model.get()))

        if hasattr(self, 'cap') and self.cap is not None:
            self.cap.release()
        time.sleep(0.5)
        self.cap = cv2.VideoCapture(self.camera_index.get(), self.cap_backend.get())
        if not self.cap.isOpened():
            print("Failed to reconnect camera.")
            return False
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.camera_x.get())
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.camera_y.get())
        
        # Update the image pipeline with the new camera
        self.image_pipeline.camera = self.cap
        self.image_pipeline.target_res = self.target_res
        self.image_pipeline.target_res_model = self.target_res_model
        
        print("Camera reconnected successfully.")
        return True
    
    # The draw_yolo_results method has been replaced by the image pipeline's
    # execute_pipeline method which handles displaying annotated images

    def classify_and_update_queue(self):
        """Classify the current frame and update the carousel queue."""
        # Stop regular feed updates and keep showing YOLO results
        self.show_live = True
        self.show_segmented = True

        self.image_pipeline.set_score_thresholds(
            ripe=self.classification_threshold_ripe.get(),
            underripe=self.classification_threshold_underripe.get(),
            underripe_green=self.classification_threshold_underripe.get(),
            overripe=self.classification_threshold_overripe.get()
        )
        
        # Force UI update to ensure we're seeing the most current camera view before we process
        self.root.update_idletasks()
        
        # Completely reset image pipeline to ensure fresh capture
        self.image_pipeline.clear_all()  # This will also flush the camera buffer
        
        # Capture a few frames to ensure we're looking at current data
        for _ in range(3):
            self.cap.read()  # Read and discard a few frames
        
        # 1. Capture image, process it, and classify bounding boxes using the pipeline
        self.image_pipeline.show_segmented = True  # Enable YOLO result display
        success, classifications = self.image_pipeline.execute_pipeline(
            bbox_manager=self.bbox_manager,
            save_image=self.save_images_var.get(),
            save_dir=self.auto_capture_dir.get() if self.save_images_var.get() else None
        )
        
        if not success:
            # If capture failed, attempt to reconnect camera
            print("Camera read failed")
            if not self.reconnect_camera():
                print("Camera still not accessible after reconnect attempt.")
                return
            
            # Try again after reconnect
            success, classifications = self.image_pipeline.execute_pipeline(
                bbox_manager=self.bbox_manager,
                save_image=self.save_images_var.get(),
                save_dir=self.auto_capture_dir.get() if self.save_images_var.get() else None
            )
            
            if not success:
                return
        
        # Update carousel with classifications
        self.update_carousel_with_classifications(classifications)
        
        # 2. Check ejection ports and send serial commands
        eject_array = self.carousel_widget.get_eject_array()
        self.label_eject_array.config(text=f"Eject array: {eject_array}")

        # Increment sort count up to shake interval and wrap around
        self.current_sort_count += 1
        self.shake = False
        if self.current_sort_count >= self.shake_interval.get():
            self.current_sort_count = 0
            self.shake = True
        
        # 3. Send serial commands
        self.send_eject_command_with_ui_values(eject_array, self.shake)
        
        # 4. Update UI components and ensure the segmented image is displayed
        self.cropped_image_widget.update_display()
        
        # Make sure bounding boxes are visible
        self.bbox_manager.draw_bboxes()
        
        # Segmented display will stay visible until user clears it or runs another operation

    def clear_and_restart_feed(self):
        """Clear the display and restart the regular camera feed."""
        self.show_live = False
        self.show_segmented = False
        self.clear()
        
    def update_carousel_with_classifications(self, classifications):
        """
        Update the carousel with the classifications returned by the image pipeline.
        
        Args:
            classifications: List of (classification, cropped_image) tuples for each bbox
        """
        if not classifications:
            return
            
        # Reset bbox states before updating with new classifications
        self.reset_bbox_states()
        
        # Update the carousel based on classifications
        for i, (classification, cropped_image) in enumerate(classifications):
            # Update the bbox states in the UI
            if classification:
                self.bbox_states[i] = [{'class': classification, 'score': 1.0, 'centroid': (0, 0)}]
            
            # Update the carousel
            if i == 0:
                # For the first slot, create a new Blueberry object and add it to the carousel
                self.carousel_widget.add_to_carousel(classification, cropped_image)
            else:
                # For subsequent slots, update the history of the existing Blueberry object
                blueberry = self.carousel_widget.carousel_slots[i]
                if blueberry:
                    blueberry.add_classification_attempt(classification, cropped_image)
        
        # Update the bbox states display
        self.set_bboxes_states_from_classifications(classifications)

    def set_bboxes_states_from_classifications(self, classifications):
        """
        Update the bbox states display based on the classifications.
        
        Args:
            classifications: List of (classification, cropped_image) tuples
        """
        lines = []
        for i, (classification, _) in enumerate(classifications):
            if classification:
                line = f"BOX{i+1}: {classification}"
            else:
                line = f"BOX{i+1}: None"
            lines.append(line)
            
        state_text = "Bounding Box States:\n" + "\n".join(lines)
        self.label_states.config(text=state_text)
        
    def send_eject_command_with_ui_values(self, eject_array, shake):
        conveyor_param_1 = self.conveyor_param_1.get()
        conveyor_param_2 = self.conveyor_param_2.get()
        conveyor_param_3 = self.conveyor_param_3.get()
        conveyor_param_4 = self.conveyor_param_4.get()
        if shake:
            conveyor_param_1 = 9
            conveyor_param_5 = self.conveyor_param_5.get()
            success, message = self.serial_manager.send_eject_command([0,0,0], 1 , conveyor_param_1, conveyor_param_2, conveyor_param_3, conveyor_param_4, conveyor_param_5)
            self.label_serial_status.config(text=f"Last Serial Command: {message if success else 'Error'}")
            print(f"Shake activated - sent eject command with shake params: {conveyor_param_1}, {conveyor_param_2}, {conveyor_param_3}, {conveyor_param_4}, {conveyor_param_5}")
            #send aditional shake command to the arduino to shake conveyor belt again after 1 second
            # time.sleep(1)
            # success, message = self.serial_manager.send_eject_command([0,0,0], 1, conveyor_param_1, conveyor_param_2, conveyor_param_3, conveyor_param_4, 0)
            # time.sleep(1)
            # success, message = self.serial_manager.send_eject_command(eject_array, 1, conveyor_param_1, conveyor_param_2, conveyor_param_3, conveyor_param_4, 0)
        else:
            conveyor_param_1 = self.conveyor_param_1.get()
            conveyor_param_5 = 0
            success, message = self.serial_manager.send_eject_command(eject_array, 1, conveyor_param_1, conveyor_param_2, conveyor_param_3, conveyor_param_4, conveyor_param_5)
            self.label_serial_status.config(text=f"Last Serial Command: {message if success else 'Error'}")
    
    # The classify_bbox method has been moved to the ImagePipelineManager class

if __name__ == "__main__":
    root = tk.Tk()
    root.attributes('-fullscreen', True)
    app = WebcamApp(root)
    root.mainloop()