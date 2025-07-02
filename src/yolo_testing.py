import os
# Set environment vars before any scientific libs are imported
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
os.environ["KMP_DUPLICATE_LIB_OK"] = "TRUE"

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

try:
    from AVFoundation import AVCaptureDevice, AVCaptureSession, AVCaptureDeviceInput
    from Foundation import NSBundle
    AVFOUNDATION_AVAILABLE = True
    print("AVFoundation is available")
except ImportError:
    AVFOUNDATION_AVAILABLE = False
    print("AVFoundation is not available, exposure control will be disabled")
try:
    import uvc
except ImportError:
    uvc = None
    print("pyuvc not installed. UVC controls will be disabled.")

# Disable MKL threading to avoid OpenMP conflicts
os.environ["MKL_NUM_THREADS"] = "1"
os.environ["OMP_NUM_THREADS"] = "1"
try:
    import torch
    print(torch.__version__)
    print(torch.__config__.show())
    torch.set_num_threads(1)
except ImportError:
    print("PyTorch not installed, but it is required for ultralytics.")

class WebcamApp:
    def __init__(self, root):
        self.root = root
        self.root.title("YOLO Webcam Segmentation")
        
        # Load YOLO model
        self.model = YOLO("best.pt")
        self.model.to('cpu')
        
        # Initialize webcam
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            raise Exception("Webcam not accessible")
        
        # get camera type
        camera_type = self.cap.get(cv2.CAP_PROP_BACKEND)
        if camera_type == cv2.CAP_V4L2:
            print("Using V4L2 backend")
        elif camera_type == cv2.CAP_DSHOW:
            print("Using DirectShow backend")
        elif camera_type == cv2.CAP_GSTREAMER:
            print("Using GStreamer backend")
        else:
            print(f"Using unknown backend: {camera_type}")
        
        # Initialize UVC device
        self.uvc_device = None
        self.uvc_controls = {}
        if uvc:
            print("Initializing UVC device...")
            try:
                # List UVC devices
                print("Listing UVC devices...")
                devices = uvc.device_list()
                if devices:
                    print(f"Found {len(devices)} UVC devices")
                    self.uvc_device = uvc.Capture(devices[0]['uid'])
                    print(f"Using UVC device: {self.uvc_device}")
                    # Get available controls (e.g., exposure, brightness)
                    for ctrl in self.uvc_device.controls:
                        print(f"Control: {ctrl.display_name} (ID: {ctrl.id}, Type: {ctrl.type})")
                        self.uvc_controls[ctrl.display_name] = ctrl
            except Exception as e:
                print(f"UVC initialization failed: {e}")
        
        # Define target resolution
        self.camera_x = 1920
        self.camera_y = 1080
        self.scale_feed = 0.5
        self.target_res = (round(self.camera_x*self.scale_feed), round(self.camera_y*self.scale_feed))
        self.exposure_unsupported = True
        self.capture_device = None
        self.exposure = 1  # Default exposure value (normalized 0 to 1)

        # Define bounding boxes [[TopCornerX, TopCornerY, BottomCornerX, BottomCornerY], ...]
        self.bboxes = [
            [100, 100, 250, 250],  # Box 1
            [300, 100, 450, 250],  # Box 2
            [100, 300, 250, 450],  # Box 3
        ]
        
        # Initialize state for each bounding box
        self.bbox_states = [[] for _ in self.bboxes]
        
        # GUI elements
        self.canvas = tk.Canvas(root, width=self.target_res[0], height=self.target_res[1])
        self.canvas.pack()
        
        self.btn_capture = tk.Button(root, text="Capture & Segment", command=self.capture)
        self.btn_capture.pack(side=tk.LEFT, padx=5, pady=10)
        
        self.btn_clear = tk.Button(root, text="Clear", command=self.clear)
        self.btn_clear.pack(side=tk.LEFT, padx=5, pady=10)
        
        self.btn_draw = tk.Button(root, text="Draw Bounding Box", command=self.toggle_draw_mode)
        self.btn_draw.pack(side=tk.LEFT, padx=5, pady=10)
        
        self.btn_edit = tk.Button(root, text="Edit Bounding Box", command=self.toggle_edit_mode)
        self.btn_edit.pack(side=tk.LEFT, padx=5, pady=10)

        # Exposure slider
        self.exposure_label = tk.Label(root, text="Exposure: Supported" if not self.exposure_unsupported else "Exposure: Unsupported")
        self.exposure_label.pack(pady=5)
        self.exposure_scale = tk.Scale(root, from_=0.0, to=1.0, resolution=0.1, orient=tk.HORIZONTAL, 
                                      command=self.update_exposure, length=200)
        self.exposure_scale.set(self.exposure)
        self.exposure_scale.pack(pady=5)
        if self.exposure_unsupported:
            self.exposure_scale.config(state='disabled')
        
        self.label_states = tk.Label(root, text="Bounding Box States: []", wraplength=600)
        self.label_states.pack(pady=10)

        self.auto_capture_running = False
        self.auto_capture_interval = tk.IntVar(value=5)
        self.auto_capture_dir = tk.StringVar(value="/Users/jasper/Desktop/blueberry sorter/BlueberryJam/auto_captures")

        self.entry_interval = tk.Entry(root, textvariable=self.auto_capture_interval, width=5)
        self.entry_interval.pack(side=tk.LEFT, padx=5)
        self.label_interval = tk.Label(root, text="seconds")
        self.label_interval.pack(side=tk.LEFT)

        self.entry_dir = tk.Entry(root, textvariable=self.auto_capture_dir, width=40)
        self.entry_dir.pack(side=tk.LEFT, padx=5)
        self.btn_auto_capture = tk.Button(root, text="Start Auto Capture", command=self.toggle_auto_capture)
        self.btn_auto_capture.pack(side=tk.LEFT, padx=5)
        
        # State variables
        self.show_segmented = False
        self.segmented_image = None
        self.draw_mode = False
        self.edit_mode = False
        self.start_x = None
        self.start_y = None
        self.temp_bbox = None
        self.temp_rect_id = None
        self.bbox_items = []  # Store canvas rectangle and text IDs
        self.image_id = None  # Store canvas image ID
        self.selected_bbox_idx = None
        self.drag_mode = None  # 'move', 'top-left', 'top-right', 'bottom-left', 'bottom-right'
        self.proximity = 10  # Pixels for edge/corner detection

        # Carousel and ejection ports
        self.carousel_slots = deque([None]*20, maxlen=20)  # 20-slot queue
        self.eject_ports = {"RIPE": 5, "UNDERRIPE": 8, "OVERRIPE": 11}
        self.class_labels = ["RIPE", "UNDERRIPE", "OVERRIPE"]
        
        # Bind mouse events for drawing and editing
        self.canvas.bind("<Button-1>", self.start_action)
        self.canvas.bind("<B1-Motion>", self.update_action)
        self.canvas.bind("<ButtonRelease-1>", self.end_action)

         #initialize the serial connection
        ports = serial.tools.list_ports.comports()
        for port, desc, hwid in sorted(ports):
            print("{}: {} [{}]".format(port, desc, hwid))
        self.serial_port = None
        self.serial_port1 = None
        try:
            self.serial_port = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Adjust port as needed (e.g., '/dev/ttyUSB0')
            #self.serial_port = serial.Serial('/dev/cu.usbserial-21203', 9600, timeout=1)  # Adjust port as needed (e.g., '/dev/ttyUSB0')
        except serial.SerialException as e:
            print(f"Failed to open serial port: {e}")
        
        self.send_serial_data()
        
        # Display initial webcam feed
        self.update_feed()
    
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

    def update_uvc_control(self, control_name, value):
        if self.uvc_device and control_name in self.uvc_controls:
            try:
                self.uvc_controls[control_name].value = int(value)
            except Exception as e:
                print(f"Failed to set {control_name}: {e}")
    
    def check_exposure_support_AVF(self):
                # Initialize AVFoundation for exposure control
        if AVFOUNDATION_AVAILABLE:
            devices = AVCaptureDevice.devices()
            print(f"Found {len(devices)} AVFoundation devices")
            for device in devices:
                if device.hasMediaType_('vide'):
                    self.capture_device = device
                    print(f"Using AVFoundation device: {device.localizedName()}")
                    break
            if self.capture_device:
                try:
                    self.set_exposure_avf(self.exposure)
                    # Check if exposure adjustment is supported
                    if self.capture_device.isExposureModeSupported_(0):  # AVCaptureExposureModeContinuousAutoExposure
                        print("Exposure adjustment lock supported")
                        self.exposure_unsupported = False
                    elif self.capture_device.isExposureModeSupported_(1):
                        print("autoExpose supported")
                        self.exposure_unsupported = False
                    elif self.capture_device.isExposureModeSupported_(2):
                        print("ContinuousAutoExpose supported")
                        self.exposure_unsupported = False
                    elif self.capture_device.isExposureModeSupported_(3):
                        print("custom exposure supported")
                        self.exposure_unsupported = False
                    else:
                        print("Exposure adjustment not supported")
                        #self.exposure_unsupported = True
                except:
                    pass

    def set_exposure_avf(self, value):
        if self.capture_device and not self.exposure_unsupported:
            try:
                # Lock device for configuration
                self.capture_device.lockForConfiguration_(None)
                # Map 0-1 range to camera's exposure range
                min_exposure = self.capture_device.minExposureTargetBias
                max_exposure = self.capture_device.maxExposureTargetBias
                exposure_value = min_exposure + (max_exposure - min_exposure) * value
                self.capture_device.setExposureTargetBias_(exposure_value)
                self.capture_device.unlockForConfiguration()
            except:
                self.exposure_unsupported = True
                self.exposure_label.config(text="Exposure: Unsupported")
                self.exposure_scale.config(state='disabled')
    
    def update_exposure(self, value):
        self.exposure = float(value)
        self.set_exposure(self.exposure)

    def send_serial_data(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                # convert bbox_states to json and send
                data = json.dumps(self.bbox_states)+'\n'
                self.serial_port.write(data.encode('utf-8'))
            except serial.SerialException as e:
                print(f"serial write error: {e}")
        # Schedule next transmission
        self.root.after(3000, self.send_serial_data)
    
    def toggle_draw_mode(self):
        if self.edit_mode:
            self.toggle_edit_mode()
        self.draw_mode = not self.draw_mode
        self.btn_draw.config(relief="sunken" if self.draw_mode else "raised")
    
    def toggle_edit_mode(self):
        if self.draw_mode:
            self.toggle_draw_mode()
        self.edit_mode = not self.edit_mode
        self.btn_edit.config(relief="sunken" if self.edit_mode else "raised")
    
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

    
    
    def draw_bboxes(self):
        # Clear existing bounding box items
        for rect_id, text_id in self.bbox_items:
            self.canvas.delete(rect_id)
            self.canvas.delete(text_id)
        self.bbox_items = []
        
        # Draw new bounding boxes as canvas items
        for i, bbox in enumerate(self.bboxes):
            rect_id = self.canvas.create_rectangle(bbox, outline="green", width=2, tags=f"bbox_{i}")
            text_id = self.canvas.create_text(bbox[2] - 10, bbox[3] - 10, text=str(i), fill="white", anchor="se", tags=f"bbox_{i}")
            self.bbox_items.append((rect_id, text_id))
            
            # Bind hover and click events
            self.canvas.tag_bind(f"bbox_{i}", "<Enter>", lambda e, idx=i: self.on_bbox_hover(idx, True))
            self.canvas.tag_bind(f"bbox_{i}", "<Leave>", lambda e, idx=i: self.on_bbox_hover(idx, False))
            self.canvas.tag_bind(f"bbox_{i}", "<Button-1>", lambda e, idx=i: self.on_bbox_click(idx))
    
    def on_bbox_hover(self, index, enter):
        rect_id, _ = self.bbox_items[index]
        color = "yellow" if enter else "green"
        self.canvas.itemconfig(rect_id, outline=color)
    
    def on_bbox_click(self, index):
        print(f"Clicked bounding box {index}")
    
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
        self.send_next_serial()

        print(f"Classified: {slot_classifications}, Added: {best_class}, Queue: {list(self.carousel_slots)}")
        print(f"Eject: {eject_array}")

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
                data = json.dumps(eject_array) + '\n'
                self.serial_port.write(data.encode('utf-8'))
            except serial.SerialException as e:
                print(f"serial write error: {e}")

    def send_next_serial(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.write(b"NEXT\n")
            except serial.SerialException as e:
                print(f"serial write error: {e}")

if __name__ == "__main__":
    root = tk.Tk()
    app = WebcamApp(root)
    root.mainloop()