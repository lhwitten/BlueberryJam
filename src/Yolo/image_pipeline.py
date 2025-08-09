"""
Image Pipeline Manager for BlueberryJam project.

This module provides a structured approach to handling image capture,
processing, analysis, and display in a consistent manner.
"""

import cv2
import time
import os
import numpy as np
from PIL import Image, ImageTk
import tkinter as tk

class ImagePipelineManager:
    """
    Central manager for all image-related operations, providing a consistent
    pipeline for image capture, processing, analysis, and display.
    """
    
    def __init__(self, camera, model, target_res, target_res_model, canvas):
        """Initialize the image pipeline manager.
        
        Args:
            camera: OpenCV video capture object
            model: YOLO model for classification
            target_res: Resolution for display
            target_res_model: Resolution for model processing
            canvas: Tkinter canvas for display
        """
        self.camera = camera
        self.model = model
        self.target_res = target_res
        self.target_res_model = target_res_model
        self.canvas = canvas
        
        # Pipeline state storage
        self.raw_frame = None
        self.processed_frame = None
        self.analyzed_frame = None
        self.display_image = None
        self.yolo_results = None
        
        # Display variables
        self.image_id = None
        self.photo = None
        self.show_segmented = False
        self.show_live = False
        
        # Observer pattern for components that need image updates
        self.display_observers = []
        self.classification_observers = []

        self.SCORE_THRESHOLDS = {
            "RIPE": 0.65,
            "UNDERRIPE": 0.60,
            "UNDERRIPE-GREEN": 0.60,
            "OVERRIPE": 0.70
        }

    def add_display_observer(self, observer):
        """Add an observer that needs display updates."""
        if observer not in self.display_observers:
            self.display_observers.append(observer)
            
    def add_classification_observer(self, observer):
        """Add an observer that needs classification results."""
        if observer not in self.classification_observers:
            self.classification_observers.append(observer)
    
    def capture_frame(self):
        """Capture a new frame from the camera."""
        # Fast grab-then-retrieve approach to get the most current frame
        # This is more efficient than read() and helps avoid lag
        
        # Grab several frames but only process the last one
        for _ in range(5):  # Grab multiple frames to clear any buffer
            self.camera.grab()
        
        # Now retrieve only the last grabbed frame
        ret, frame = self.camera.retrieve()
        if not ret:
            # Fall back to standard read method if retrieve fails
            ret, frame = self.camera.read()
            if not ret:
                print("Failed to capture frame")
                return False
        
        self.raw_frame = frame
        return True
        
    def flush_camera_buffer(self):
        """Flush the camera buffer to get the most recent frame."""
        # Read and discard frames to clear the buffer completely
        # Increase the number of frames to flush to ensure we get the current frame
        for _ in range(15):  # Increased from 5 to 15 to handle larger buffer sizes
            self.camera.grab()  # Faster than read() as it doesn't decode the image
    
    def process_frame(self):
        """Process the raw frame for display and analysis."""
        if self.raw_frame is None:
            return False
            
        # Create a processed version for display
        self.processed_frame = cv2.resize(self.raw_frame.copy(), self.target_res)
        
        # Create a version for model processing
        self.model_frame = cv2.resize(self.raw_frame.copy(), self.target_res_model)
        
        return True
        
    def analyze_frame(self):
        """Run the YOLO model on the processed frame."""
        if self.model_frame is None:
            return False
            
        self.yolo_results = self.model(self.model_frame)
        return True
        
    def create_display_image(self):
        """Create the image for display with all annotations."""
        if self.show_segmented and self.yolo_results is not None:
            # Use the YOLO annotated image
            try:
                annotated_frame = self.yolo_results[0].plot()
                annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
                annotated_frame = cv2.resize(annotated_frame, self.target_res)
                self.analyzed_frame = annotated_frame
                print("Created YOLO segmented display image")
            except Exception as e:
                print(f"Error creating YOLO display: {e}")
                # Fall back to regular image if YOLO display fails
                if self.processed_frame is not None:
                    self.analyzed_frame = cv2.cvtColor(self.processed_frame, cv2.COLOR_BGR2RGB)
                else:
                    return False
        elif self.processed_frame is not None:
            # Use the processed frame without annotations
            self.analyzed_frame = cv2.cvtColor(self.processed_frame, cv2.COLOR_BGR2RGB)
        else:
            return False
            
        self.photo = ImageTk.PhotoImage(image=Image.fromarray(self.analyzed_frame))
        return True
    
    def update_display(self):
        """Update the display on the canvas."""
        if self.photo is None:
            return False
            
        if self.image_id is None:
            self.image_id = self.canvas.create_image(0, 0, image=self.photo, anchor=tk.NW)
        else:
            self.canvas.itemconfig(self.image_id, image=self.photo)
            
        # Notify display observers
        for observer in self.display_observers:
            if hasattr(observer, 'on_display_updated'):
                observer.on_display_updated(self.analyzed_frame)
                
        return True
    
    def classify_bboxes(self, bbox_manager):
        """Classify all bounding boxes based on the current YOLO results."""
        if self.yolo_results is None or self.model_frame is None:
            return None
            
        bboxes = bbox_manager.get_bboxes()
        classifications = []
        
        for i, bbox in enumerate(bboxes):
            classification, cropped_image = self.classify_bbox(i, bbox)
            classifications.append((classification, cropped_image))
            
        # Notify classification observers
        for observer in self.classification_observers:
            if hasattr(observer, 'on_classifications_updated'):
                observer.on_classifications_updated(classifications)
                
        return classifications
    
    def set_score_thresholds(self, ripe=0.65, underripe=0.60, underripe_green=0.60, overripe=0.70):
        """Set independent score thresholds for each class."""
        self.SCORE_THRESHOLDS = {
            "RIPE": ripe,
            "UNDERRIPE": underripe,
            "UNDERRIPE-GREEN": underripe_green,
            "OVERRIPE": overripe
        }
        print(f"Score thresholds updated: {self.SCORE_THRESHOLDS}")
    
    def classify_bbox(self, bbox_index, bbox):
        """
        Classify a single bounding box using YOLO results.

        Args:
            bbox_index: Index of the bounding box
            bbox: (x1, y1, x2, y2) tuple defining the bounding box

        Returns:
            Tuple of (classification, cropped_image)
        """
        # Independent score thresholds for each class
        SCORE_THRESHOLDS = {
            "RIPE": 0.65,
            "UNDERRIPE": 0.60,
            "UNDERRIPE-GREEN": 0.60,
            "OVERRIPE": 0.70
        }

        # Scale bounding box coordinates to match model resolution
        scale_x = self.target_res_model[0] / self.target_res[0]
        scale_y = self.target_res_model[1] / self.target_res[1]

        bx1, by1, bx2, by2 = bbox
        model_bx1 = int(bx1 * scale_x)
        model_by1 = int(by1 * scale_y)
        model_bx2 = int(bx2 * scale_x)
        model_by2 = int(by2 * scale_y)

        print(f"BBox {bbox_index}: Display coords {bbox}, Model coords [{model_bx1}, {model_by1}, {model_bx2}, {model_by2}]")

        detections = []

        for result in self.yolo_results:
            if result.boxes and result.boxes.xyxy is not None:
                boxes = result.boxes.xyxy.cpu().numpy()
                classes = result.boxes.cls.cpu().numpy()
                scores = result.boxes.conf.cpu().numpy()
                class_names = result.names

                for box, cls, score in zip(boxes, classes, scores):
                    x1, y1, x2, y2 = box[:4]
                    centroid_x = (x1 + x2) / 2
                    centroid_y = (y1 + y2) / 2

                    centroid_in_bbox = (model_bx1 <= centroid_x <= model_bx2 and model_by1 <= centroid_y <= model_by2)

                    intersection_x1 = max(model_bx1, x1)
                    intersection_y1 = max(model_by1, y1)
                    intersection_x2 = min(model_bx2, x2)
                    intersection_y2 = min(model_by2, y2)

                    if intersection_x2 > intersection_x1 and intersection_y2 > intersection_y1:
                        intersection_area = (intersection_x2 - intersection_x1) * (intersection_y2 - intersection_y1)
                        detection_area = (x2 - x1) * (y2 - y1)
                        overlap_ratio = intersection_area / detection_area

                        class_name = class_names[int(cls)]
                        threshold = self.SCORE_THRESHOLDS.get(class_name, 0.65)

                        if (centroid_in_bbox or overlap_ratio > 0.4) and score >= threshold:
                            print(f"BBox {bbox_index}: Detection with overlap ratio {overlap_ratio:.2f}, centroid_in_bbox={centroid_in_bbox}, class={class_name}, score={score:.2f}, threshold={threshold:.2f}")
                            detections.append({
                                'class': class_name,
                                'score': score,
                                'box': box,
                                'centroid': (centroid_x, centroid_y)
                            })

        cropped_image = self.model_frame[model_by1:model_by2, model_bx1:model_bx2]

        berry_classes = [det['class'] for det in detections]

        if not berry_classes:
            print(f"BBox {bbox_index}: No detections found in bounding box.")
            return None, cropped_image

        if len(detections) > 1:
            def calculate_iou(box1, box2):
                x1_1, y1_1, x2_1, y2_1 = box1
                x1_2, y1_2, x2_2, y2_2 = box2
                x_left = max(x1_1, x1_2)
                y_top = max(y1_1, y1_2)
                x_right = min(x2_1, x2_2)
                y_bottom = min(y2_1, y2_2)
                if x_right < x_left or y_bottom < y_top:
                    return 0.0
                intersection_area = (x_right - x_left) * (y_bottom - y_top)
                box1_area = (x2_1 - x1_1) * (y2_1 - y1_1)
                box2_area = (x2_2 - x1_2) * (y2_2 - y1_2)
                union_area = box1_area + box2_area - intersection_area
                return intersection_area / union_area if union_area > 0 else 0.0

            groups = []
            for det in detections:
                found_group = False
                for group in groups:
                    for group_det in group:
                        iou = calculate_iou(det['box'], group_det['box'])
                        if iou > 0.5:
                            group.append(det)
                            found_group = True
                            break
                    if found_group:
                        break
                if not found_group:
                    groups.append([det])

            filtered_detections = []
            for group in groups:
                best_detection = max(group, key=lambda det: det['score'])
                filtered_detections.append(best_detection)

            print(f"BBox {bbox_index}: Found {len(detections)} raw detections, filtered to {len(filtered_detections)} unique berries")
            detections = filtered_detections

        berry_classes = [det['class'] for det in detections]

        if len(berry_classes) > 1:
            x1 = min(det['box'][0] for det in detections)
            y1 = min(det['box'][1] for det in detections)
            x2 = max(det['box'][2] for det in detections)
            y2 = max(det['box'][3] for det in detections)
            box_crop = self.model_frame[int(y1):int(y2), int(x1):int(x2)]

            if "OVERRIPE" in berry_classes:
                print(f"BBox {bbox_index}: Multiple berries detected, at least one OVERRIPE. Classified as OVERRIPE.")
                return "OVERRIPE", box_crop
            elif all(cls == "RIPE" for cls in berry_classes):
                print(f"BBox {bbox_index}: Multiple berries detected, all RIPE. Classified as RIPE.")
                return "RIPE", box_crop
            elif ("UNDERRIPE" in berry_classes or "UNDERRIPE-GREEN" in berry_classes) and "RIPE" in berry_classes:
                print(f"BBox {bbox_index}: Both RIPE and UNDERRIPE(-GREEN) detected. Classified as RIPE.")
                return "RIPE", box_crop
            elif "UNDERRIPE" in berry_classes or "UNDERRIPE-GREEN" in berry_classes:
                print(f"BBox {bbox_index}: Multiple berries detected, at least one UNDERRIPE(-GREEN). Classified as UNDERRIPE.")
                return "UNDERRIPE", box_crop
            else:
                print(f"BBox {bbox_index}: Multiple berries detected, but no clear classification.")
                return None, box_crop
        elif len(berry_classes) == 1:
            det = detections[0]
            bbox = det['box']
            score = det['score']
            det_crop = self.model_frame[int(bbox[1]):int(bbox[3]), int(bbox[0]):int(bbox[2])]

            if berry_classes[0] == "UNDERRIPE-GREEN":
                print(f"BBox {bbox_index}: Single berry detected, class UNDERRIPE-GREEN (score: {score:.2f}). Classified as UNDERRIPE.")
                return "UNDERRIPE", det_crop
            else:
                print(f"BBox {bbox_index}: Single berry detected, class {berry_classes[0]} (score: {score:.2f}). Classified as {berry_classes[0]}.")
                return berry_classes[0], det_crop
        else:
            print(f"BBox {bbox_index}: No valid classification found.")
            return None, cropped_image
    
    def save_current_frame(self, directory):
        """Save the current frame to disk."""
        if self.model_frame is None:
            return False
            
        os.makedirs(directory, exist_ok=True)
        timestamp = time.strftime("%Y%m%d_%H%M%S")
        filename = f"{directory}/capture_{timestamp}.jpg"
        cv2.imwrite(filename, self.model_frame)
        print(f"Saved: {filename}")
        return True
        
    def reset_display(self):
        """Reset the display state."""
        self.show_segmented = False
        self.yolo_results = None
        self.analyzed_frame = None
        self.photo = None
        
    def clear_all(self):
        """Clear all image data and reset state."""
        self.reset_display()
        self.raw_frame = None
        self.processed_frame = None
        self.model_frame = None
        
        # Flush the camera buffer thoroughly to ensure we get fresh frames
        # First, stop capturing for a short moment to let the camera hardware reset
        time.sleep(0.1)
        
        # Then completely flush the buffer
        self.flush_camera_buffer()
        
        # Additional flush after a brief pause to ensure buffer is truly empty
        time.sleep(0.1)
        self.flush_camera_buffer()
        
    def execute_pipeline(self, bbox_manager=None, save_image=False, save_dir=None):
        """
        Execute the full image pipeline: capture, process, analyze, classify, display.
        
        Returns:
            Tuple of (success, classifications)
        """
        # Make sure any previous state is cleared
        self.raw_frame = None
        self.processed_frame = None
        self.model_frame = None
        
        # Direct capture approach to get the most current frame possible
        # Skip buffered frames by grabbing several frames quickly and only decoding the last one
        for _ in range(3):
            self.camera.grab()
            
        # Now get the most current frame
        ret, frame = self.camera.retrieve()
        if ret:
            self.raw_frame = frame
        else:
            # Fall back to regular capture method
            if not self.capture_frame():
                print("Pipeline: Frame capture failed")
                return False, None
            
        # Process the frame
        if not self.process_frame():
            print("Pipeline: Frame processing failed")
            return False, None
            
        # Save image if requested
        if save_image and save_dir:
            self.save_current_frame(save_dir)
            
        # Analyze the frame with YOLO
        if not self.analyze_frame():
            print("Pipeline: YOLO analysis failed")
            return False, None
            
        # Classify bounding boxes if provided
        classifications = None
        if bbox_manager:
            classifications = self.classify_bboxes(bbox_manager)
            
        # Create the display image
        if not self.create_display_image():
            print("Pipeline: Display image creation failed")
            return False, classifications
            
        # Update the display
        if not self.update_display():
            print("Pipeline: Display update failed")
            return False, classifications
            
        # Print success message for debugging
        if self.show_segmented:
            print("Pipeline complete: YOLO segmentation displayed")
        else:
            print("Pipeline complete: Regular image displayed")
            
        return True, classifications
