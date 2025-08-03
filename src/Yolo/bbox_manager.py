import tkinter as tk
from tkinter import simpledialog
from collections import deque


class BoundingBoxManager:
    """Manages bounding box drawing, editing, and interactions on a canvas."""
    
    def __init__(self, canvas, target_res):
        self.canvas = canvas
        self.target_res = target_res
        
        # Define bounding boxes [[TopCornerX, TopCornerY, BottomCornerX, BottomCornerY], ...]
        self.bboxes = [
            [562, 13, 817, 455], #Box 0
            [292, 13, 552, 454], #Box 1
            [11, 13, 283, 455], #Box 2
        ]
        
        # Callback for when bbox count changes
        self.on_bbox_count_changed = None
        
        # State variables for drawing/editing
        self.show_segmented = False
        self.segmented_image = None
        self.start_x = None
        self.start_y = None
        self.temp_bbox = None
        self.temp_rect_id = None
        self.bbox_items = []  # Store canvas rectangle and text IDs
        self.selected_bbox_idx = None
        self.drag_mode = None  # 'move', 'top-left', 'top-right', 'bottom-left', 'bottom-right'
        self.proximity = 10  # Pixels for edge/corner detection
        
        # Mode flags
        self.draw_mode = False
        self.edit_mode = False
        
        # Bind mouse events for drawing and editing
        self.canvas.bind("<Button-1>", self.start_action)
        self.canvas.bind("<B1-Motion>", self.update_action)
        self.canvas.bind("<ButtonRelease-1>", self.end_action)
        
    def set_modes(self, draw_mode, edit_mode):
        """Set the current drawing and editing modes."""
        self.draw_mode = draw_mode
        self.edit_mode = edit_mode
        
    def start_action(self, event):
        """Handle mouse button press events."""
        if self.draw_mode:
            self.start_x, self.start_y = event.x, event.y
            self.temp_bbox = [self.start_x, self.start_y, self.start_x, self.start_y]
            self.temp_rect_id = self.canvas.create_rectangle(self.temp_bbox, outline="red", width=2)
        elif self.edit_mode:
            self.selected_bbox_idx, self.drag_mode = self.find_bbox_edge_or_corner(event.x, event.y)
            if self.selected_bbox_idx is not None:
                self.start_x, self.start_y = event.x, event.y
    
    def find_bbox_edge_or_corner(self, x, y):
        """Find which bounding box edge or corner is being clicked."""
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
        """Handle mouse drag events."""
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
        """Handle mouse button release events."""
        if self.draw_mode and self.start_x is not None:
            end_x, end_y = event.x, event.y
            x1, x2 = min(self.start_x, end_x), max(self.start_x, end_x)
            y1, y2 = min(self.start_y, end_y), max(self.start_y, end_y)
            x1, y1 = max(0, x1), max(0, y1)
            x2, y2 = min(self.target_res[0], x2), min(self.target_res[1], y2)
            new_bbox = [x1, y1, x2, y2]
            index = simpledialog.askinteger("Input", "Enter index for this bounding box (0 to append, 1 to insert at start, etc.):", 
                                          parent=self.canvas.master, minvalue=0)
            if index is not None:
                if index == 0 or index >= len(self.bboxes):
                    self.bboxes.append(new_bbox)
                else:
                    self.bboxes.insert(index, new_bbox)
                self.draw_bboxes()
                # Notify callback that bbox count changed
                self._notify_bbox_count_changed()
            self.canvas.delete(self.temp_rect_id)
            self.start_x = None
            self.start_y = None
            self.temp_bbox = None
            self.temp_rect_id = None
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
        """Draw all bounding boxes on the canvas."""
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
        """Handle bounding box hover events."""
        # Only highlight yellow if not already selected
        if index < len(self.bbox_items):
            rect_id, _, _ = self.bbox_items[index]
            if index == self.selected_bbox_idx:
                color = "yellow"
            else:
                color = "yellow" if enter else "green"
            self.canvas.itemconfig(rect_id, outline=color)
            
            if self.draw_mode or self.edit_mode:
                self.selected_bbox_idx = index

    def on_bbox_click(self, index):
        """Handle bounding box click events."""
        if self.draw_mode or self.edit_mode:
            self.selected_bbox_idx = index
            # Redraw to update highlight
            self.draw_bboxes()

    def delete_selected_bbox(self):
        """Delete the currently selected bounding box."""
        if (self.draw_mode or self.edit_mode) and self.selected_bbox_idx is not None:
            idx = self.selected_bbox_idx
            if 0 <= idx < len(self.bboxes):
                del self.bboxes[idx]
                self.selected_bbox_idx = None
                self.draw_bboxes()
                # Notify callback that bbox count changed
                self._notify_bbox_count_changed()
                return True
        return False
    
    def get_bbox_count(self):
        """Get the number of bounding boxes."""
        return len(self.bboxes)
    
    def get_bboxes(self):
        """Get a copy of the current bounding boxes."""
        return self.bboxes.copy()
    
    def toggle_draw_mode(self):
        """Toggle draw mode on/off."""
        self.draw_mode = not self.draw_mode
        if self.draw_mode:
            self.edit_mode = False
        return self.draw_mode
    
    def toggle_edit_mode(self):
        """Toggle edit mode on/off."""
        self.edit_mode = not self.edit_mode
        if self.edit_mode:
            self.draw_mode = False
        return self.edit_mode
    
    def set_bbox_count_changed_callback(self, callback):
        """Set callback function to be called when bbox count changes."""
        self.on_bbox_count_changed = callback
    
    def _notify_bbox_count_changed(self):
        """Notify callback that bbox count has changed."""
        if self.on_bbox_count_changed:
            self.on_bbox_count_changed(len(self.bboxes))
