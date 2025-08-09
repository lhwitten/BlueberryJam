import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
from collections import deque, Counter
import time

class Blueberry:
    """Represents a blueberry with classification history."""
    def __init__(self, initial_classification, cropped_image):
        self.history = [initial_classification]  # Stores classification attempts
        self.images = [cropped_image]  # Stores cropped images

    def add_classification_attempt(self, classification, image):
        """Add a classification attempt to the history."""
        self.history.append(classification)
        self.images.append(image)

    def determine_final_class(self):
        """Determine the final classification based on history."""
        counts = Counter(self.history)
        return counts.most_common(1)[0][0] if counts else None  

class CarouselStatusWidget:
    """Widget for displaying carousel slot status in a circular layout."""

    QUEUE_LENGTH = 20  # Define queue length as a class-level variable
    
    # Color scheme constants
    COLORS = {
        "RIPE": {"fill": "#b3e0ff", "outline": "#3399ff"},
        "UNDERRIPE": {"fill": "#baffc9", "outline": "#33cc66"},
        "OVERRIPE": {"fill": "#ffb3b3", "outline": "#ff6666"},
        None: {"fill": "#eeeeee", "outline": "#cccccc"},
        "SELECTED": {"fill": "#fffbe6", "outline": "#ffcc00"}  # Highlight color
    }

    def __init__(self, parent_frame, width=400, height=280):
        self.parent_frame = parent_frame
        self.width = width
        self.height = height
        
        # Statistics tracking
        self.stats = {
            "RIPE": 0,
            "UNDERRIPE": 0,
            "OVERRIPE": 0
        }
        self.start_time = time.time()
        self.total_berries_sorted = 0
        
        # Carousel data
        self.carousel_slots = deque([None] * self.QUEUE_LENGTH, maxlen=self.QUEUE_LENGTH)  # Use QUEUE_LENGTH
        self.eject_ports = {"RIPE": 3, "UNDERRIPE": 5, "OVERRIPE": 7}
        self.class_labels = ["RIPE", "UNDERRIPE", "OVERRIPE"]

        self.selected_slot = None  # Track the currently selected slot

        # Create frame for label and reset button
        self.header_frame = tk.Frame(parent_frame)
        self.header_frame.pack(side=tk.TOP, fill=tk.X, pady=(2, 2))
        
        # Create label
        self.label = tk.Label(self.header_frame, text="Carousel Slots:")
        self.label.pack(side=tk.LEFT, pady=(2, 2))
        
        # Add reset statistics button
        self.reset_button = tk.Button(self.header_frame, text="Reset Statistics", command=self.reset_statistics)
        self.reset_button.pack(side=tk.RIGHT, padx=(0, 5))
        
        # Create canvas
        self.canvas = tk.Canvas(parent_frame, width=width, height=height, highlightthickness=0)
        self.canvas.pack(side=tk.TOP, pady=(0, 0))
        
        # Tooltip variables
        self.tooltip = None
        self.tooltip_slot = None
        
        # Bind mouse events for tooltip and slot selection
        self.canvas.bind("<Motion>", self.on_mouse_motion)
        self.canvas.bind("<Leave>", self.hide_tooltip)
        self.canvas.bind("<Button-1>", self.on_slot_click)
        
        # Initial draw
        self.draw_carousel_status()
        
        # Set up auto-refresh for statistics
        self.auto_refresh()

        self.selection_callbacks = []

    def register_selection_callback(self, callback):
        self.selection_callbacks.append(callback)
    
    def add_to_carousel(self, initial_classification, cropped_image):
        """Add a new Blueberry object to the carousel queue."""
        blueberry = Blueberry(initial_classification, cropped_image)
        self.carousel_slots.appendleft(blueberry)
        self.draw_carousel_status()
    
    def get_eject_array(self):
        """Get array indicating which ports should eject based on current carousel state."""
        eject_array = [0, 0, 0]
        for idx, label in enumerate(self.class_labels):
            port_pos = self.eject_ports[label]
            slot_val = self.carousel_slots[port_pos] 
            if slot_val and slot_val.determine_final_class() == label:
                eject_array[idx] = 1
                # Update statistics when ejecting
                self.stats[label] += 1
                self.total_berries_sorted += 1
        print(f"eject_array: {eject_array}")

        """ draw the eject array label on top of the carousel """
        return eject_array
    
    def draw_carousel_status(self):
        """Draws the circular carousel status display."""
        self.canvas.delete("all")
        n = len(self.carousel_slots)
        cx, cy = self.width // 2, self.height // 2 - 20  # Center of the canvas
        r = min(self.width, self.height) // 3.5      # Radius of the circle
        circle_r = 10
        r2 = r + circle_r*2*1.1  # Radius for outer perimeter circle

        self.canvas.create_oval(
            cx - r2, cy - r2,
            cx + r2, cy + r2,
            fill="#fff", outline="#222", width=1
        )                            # Radius of each slot circle

        # Update tooltip if it's currently showing
        if self.tooltip and self.tooltip_slot is not None:
            self.update_tooltip_content()

        # Draw slot circles
        for i, blueberry in enumerate(self.carousel_slots):
            label = blueberry.determine_final_class() if blueberry else None
            angle = -2 * np.pi * (i-1) / n - np.pi/2  # Start at top, clockwise direction
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
            # Highlight selected slot
            if self.selected_slot == i:
                fill = self.COLORS["SELECTED"]["fill"]
                outline = self.COLORS["SELECTED"]["outline"]
            else:
                fill = self.COLORS.get(label, self.COLORS[None])["fill"]
                outline = self.COLORS.get(label, self.COLORS[None])["outline"]
            
            # Create slot circle with tags for hover detection
            slot_id = f"slot_{i}"
            self.canvas.create_oval(
                x - circle_r, y - circle_r, x + circle_r, y + circle_r,
                fill=fill, outline=outline, width=2, tags=slot_id
            )
            self.canvas.create_text(
                x, y, text=str(i), fill="#222", font=("Arial", 10, "bold"), tags=slot_id
            )
            # Optionally, show label as small text below
            if label:
                self.canvas.create_text(
                    x, y + circle_r + 8, text=label[0], fill="#555", font=("Arial", 8), tags=slot_id
                )
        
        # Draw eject port triangles
        triangle_size = circle_r * 2  # Size of the triangle
        
        for label, pos in self.eject_ports.items():
            idx = pos - 1  # 0-based
            angle = -2 * np.pi * idx / n - np.pi/2
            # Triangle tip is triangle_size away from the slot circle edge
            base_dist = r + circle_r + 5
            tip_dist = base_dist + ( triangle_size * ( np.sqrt(3) / 2 ) ) # Tip is further out
            angle_offset = np.arctan(triangle_size / (2 * base_dist))
            x_tip = cx + tip_dist * np.cos(angle)
            y_tip = cy + tip_dist * np.sin(angle)
            x_base1 = cx + base_dist * np.cos(angle - angle_offset)
            y_base1 = cy + base_dist * np.sin(angle - angle_offset)
            x_base2 = cx + base_dist * np.cos(angle + angle_offset)
            y_base2 = cy + base_dist * np.sin(angle + angle_offset)
            # If selected_slot is the label, outline is yellow
            outline_color = "#ffcc00" if self.selected_slot == label else "#222"
            self.canvas.create_polygon(
                [(x_tip, y_tip), (x_base1, y_base1), (x_base2, y_base2)],
                fill=self.COLORS[label]["outline"], outline=outline_color, width=2, tags=(f"eject_{label}",)
            )
            # Add label tag as text near the triangle tip, right edge aligned to tip
            label_x = x_tip - 5  # Align right edge of text to triangle tip
            label_y = y_tip
            font_style = ("Arial", 9, "bold underline") if self.selected_slot == label else ("Arial", 9, "bold")
            self.canvas.create_text(
                label_x, label_y, text=label, fill=self.COLORS[label]["outline"],
                font=font_style, anchor="e", tags=(f"eject_{label}",)
            )

        # draw pie chart in the center of the carousel
        pie_radius = (r - circle_r) * 0.8
        pie_center_x = cx
        pie_center_y = cy
        self.canvas.create_oval(
            pie_center_x - pie_radius, pie_center_y - pie_radius,
            pie_center_x + pie_radius, pie_center_y + pie_radius,
            fill="#fff", outline="#222", width=2
        )
        # Calculate angles for pie chart segments
        total_count = sum(self.stats.values())  
        if total_count > 0:
            start_angle = 0
            for label in self.class_labels:
                count = self.stats[label]
                if count > 0:
                    angle = 2 * np.pi * count / total_count
                    end_angle = start_angle + angle
                    # Draw pie segment
                    self.canvas.create_arc(
                        pie_center_x - pie_radius, pie_center_y - pie_radius,
                        pie_center_x + pie_radius, pie_center_y + pie_radius,
                        start=np.degrees(start_angle), extent=np.degrees(angle),
                        fill=self.COLORS[label]["fill"], outline=self.COLORS[label]["outline"], width=2
                    )
                    start_angle = end_angle
        else:
            # Draw empty pie chart
            self.canvas.create_text(
                pie_center_x, pie_center_y, text="No berries\nsorted yet",
                fill="#666", font=("Arial", 10)
            )
            
        # Draw statistics at bottom of canvas
        stats_data = self.get_statistics()
        total_time = stats_data["total_time"]
        berries_per_minute = stats_data["berries_per_minute"]
        total_berries = stats_data["total_berries"]
        
        # Format time as HH:MM:SS
        hours = int(total_time // 3600)
        minutes = int((total_time % 3600) // 60)
        seconds = int(total_time % 60)
        time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        
        # Position statistics text at the bottom of the canvas
        stats_y = self.height - 40
        self.canvas.create_text(
            cx, stats_y, text=f"Total Time: {time_str}",
            fill="#222", font=("Arial", 10, "bold")
        )
        self.canvas.create_text(
            cx, stats_y + 15, text=f"Rate: {berries_per_minute:.1f} berries/min | Total: {total_berries} berries",
            fill="#222", font=("Arial", 10, "bold")
        )

    def clear_carousel(self):
        """Clear all items from the carousel."""
        self.carousel_slots = deque([None] * self.QUEUE_LENGTH, maxlen=self.QUEUE_LENGTH)
        self.selected_slot = None
        self.draw_carousel_status()
    
    def reset_statistics(self):
        """Reset all statistics."""
        self.stats = {"RIPE": 0, "UNDERRIPE": 0, "OVERRIPE": 0}
        self.start_time = time.time()
        self.total_berries_sorted = 0
    
    def get_statistics(self):
        """Get current statistics."""
        total_time = time.time() - self.start_time
        berries_per_minute = (self.total_berries_sorted / total_time * 60) if total_time > 0 else 0
        return {
            "stats": self.stats.copy(),
            "total_time": total_time,
            "berries_per_minute": berries_per_minute,
            "total_berries": self.total_berries_sorted
        }
    
    def get_carousel_state(self):
        """Get the current state of the carousel."""
        return [blueberry.determine_final_class() if blueberry else None for blueberry in self.carousel_slots]
    
    def on_mouse_motion(self, event):
        """Handle mouse motion over the canvas."""
        # Find which slot the mouse is over
        overlapping = self.canvas.find_overlapping(event.x, event.y, event.x, event.y)
        slot_num = None
        
        for item in overlapping:
            tags = self.canvas.gettags(item)
            for tag in tags:
                if tag.startswith("slot_"):
                    slot_num = int(tag.split("_")[1])
                    break
            if slot_num is not None:
                break
        
        if slot_num is not None:
            if self.tooltip_slot != slot_num:
                self.hide_tooltip()
                self.show_tooltip(event.x, event.y, slot_num)
                self.tooltip_slot = slot_num
        else:
            self.hide_tooltip()

    def set_selected_slot(self, slot_num):
        self.selected_slot = slot_num
        for callback in self.selection_callbacks:
            callback()

    def on_slot_click(self, event):
        """Handle mouse click to select a slot and highlight it."""
        overlapping = self.canvas.find_overlapping(event.x, event.y, event.x, event.y)
        slot_num = None
        for item in overlapping:
            tags = self.canvas.gettags(item)
            for tag in tags:
                if tag.startswith("slot_"):
                    slot_num = int(tag.split("_")[1])
                    break
                elif tag.startswith("eject_"):
                    slot_num = tag.split("_")[1]
                    break
            if slot_num is not None:
                break
        if slot_num is not None:
            self.set_selected_slot(slot_num)
            self.draw_carousel_status()
            self.draw_carousel_status()
        else:
            self.set_selected_slot(None)
            self.draw_carousel_status()
            self.draw_carousel_status()
    
    def show_tooltip(self, x, y, slot_num):
        """Show tooltip with blueberry history for the given slot."""
        blueberry = self.carousel_slots[slot_num]
        if not blueberry or not blueberry.history:
            return
        
        # Create tooltip window
        self.tooltip = tk.Toplevel(self.canvas)
        self.tooltip.wm_overrideredirect(True)
        
        # Position tooltip near mouse cursor
        root_x = self.canvas.winfo_rootx() + x + 10
        root_y = self.canvas.winfo_rooty() + y + 10
        self.tooltip.geometry(f"+{root_x}+{root_y}")
        
        # Populate tooltip content
        self._populate_tooltip_content(blueberry)
    
    def update_tooltip_content(self):
        """Update the tooltip content without changing its position."""
        if not self.tooltip or self.tooltip_slot is None:
            return
        
        blueberry = self.carousel_slots[self.tooltip_slot]
        if not blueberry or not blueberry.history:
            self.hide_tooltip()
            return
        
        # Clear existing content
        for widget in self.tooltip.winfo_children():
            widget.destroy()
        
        # Repopulate tooltip content
        self._populate_tooltip_content(blueberry)
    
    def _populate_tooltip_content(self, blueberry):
        """Helper method to populate tooltip content for a given blueberry."""
        # Add history entries using colored rectangles
        for i, classification in enumerate(blueberry.history):
            color = self.COLORS.get(classification, self.COLORS[None])["fill"]
            tk.Label(
                self.tooltip,
                text=f"{i+1}. {classification}",
                font=("Arial", 9),
                bg=color,
                fg="#000000",
                padx=4,
                pady=2,
                anchor="w"
            ).pack(anchor="w", fill="x")
        
        # Add final classification
        final_class = blueberry.determine_final_class()
        final_color = self.COLORS.get(final_class, self.COLORS[None])["fill"]
        tk.Label(
            self.tooltip,
            text=f"Final: {final_class}",
            font=("Arial", 9, "bold"),
            bg=final_color,
            fg="#000000",
            padx=4,
            pady=4,
            anchor="w"
        ).pack(anchor="w", fill="x")
    
    def hide_tooltip(self, event=None):
        """Hide the tooltip."""
        if self.tooltip:
            self.tooltip.destroy()
            self.tooltip = None
            self.tooltip_slot = None

    def auto_refresh(self):
        """Refresh the carousel display every second to update statistics."""
        self.draw_carousel_status()
        # Schedule the next refresh
        self.parent_frame.after(1000, self.auto_refresh)
    
    def get_selected_blueberries(self):
        """Get the currently selected blueberry object."""
        
        # check if selected slot is a classification label, and not a slot number, and return all images that match
        if isinstance(self.selected_slot, str) and self.selected_slot in self.class_labels:
            # Return all blueberries that match the selected classification label
            return [blueberry for blueberry in self.carousel_slots if blueberry and blueberry.determine_final_class() == self.selected_slot]
        # check if it is a number and within range, and return the blueberry object's images
        elif isinstance(self.selected_slot, int) and 0 <= self.selected_slot < len(self.carousel_slots):
            blueberry = self.carousel_slots[self.selected_slot]
            if blueberry:
                return [blueberry]
            # for debugging, print the blueberry object info:
            print(f"blueberry object at slot {self.selected_slot}: {blueberry}")
        return None
    


class CroppedImageWidget:
    """Widget for displaying cropped images berry image history in a row, scrollable if overflow."""
    # Images are displayed in a row, with the most recent on the left.
    # The selected slot is chosen by clicking.

    def __init__(self, parent_frame, carousel_widget, width=600, height=100):  # Increased default height
        self.parent_frame = parent_frame
        self.carousel_widget = carousel_widget
        self.carousel_widget.register_selection_callback(self.update_display)

        self.width = width
        self.height = height

        # Create main frame
        self.frame = tk.Frame(parent_frame, width=width, height=height)
        self.frame.pack_propagate(False)

        # Create a canvas for scrolling
        self.canvas = tk.Canvas(self.frame, width=width, height=height)
        self.canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

        # Add horizontal scrollbar
        self.scrollbar = tk.Scrollbar(self.frame, orient=tk.HORIZONTAL, command=self.canvas.xview)
        self.scrollbar.pack(side=tk.BOTTOM, fill=tk.X)
        self.canvas.configure(xscrollcommand=self.scrollbar.set)

        # Create image container frame inside canvas
        self.images_frame = tk.Frame(self.canvas, height=height-20)  # Account for scrollbar height
        self.images_frame_id = self.canvas.create_window((0, 0), window=self.images_frame, anchor="nw")

        self.tk_images = []  # Keep references to avoid garbage collection

        # Bind resizing to update scroll region
        self.images_frame.bind("<Configure>", self._on_frame_configure)

        self.update_display()  # Initial display update

    def _on_frame_configure(self, event):
        # Update scroll region to fit the inner frame
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))

    def pack(self, **kwargs):
        """Pack the main frame."""
        self.frame.pack(**kwargs)

    def grid(self, **kwargs):
        """Grid the main frame."""
        self.frame.grid(**kwargs)

    def update_display(self):
        """Update the display with cropped images from the selected slot."""
        # Clear previous images
        for widget in self.images_frame.winfo_children():
            widget.destroy()
        self.tk_images.clear()

        # Get the selected blueberries
        blueberries = self.carousel_widget.get_selected_blueberries()

        if blueberries:
            # Display images from the selected blueberry objects
            for blueberry in blueberries:
                final_class = blueberry.determine_final_class()
                border_color = self.carousel_widget.COLORS.get(final_class, self.carousel_widget.COLORS[None])["outline"]
                for idx, image in enumerate(blueberry.images):
                    if isinstance(image, str):
                        image = Image.open(image)
                    elif isinstance(image, np.ndarray):
                        image = Image.fromarray(image[..., ::-1].astype('uint8'))

                    # Calculate available height for image considering label and padding
                    label_height = 15  # Approximate height for the classification text
                    padding = 6  # Top and bottom padding
                    scrollbar_height = 20  # Height of the scrollbar
                    
                    # Available height for the image
                    img_height = self.height - label_height - padding - scrollbar_height
                    
                    # Resize image: scale height, maintain aspect ratio
                    aspect_ratio = image.width / image.height
                    img_width = int(img_height * aspect_ratio)
                    image = image.resize((img_width, img_height), Image.Resampling.LANCZOS)
                    tk_image = ImageTk.PhotoImage(image)
                    self.tk_images.append(tk_image)  # Prevent garbage collection

                    # Create a frame for image and label
                    img_frame = tk.Frame(self.images_frame)
                    img_frame.pack(side=tk.LEFT, padx=2, pady=2, fill=tk.Y)

                    # Add the label above the image with the classification and score
                    classification = blueberry.history[idx] if idx < len(blueberry.history) else ""
                    score = ""
                    # If classification is a tuple or list, extract score if present
                    if isinstance(classification, (tuple, list)) and len(classification) > 1:
                        score = f" ({classification[1]:.2f})"
                        classification_text = f"{classification[0]}{score}"
                    else:
                        classification_text = str(classification)
                    score_label = tk.Label(
                        img_frame,
                        text=classification_text,
                        font=("Arial", 7),
                        bg="#fff",
                        fg="#222"
                    )
                    score_label.pack(side=tk.TOP, pady=(1, 0))

                    label = tk.Label(
                        img_frame, 
                        image=tk_image, 
                        bd=2, 
                        relief=tk.RIDGE, 
                        highlightbackground=border_color, 
                        highlightcolor=border_color, 
                        highlightthickness=2
                    )
                    label.pack(side=tk.TOP, pady=(0, 2))

        # Update scroll region after adding images
        self.images_frame.update_idletasks()
        self.canvas.configure(scrollregion=self.canvas.bbox("all"))
