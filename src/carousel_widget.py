import tkinter as tk
import numpy as np
from collections import deque, Counter
import time

class Blueberry:
    """Represents a blueberry with classification history."""
    def __init__(self, initial_classification):
        self.history = [initial_classification]  # Stores classification attempts

    def add_classification_attempt(self, classification):
        """Add a classification attempt to the history."""
        self.history.append(classification)

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
        None: {"fill": "#eeeeee", "outline": "#cccccc"}
    }

    def __init__(self, parent_frame, width=450, height=450):
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
        
        # Create canvas
        self.canvas = tk.Canvas(parent_frame, width=width, height=height, highlightthickness=0)
        self.canvas.pack(side=tk.BOTTOM, pady=(16, 0))
        
        # Create label
        self.label = tk.Label(parent_frame, text="Carousel Slots:")
        self.label.pack(side=tk.BOTTOM, pady=(0, 2))
        
        # Tooltip variables
        self.tooltip = None
        self.tooltip_slot = None
        
        # Bind mouse events for tooltip
        self.canvas.bind("<Motion>", self.on_mouse_motion)
        self.canvas.bind("<Leave>", self.hide_tooltip)
        
        # Initial draw
        self.draw_carousel_status()
    
    def add_to_carousel(self, initial_classification):
        """Add a new Blueberry object to the carousel queue."""
        blueberry = Blueberry(initial_classification)
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
        return eject_array
    
    def draw_carousel_status(self):
        """Draws the circular carousel status display."""
        self.canvas.delete("all")
        n = len(self.carousel_slots)
        cx, cy = self.width // 2, self.height // 2  # Center of the canvas
        r = min(self.width, self.height) // 3       # Radius of the circle
        circle_r = 10                               # Radius of each slot circle

        # Update tooltip if it's currently showing
        if self.tooltip and self.tooltip_slot is not None:
            self.update_tooltip_content()

        # Draw slot circles
        for i, blueberry in enumerate(self.carousel_slots):
            label = blueberry.determine_final_class() if blueberry else None
            angle = -2 * np.pi * (i-1) / n - np.pi/2  # Start at top, clockwise direction
            x = cx + r * np.cos(angle)
            y = cy + r * np.sin(angle)
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
        triangle_size = 18
        for label, pos in self.eject_ports.items():
            idx = pos - 1  # 0-based
            angle = -2 * np.pi * idx / n - np.pi/2
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
            self.canvas.create_polygon(
                [(x_tip, y_tip), (x_base1, y_base1), (x_base2, y_base2)],
                fill=self.COLORS[label]["outline"], outline="#222", width=2
            )
    def clear_carousel(self):
        """Clear all items from the carousel."""
        self.carousel_slots = deque([None] * self.QUEUE_LENGTH, maxlen=self.QUEUE_LENGTH)
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


class StatisticsWidget:
    """Widget for displaying sorting statistics with a horizontal stacked bar chart."""
    
    def __init__(self, parent_frame, carousel_widget, width=600, height=85):
        self.parent_frame = parent_frame
        self.carousel_widget = carousel_widget
        self.width = width
        self.height = height
        
        # Create main frame
        self.frame = tk.Frame(parent_frame)
        
        # Create text frame for statistics
        self.text_frame = tk.Frame(self.frame)
        self.text_frame.pack(anchor="w", side=tk.LEFT, padx=(0, 20))
        
        # Statistics labels
        self.time_label = tk.Label(self.text_frame, text="Total Time: 0:00:00", font=("Arial", 12), anchor="w", justify="left")
        self.time_label.pack(anchor="w")
        
        self.rate_label = tk.Label(self.text_frame, text="Rate: 0.0 berries/min", font=("Arial", 12), anchor="w", justify="left")
        self.rate_label.pack(anchor="w")
        
        self.total_label = tk.Label(self.text_frame, text="Total: 0 berries", font=("Arial", 12), anchor="w", justify="left")
        self.total_label.pack(anchor="w")
        
        # Create canvas for the bar chart - fixed width for now
        self.canvas_width = 400
        # self.canvas = tk.Canvas(self.frame, width=self.canvas_width, height=height, highlightthickness=0)
        # self.canvas.pack(side=tk.LEFT, padx=(20, 20))

        self.canvas = tk.Canvas(self.frame, height=height, highlightthickness=0)
        self.canvas.pack(side=tk.LEFT, fill='both', expand=True, padx=(0, 40))
        
        # Initial draw
        self.update_display()
    
    def pack(self, **kwargs):
        """Pack the main frame."""
        self.frame.pack(**kwargs)
    
    def grid(self, **kwargs):
        """Grid the main frame."""
        self.frame.grid(**kwargs)
    
    def update_display(self):
        """Update the statistics display."""
        stats_data = self.carousel_widget.get_statistics()
        self.draw_bar_chart(stats_data)
        self.update_text_labels(stats_data)
    
    def draw_bar_chart(self, stats_data):
        """Draw the horizontal stacked bar chart."""
        self.canvas.delete("all")
        
        stats = stats_data["stats"]
        total = stats_data["total_berries"]
        
        if total == 0:
            # Draw empty bar
            self.canvas.create_rectangle(10, 20, self.canvas.winfo_width() - 20, 40, 
                                       fill="#eeeeee", outline="#cccccc", width=2)
            self.canvas.create_text(self.canvas.winfo_width() // 2, 30, text="No berries sorted yet", 
                                  fill="#666", font=("Arial", 10))
            return
        
        # Calculate percentages and bar segments
        bar_x = 10
        bar_y1 = 20
        bar_y2 = 60
        self.canvas.update_idletasks()
        bar_width = self.canvas.winfo_width() - 50
        
        current_x = bar_x
        
        # Draw segments for each classification
        for label in ["RIPE", "UNDERRIPE", "OVERRIPE"]:
            count = stats[label]
            if count > 0:
                percentage = count / total
                segment_width = bar_width * percentage
                
                color = self.carousel_widget.COLORS[label]["fill"]
                outline = self.carousel_widget.COLORS[label]["outline"]
                
                self.canvas.create_rectangle(current_x, bar_y1, current_x + segment_width, bar_y2,
                                           fill=color,)
                
                # Add percentage text if segment is wide enough
                if segment_width > 40:
                    text_x = current_x + segment_width / 2
                    self.canvas.create_text(text_x, 30, text=f"{percentage*100:.1f}%",
                                          fill="#000", font=("Arial", 9, "bold"))
                
                current_x += segment_width
        
        # Add border around entire bar
        self.canvas.create_rectangle(bar_x, bar_y1, bar_x + bar_width, bar_y2,
                                   fill="")
        
        # Add legend below the bar
        legend_y = 70
        legend_x = bar_x
        
        for i, label in enumerate(["RIPE", "UNDERRIPE", "OVERRIPE"]):
            count = stats[label]
            percentage = (count / total * 100) if total > 0 else 0
            color = self.carousel_widget.COLORS[label]["fill"]
            
            # Small colored square
            self.canvas.create_rectangle(legend_x, legend_y, legend_x + 10, legend_y + 10,
                                       fill=color, outline="#333", width=1)
            
            # Label text
            text = f"{label}: {count} ({percentage:.1f}%)"
            self.canvas.create_text(legend_x + 15, legend_y + 5, text=text,
                                  anchor="w", fill="#333", font=("Arial", 8))
            
            # Move to next position
            text_width = len(text) * 6 + 25  # Approximate text width
            legend_x += text_width + 20
    
    def update_text_labels(self, stats_data):
        """Update the text labels with current statistics."""
        total_time = stats_data["total_time"]
        berries_per_minute = stats_data["berries_per_minute"]
        total_berries = stats_data["total_berries"]
        
        # Format time as HH:MM:SS
        hours = int(total_time // 3600)
        minutes = int((total_time % 3600) // 60)
        seconds = int(total_time % 60)
        time_str = f"{hours:02d}:{minutes:02d}:{seconds:02d}"
        
        self.time_label.config(text=f"Total Time: {time_str}")
        self.rate_label.config(text=f"Rate: {berries_per_minute:.1f} berries/min")
        self.total_label.config(text=f"Total: {total_berries} berries")
