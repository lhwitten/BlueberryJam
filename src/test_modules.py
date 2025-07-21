#!/usr/bin/env python3
"""
Test script to verify the modular structure works correctly.
This script creates a simple GUI to test the individual modules.
"""

import tkinter as tk
from tkinter import ttk, messagebox
import sys
import os

# Add the current directory to Python path to import our modules
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

try:
    from bbox_manager import BoundingBoxManager
    from serial_manager import SerialManager, SerialControlWidget, PHIndicatorWidget
    from carousel_widget import CarouselStatusWidget
    print("✓ All modules imported successfully!")
except ImportError as e:
    print(f"✗ Import error: {e}")
    sys.exit(1)


class ModuleTestApp:
    """Simple test application to verify module functionality."""
    
    def __init__(self, root):
        self.root = root
        self.root.title("Module Test Application")
        self.root.geometry("800x600")
        
        # Create notebook for tabs
        notebook = ttk.Notebook(root)
        notebook.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # Test BoundingBox Manager
        bbox_frame = ttk.Frame(notebook)
        notebook.add(bbox_frame, text="BBox Manager")
        self.test_bbox_manager(bbox_frame)
        
        # Test Serial Manager
        serial_frame = ttk.Frame(notebook)
        notebook.add(serial_frame, text="Serial Manager")
        self.test_serial_manager(serial_frame)
        
        # Test Carousel Widget
        carousel_frame = ttk.Frame(notebook)
        notebook.add(carousel_frame, text="Carousel Widget")
        self.test_carousel_widget(carousel_frame)
        
        print("✓ Test application initialized successfully!")
    
    def test_bbox_manager(self, parent):
        """Test the BoundingBoxManager module."""
        tk.Label(parent, text="BoundingBox Manager Test", font=("Arial", 16, "bold")).pack(pady=10)
        
        # Create a canvas for testing
        canvas = tk.Canvas(parent, width=400, height=300, bg="white")
        canvas.pack(pady=10)
        
        # Initialize bbox manager
        self.bbox_manager = BoundingBoxManager(canvas, (400, 300))
        
        # Control buttons
        btn_frame = tk.Frame(parent)
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="Toggle Draw Mode", 
                 command=self.bbox_manager.toggle_draw_mode).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Toggle Edit Mode", 
                 command=self.bbox_manager.toggle_edit_mode).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Clear States", 
                 command=self.bbox_manager.clear_states).pack(side=tk.LEFT, padx=5)
        
                        # keyboard bindings
        self.root.bind("<Escape>", lambda e: self.root.quit())
        self.root.bind("<Delete>", self.bbox_manager.delete_selected_bbox())
        self.root.bind("<BackSpace>", self.bbox_manager.delete_selected_bbox())
        self.root.bind("<Control-d>", lambda e: self.toggle_draw_mode())
        self.root.bind("<Control-e>", lambda e: self.toggle_edit_mode())
        
        # Draw initial bboxes
        self.bbox_manager.draw_bboxes()
        
        tk.Label(parent, text="✓ BoundingBox Manager loaded successfully!", 
                fg="green").pack(pady=5)
    
    def test_serial_manager(self, parent):
        """Test the SerialManager module."""
        tk.Label(parent, text="Serial Manager Test", font=("Arial", 16, "bold")).pack(pady=10)
        
        # Initialize serial manager
        self.serial_manager = SerialManager(self.root)
        
        # Create serial control widget
        control_frame = tk.Frame(parent)
        control_frame.pack(fill=tk.X, padx=20, pady=10)
        self.serial_control = SerialControlWidget(control_frame, self.serial_manager)
        
        # Create PH indicator widget
        indicator_frame = tk.Frame(parent)
        indicator_frame.pack(fill=tk.X, padx=20, pady=10)
        tk.Label(indicator_frame, text="PH Trigger Status:").grid(row=0, column=0, sticky="w")
        self.ph_indicator = PHIndicatorWidget(indicator_frame, self.serial_manager)
        
        # Test buttons
        test_frame = tk.Frame(parent)
        test_frame.pack(pady=10)
        
        tk.Button(test_frame, text="Test Eject Command", 
                 command=lambda: self.serial_manager.send_eject_command([1, 0, 1], 5, 3)).pack(side=tk.LEFT, padx=5)
        tk.Button(test_frame, text="Test NEXT Command", 
                 command=self.serial_manager.send_next_command).pack(side=tk.LEFT, padx=5)
        
        # Status
        status_text = "Connected" if self.serial_manager.is_connected() else "Disconnected"
        tk.Label(parent, text=f"✓ Serial Manager loaded successfully! Status: {status_text}", 
                fg="green").pack(pady=5)
    
    def test_carousel_widget(self, parent):
        """Test the CarouselWidget module."""
        tk.Label(parent, text="Carousel Widget Test", font=("Arial", 16, "bold")).pack(pady=10)
        
        # Initialize carousel widget
        carousel_frame = tk.Frame(parent)
        carousel_frame.pack(expand=True, fill=tk.BOTH)
        self.carousel_widget = CarouselStatusWidget(carousel_frame, width=300, height=300)
        
        # Test buttons
        btn_frame = tk.Frame(parent)
        btn_frame.pack(pady=10)
        
        tk.Button(btn_frame, text="Add RIPE", 
                 command=lambda: self.carousel_widget.add_to_carousel("RIPE")).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Add UNDERRIPE", 
                 command=lambda: self.carousel_widget.add_to_carousel("UNDERRIPE")).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Add OVERRIPE", 
                 command=lambda: self.carousel_widget.add_to_carousel("OVERRIPE")).pack(side=tk.LEFT, padx=5)
        tk.Button(btn_frame, text="Clear", 
                 command=self.carousel_widget.clear_carousel).pack(side=tk.LEFT, padx=5)
        
        # Show eject status
        def show_eject_status():
            eject_array = self.carousel_widget.get_eject_array()
            tk.messagebox.showinfo("Eject Status", f"Eject Array: {eject_array}")
        
        tk.Button(btn_frame, text="Show Eject Status", 
                 command=show_eject_status).pack(side=tk.LEFT, padx=5)
        
        tk.Label(parent, text="✓ Carousel Widget loaded successfully!", 
                fg="green").pack(pady=5)


def main():
    """Run the module test application."""
    print("Starting module test application...")
    
    root = tk.Tk()
    app = ModuleTestApp(root)
    
    print("✓ All modules tested successfully!")
    print("You can now interact with each module in the GUI tabs.")
    
    root.mainloop()


if __name__ == "__main__":
    main()
