# BlueberryJam - Modular Code Structure

The code has been refactored into separate modules for better organization and maintainability.

## Module Structure

### `src/yolo_testing.py` (Main Application)
- Main application class `WebcamApp`
- Camera handling and YOLO model integration
- GUI layout and controls
- Classification logic and analysis

### `src/bbox_manager.py` (Bounding Box Management)
- `BoundingBoxManager` class for handling bounding box operations
- Drawing, editing, and interaction with bounding boxes
- Mouse event handling for draw/edit modes
- State management for bounding box data

### `src/serial_manager.py` (Serial Communication)
- `SerialManager` class for serial port communication
- `SerialControlWidget` for GUI serial controls
- `PHIndicatorWidget` for PH trigger status display
- Background thread for serial listening
- Command sending for eject operations

### `src/carousel_widget.py` (Carousel Status Display)
- `CarouselStatusWidget` class for circular carousel visualization
- Queue management for berry classifications
- Eject port calculation and display
- Visual representation of carousel state

## Key Benefits of This Structure

1. **Separation of Concerns**: Each module handles a specific aspect of the application
2. **Reusability**: Modules can be used independently or in other projects
3. **Maintainability**: Easier to debug and modify specific functionality
4. **Testability**: Individual modules can be tested separately
5. **Cleaner Code**: Main application is less cluttered and more focused

## Usage

The main application imports and uses these modules:

```python
from bbox_manager import BoundingBoxManager
from serial_manager import SerialManager, SerialControlWidget, PHIndicatorWidget
from carousel_widget import CarouselStatusWidget
```

Each module is self-contained and provides a clear API for interaction with the main application.

## Dependencies

- `bbox_manager.py`: Standard tkinter libraries
- `serial_manager.py`: `pyserial` library for serial communication
- `carousel_widget.py`: Standard tkinter and numpy
- Main application: PIL, OpenCV, ultralytics YOLO, torch

## Future Enhancements

This modular structure makes it easy to:
- Add new visualization widgets
- Implement different communication protocols
- Enhance bounding box functionality
- Add new classification methods
- Create unit tests for each module
