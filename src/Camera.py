import sys
import time
import os
import picamera2
from picamera2 import Picamera2
from datetime import datetime
import cv2

class cam_struct:
    def __init__(self,cam,directory,save_output,stream_type):
        self.cam = cam
        self.directory = directory
        self.save_output = save_output
        self.stream_type = stream_type #0 if static, 1 if stream


def initialize_camera_stream(save_output =0):

    picam2 = Picamera2()

    # Set the capture directory and ensure it exists
    #capture_directory = "../captures/streams/" + f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    capture_directory = "/home/blueberryjam/BlueberryJam/captures/streams/" + f"{datetime.now().strftime('%Y%m%d_%H%M%S')}"
    
    os.makedirs(capture_directory, exist_ok=True)

    # Set the capture interval (in seconds)
    capture_interval = 5  # Adjust this for your preferred interval

    # Start the camera and open preview
    picam2.start()


    stream_cam = cam_struct(picam2,capture_directory,save_output,capture_directory)

    return stream_cam

def capture_camera_stream(camera:cam_struct, override_write = 0):
    """
    Capture a frame from a streaming camera, comes with the option to write it to the camera's directory

    override_write = 1: always write
    override_write = -1: never write
    override_write = 0: use camera struct logic
    """
    frame = camera.cam.capture_array()
    filename = os.path.join(camera.directory, f"image_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jpg")


    if override_write ==-1:
        return frame
    elif override_write ==1:
        camera.cam.capture_file(filename)
    elif camera.save_output ==1:
        camera.cam.capture_file(filename)
    return frame

def end_camera_stream(camera:cam_struct):
        # Stop and close the camera
    camera.cam.stop()
    camera.cam.close()
    cv2.destroyAllWindows()
    print("Camera closed.")