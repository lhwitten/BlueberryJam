from Camera import *
from preprocess_img import preprocess_image

def main():
    capture_interval = .5  # Adjust this for your preferred interval



    last_process_time = time.time()

    my_cam = initialize_camera_stream()

    try:
        while True:

            current_time = time.time()
            if current_time - last_process_time >= capture_interval:
                
                # Grab the latest frame for preview
                frame = capture_camera_stream(my_cam,1)
                
                # Show the video stream (requires display capability)
                cv2.imshow("Camera Stream", frame)
                image_path = 'R_06.11'

                mask = [(0, 70, 100), (95, 125, 134)] # [low, high]

                #processed = preprocess_image(mask,path=None,frame=frame)
                processed = preprocess_image(mask,path=None,frame=frame)
                cv2.imshow("Processed Stream", processed)

                last_process_time = current_time  # Update last capture time

            # Check for the quit key ('q') to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping capture...")
    finally:
        end_camera_stream(my_cam)

if __name__ == "__main__":
    main()