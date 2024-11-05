from Camera import *
from preprocess_img import preprocess_image
from calculate_timing import calculate_blueberry_timing
import serial
import threading
import time
from Serial_connector import *


def main():
    capture_interval = .01  # Adjust this for your preferred interval

    #cv2.getBuildInformation()
    first_time =time.time()
    last_process_time = first_time
    output_folder = '/home/blueberryjam/BlueberryJam/logs/'
    log_file = datetime.now().strftime('%Y%m%d_%H%M%S') + ".txt"

    #datetime.now().strftime('%Y%m%d_%H%M%S')
    # with open(output_folder + log_file,"a") as f: #UNCOMMENT FOR LOGGING. #TODO implement logging option
    #     f.write(f"Program Start at: {time.time() - first_time}\n capture interval is {capture_interval}\n")

    my_cam = initialize_camera_stream()

    #serial communication
    comm_thread = start_comm_thread(0,interval=1)#0 notes its for variable updates
    comm_thread_berries = start_comm_thread(1,interval=1)

    #change variables
    motor_speed = 200
    servo_angles = [100, 50, 120]
    actuation_times = [1200, 1800, 1600]
    new_shutdown = False
    
    # Update variables in the communication module
    update_variables(motor_speed, servo_angles, new_shutdown, actuation_times)



    try:
        while True:
            
            #some condition which would require asynchronously changing variable states
            if 1 ==0:
                update_variables(motor_speed, new_shutdown)


            current_time = time.time()
            if current_time - last_process_time >= capture_interval:
                
                # Grab the latest frame for preview
                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"about to grab frame at: {time.time() - first_time}\n")
                frame = capture_camera_stream(my_cam,-1)
                
                # Show the video stream (requires display capability)
                # cv2.imshow("Camera Stream", frame)
                image_path = 'R_06.11'

                mask = [(0, 70, 100), (95, 125, 134)] # [low, high]
                # with open(output_folder + log_file,"a") as f:

                #     f.write(f"about to process frame at: {time.time() - first_time}\n")

                #processed = preprocess_image(mask,path=None,frame=frame)
                processed = preprocess_image(mask,path=None,frame=frame)
                # cv2.imshow("Processed Stream", processed)

                last_process_time = current_time  # Update last capture time

                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"finished frame process at: {time.time() - first_time}\n")

                blueberry_list = classify(processed) #TODO
                send_list = []
                for blueberry_obj in blueberry_list:
                    
                    valid_send, berry_candidate = calculate_blueberry_timing(blueberry_obj)

                    if valid_send ==1:
                        send_list.append(berry_candidate)
                
                update_blueberry_queue(send_list)
                


            # Check for the quit key ('q') to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping capture...")
    finally:
        end_camera_stream(my_cam)
        #serial communication shutdown
        shutdown = True
        comm_thread.join()
        comm_thread_berries.join()
        ser.close()

if __name__ == "__main__":
    main()