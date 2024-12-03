from Camera import *
from preprocess_img import preprocess_image
from preprocess_img import classify_berry_naive
from Classifier import *
from preprocess_img import *
from calculate_timing import calculate_blueberry_timing
from calculate_timing import *
import serial
import threading
import time
from Serial_connector import *
from calculate_timing import Blueberry

def main(serial_connected = True):
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

    if serial_connected:
        #serial communication
        comm_thread = start_comm_thread(0,interval=1)#0 notes its for variable updates
        comm_thread_berries = start_comm_thread(1,interval=1)

    #change variables
    motor_speed = 200
    motor_throttle = .05
    servo_angles = [100, 50, 120]
    actuation_times = [1200, 1800, 1600]
    new_shutdown = False
    
    # Update variables in the communication module
    update_variables(motor_speed, servo_angles, new_shutdown, actuation_times)

    #a list of known blueberries
    persistent_blueberry_tracker = []



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
                time_at_picture = time.time()
                
                # Show the video stream (requires display capability)
                cv2.imshow("Camera Stream", frame)
                image_path = 'R_06.11'

                #mask = [(0, 70, 100), (95, 125, 134)] # [low, high]
                #mask = [(0, 70, 100), (95, 140, 134)] # [low, high] #mask for only overripe
                #mask = [(0, 70, 100), (95, 140, 137)] #better mask for just overripe
                #mask = [(0, 70, 100), (95, 140, 150)] # mask for just overripe and ripe
                mask = [(0, 70, 100), (95, 140, 160)] #mask for all 3
                #mask = [(0, 102, 138), (95, 140, 150)] #mask for just ripe
                #mask = [(0, 70, 150), (95, 140, 160)] #mask for just green
                #user_input = input("input mask")
                masks = []
                #comment this out otherwise
                #overripe, ripe, underripe(#TODO)
                #masks = [[(0, 70, 100), (95, 140, 137)],[(0, 102, 138), (95, 140, 150)],[(0, 70, 100), (95, 140, 160)]]
                masks = [[(20, 0, 0), (180, 85, 65)],[(0, 60, 0), (38, 145, 100)],[(0, 135, 0), (180, 255, 150)]]

                
                # with open(output_folder + log_file,"a") as f:

                #     f.write(f"about to process frame at: {time.time() - first_time}\n")

                #processed = preprocess_image(mask,path=None,frame=None)
                #processed = preprocess_image(mask,path=None,frame=frame)
                frame = cv.rotate(frame,cv.ROTATE_90_COUNTERCLOCKWISE)
                
                if not masks:
                    masked,img_rgb = perform_backgrounding(mask,frame=crop_img_center(frame, 400,bias = 180))
                    processed, centroids = perform_centroiding(masked,img_rgb)
                else:
                    # for multiple masks
                    masked_imgs,img_rgb,mask_list = apply_multi_bg(masks,frame=crop_img_center(frame, 400,bias = 180))

                    cv.imshow("overripe",masked_imgs[0])
                    cv.imshow("ripe",masked_imgs[1])
                    cv.imshow("unripe",masked_imgs[2])
                    cv.imshow("total_mask",masked_imgs[3])
                    #cv.waitKey(0)

                    # print(masked_imgs)
                    # print(img_rgb)
                    processed_imgs = []
                    centroid_results = []
                    for i, img in enumerate(mask_list):
                        #cv.waitKey(0)
                        processed, centroids = perform_centroiding(img,img_rgb)
                        processed_imgs.append(processed)
                        centroid_results.append(centroids)

                
                
                # cv2.imshow("Processed Stream", processed)

                
                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"finished frame process at: {time.time() - first_time}\n")
                
                #ripeness (unset), belt num, time to actuate (unset), current_location
                berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=5) #TODO calculate current location



                
                blueberry_list = [classify_berry_naive(processed,berry)] #TODO

                blueberry_list = []
                annotation_space = img_rgb.copy()
                for centroid in centroids:
                    loc = centroid[1]
                    berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=abs(calculate_linear_location(loc)) )
                    berry.ripeness = classify_single(img_rgb,annotation_space,centroid)
                    if berry.ripeness == -5:
                        continue
                    
                    blueberry_list.append(berry)
                cv.imshow("annotated image",annotation_space)
                result = save_annotated_frame(annotation_space)
                print(result)
                if not masks:
                    annotation_space = img_rgb.copy()
                    for centroid in centroids:
                        loc = centroid[1]
                        berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=abs(calculate_linear_location(loc)) )
                        berry.ripeness = classify_single(img_rgb,annotation_space,centroid)
                        if berry.ripeness == -5:
                            continue
                        
                        blueberry_list.append(berry)
                    cv.imshow("annotated image",annotation_space)
                else:
                    annotation_space = img_rgb.copy()
                    for i,centroid_list in enumerate(centroid_results):

                        for centroid in centroid_list:
                            loc = centroid[1]
                            berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=abs(calculate_linear_location(loc)) )
                            #overripe, ripe, underripe
                            if i ==0:#ovveripe
                                berry.ripeness = 2
                                similarities = [0.0,1.0,0.0] #ripe, overripe, underripe
                                is_ripe =1
                            elif i ==1: #ripe
                                berry.ripeness = 1
                                similarities = [1.0,0.0,0.0]
                                is_ripe = 0
                            elif i ==2: #underripe
                                berry.ripeness = -1
                                similarities = [0.0,0.0,1.0]
                                is_ripe = 2
                            else:
                                continue #for full masked image

                            #berry.ripeness = classify_single(img_rgb,annotation_space,centroid)
                            # if berry.ripeness == -5:
                            #     continue
                            annotate_and_show(annotation_space,centroid,similarities,is_ripe)
                        
                            blueberry_list.append(berry)
                    
                    cv.imshow("annotated image",annotation_space)
                    result = save_annotated_frame(annotation_space)
                    print(result)


                #blueberry_list = classify_berries(masked,img_rgb,centroids)

                #update the tracker with expected blueberry locations
                update_persistence_tracker(persistent_blueberry_tracker,motor_throttle,time.time()-last_process_time)

                #append to the persistent tracker and return the list of untracked blueberries
                blueberry_list =compare_and_update_tracker(persistent_blueberry_tracker,blueberry_list) 
                
                send_list = []
                for blueberry_obj in blueberry_list:
                    
                    
                    valid_send, berry_candidate = (calculate_blueberry_timing(blueberry_obj,motor_throttle,time.time() - time_at_picture))
                    berry_candidate.actuation_time = 4.0
                    berry_candidate.ripeness =-1
                    print(f"berry actuation time is {berry_candidate.actuation_time} and valid send is {valid_send}")
                    valid_send =1

                    if valid_send ==1 and serial_connected:
                        send_list.append(berry_candidate)
                #print(f"updating queue with: {send_list}")
                update_blueberry_queue(send_list)

                last_process_time = time.time()  # Update last capture time

                


            # Check for the quit key ('q') to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    except KeyboardInterrupt:
        print("Stopping capture...")
    finally:
        end_camera_stream(my_cam)
        #serial communication shutdown
        shutdown = True

        if serial_connected:
            comm_thread.join()
            comm_thread_berries.join()
            ser.close()

if __name__ == "__main__":
    serial_connected = False

    main(serial_connected)
