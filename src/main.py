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
import pdb


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
        # comm_thread = start_comm_thread(0,interval=1)#0 notes its for serial read updates
        comm_thread_berries = start_comm_thread(1,interval=1)

    #change variables
    motor_speed = 7.0
    motor_throttle = .05
    servo_angles = [100, 50, 120]
    actuation_times = [1200, 1800, 1600]
    new_shutdown = False
    
    # Update variables in the communication module
    #update_variables(motor_speed, servo_angles, new_shutdown, actuation_times)
    control_speed = 2.0
    update_only_motor_speed(control_speed) # goal 7 in/s

    #a list of known blueberries
    persistent_blueberry_tracker = []



    try:
        while True:
            
            #some condition which would require asynchronously changing variable states
            if 1 ==0:
                update_variables(motor_speed, new_shutdown)
            
            update_only_motor_speed(control_speed)


            current_time = time.time()
            if current_time - last_process_time >= capture_interval:
                
                # Grab the latest frame for preview
                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"about to grab frame at: {time.time() - first_time}\n")
                frame = capture_camera_stream(my_cam,-1)
                # frame = cv.imread("/home/blueberryjam/BlueberryJam/img/recolored_unripe_full1.jpg")
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
                    masked_imgs,img_rgb,mask_list = apply_multi_bg(masks,frame=crop_img_center(frame, 360,bias = 250))

                    # cv.imshow("overripe",masked_imgs[0])
                    # cv.imshow("ripe",masked_imgs[1])
                    # cv.imshow("unripe",masked_imgs[2])
                    # cv.imshow("total_mask",masked_imgs[3])
                    #cv.waitKey(0)

                    # print(masked_imgs)
                    # print(img_rgb)
                    processed_imgs = []
                    centroid_results = []
                    existing_centroids = []
                    for i, img in enumerate(mask_list):
                        # overripe, ripe, underripe - i
                        #cv.waitKey(0)
                        processed, centroids = perform_centroiding(img,img_rgb)
                        processed_imgs.append(processed)

                        curr_mask_centroids = [(k[0], k[1]) for k in centroids] # (cX,cY)           
                        #print(curr_mask_centroids)     
                        # print('number of current centroids', len(centroid_results))
                        # print(centroid_results[0]) if len(centroid_results) >0 else print(len(centroid_results))
                        #compared centroids to centroid_results (keep in mind which mask should have priority (overripe))
                        # TODO: check if centroid is too close or the same as an existing centroid in another mask
                        if len(centroid_results) > 0: # only check duplicates if centroids already exist
                            #print(centroid_results)
                            #existing_centroids = [(k[0][0], k[0][1]) for k in centroid_results if k]



                            #intersections = set(existing_centroids) & set(curr_mask_centroids)
                            for j in reversed(curr_mask_centroids):
                                #print(f"existing centroids is {existing_centroids}")
                                is_close = pythag_centroid_euclid(j,existing_centroids,thresh=20)

                                if not is_close:
                                    continue
                                #favors centroids already tracked (by extension overripe)
                                #print(f"popping:{j}")
                                idx = curr_mask_centroids.index(j)
                                centroids.pop(idx)
                                curr_mask_centroids.pop(idx)
                                
                        existing_centroids.extend(curr_mask_centroids)            
                        centroid_results.append(centroids)

                        # print(f"centroid results is {centroid_results}")
                        # print(f"existing_centroids is {existing_centroids}")
                        # pdb.set_trace()
                    #print(centroid_results)
                    #cv.waitKey(0)
                        
                        


                
                
                # cv2.imshow("Processed Stream", processed)

                
                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"finished frame process at: {time.time() - first_time}\n")
                
                #ripeness (unset), belt num, time to actuate (unset), current_location
                # berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=5) #TODO calculate current location
                # blueberry_list = [classify_berry_naive(processed,berry)] #TODO

                blueberry_list = []
                # annotation_space = img_rgb.copy()
                # for centroid in centroids:
                #     loc = centroid[1]
                #     berry = Blueberry(ripeness=0,belt=1,actuation_time=0.0,location_linear=abs(calculate_linear_location(loc)) )
                #     berry.ripeness = classify_single(img_rgb,annotation_space,centroid)
                #     if berry.ripeness == -5:
                #         continue
                    
                #     blueberry_list.append(berry)
                # cv.imshow("annotated image",annotation_space)
                #result = save_annotated_frame(annotation_space)
                #print(result)
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
                    # result = save_annotated_frame(annotation_space)
                    # print(result)


                #blueberry_list = classify_berries(masked,img_rgb,centroids)

                # print(f"persistently pre update {blueberry_positions_to_string(persistent_blueberry_tracker)}")

                #update the tracker with expected blueberry locations
                update_persistence_tracker(persistent_blueberry_tracker,motor_throttle,time.time()-last_process_time,control_speed)
                # print(f"persistently post update {blueberry_positions_to_string(persistent_blueberry_tracker)}")

                #append to the persistent tracker and return the list of untracked blueberries
                # print(f"blueberry_list before sort {blueberry_positions_to_string(blueberry_list)}")
                # print(f"persistently pre sort {blueberry_positions_to_string(persistent_blueberry_tracker)}")

                blueberry_list =compare_and_update_tracker(persistent_blueberry_tracker,blueberry_list)

                # print(f"blueberry_list post sort {blueberry_positions_to_string(blueberry_list)}") 
                # print(f"persistently post sort {blueberry_positions_to_string(persistent_blueberry_tracker)}")

                #pdb.set_trace()
                
                
                send_list = []
                for blueberry_obj in blueberry_list:

                    #print(blueberry_obj)
                    
                    
                    valid_send, berry_candidate = calculate_blueberry_timing(blueberry_obj,motor_throttle,time.time() - time_at_picture,control_speed)
                    
                    #print(berry_candidate)
                    #berry_candidate.actuation_time = 4.0
                    berry_candidate.ripeness =-1
                    if valid_send:
                        print(f"berry actuation time is {berry_candidate.actuation_time} and location (in) is {blueberry_obj.location_linear}") 
                    #valid_send =1

                    if valid_send ==1 and serial_connected:
                        send_list.append(berry_candidate)
                #print(f"updating queue with: {send_list}")q
                update_blueberry_queue(send_list)

                #print(f"full processing in {time.time() - last_process_time}")

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
            # comm_thread.join()
            comm_thread_berries.join()
            ser.close()

if __name__ == "__main__":
    serial_connected = True

    main(serial_connected)
