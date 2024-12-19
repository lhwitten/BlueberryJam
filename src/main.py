from Camera import *
from preprocess_img import preprocess_image
from preprocess_img import classify_berry_naive
from Classifier import *
from preprocess_img import *
from calculate_timing import calculate_blueberry_timing
from calculate_timing import *
from buttons_screen import *
import serial
import threading
import random
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
    button_thread = start_button_thread()
    lcd_init()
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
    stop_speed = -5.0
    current_error = 0
    stop_time_before_quit = 20
    update_only_motor_speed(stop_speed) # goal 7 in/s
    #wait for start
    construct_and_send_LCD(control_speed,True,current_error)
    new_stop_press, new_start_press = False,False

    # stop_time = time.time()
    # while not new_start_press:
    #     new_start_press, new_stop_press = were_buttons_pressed()
    #     time.sleep(.05)
    #     if time.time() - stop_time > stop_time_before_quit:
    #         break
            
    # update_only_motor_speed(control_speed)
    # reset_buttons()
    # time.sleep(.5)

    #a list of known blueberries
    persistent_blueberry_tracker = []

    # construct_and_send_LCD(control_speed,False,current_error)
    first_wait = True
    try:
        while True:
            
            #some condition which would require asynchronously changing variable states
            if 1 ==0:
                update_variables(motor_speed, new_shutdown)
            
            #start and stop based on buttons
            new_start_press, new_stop_press = were_buttons_pressed()

        
            if not new_stop_press and not first_wait:
                update_only_motor_speed(control_speed)
            else:
                first_wait = False   
                update_only_motor_speed(stop_speed)
                construct_and_send_LCD(control_speed,True,current_error)
                # stop_time = time.time()
                while not new_start_press:
                    new_start_press, new_stop_press = were_buttons_pressed()
                    time.sleep(.05)
                    # if time.time() - stop_time > stop_time_before_quit:
                    #     raise Exception("wait too long on button")
                update_only_motor_speed(control_speed)
                construct_and_send_LCD(control_speed,False,current_error)
                reset_buttons()
                time.sleep(.5)
                

            reset_buttons()



            current_time = time.time()
            if current_time - last_process_time >= capture_interval:
                
                # Grab the latest frame for preview
                # with open(output_folder + log_file,"a") as f:
                #     f.write(f"about to grab frame at: {time.time() - first_time}\n")

                frame = capture_camera_stream(my_cam,-1)
                # frame = cv.imread("/home/blueberryjam/BlueberryJam/img/recolored_unripe_full1.jpg")
                time_at_picture = time.time()
                
                # Show the video stream (requires display capability)
                #cv2.imshow("Camera Stream", frame)
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

                frame = cv.rotate(frame,cv.ROTATE_90_CLOCKWISE)


                
                if not masks:
                    masked,img_rgb = perform_backgrounding(mask,frame=crop_img_center(frame, 450,bias = 140))
                    processed, centroids = perform_centroiding(masked,img_rgb)
                else:
                    # for multiple masks
                    masked_imgs,img_rgb,mask_list = apply_multi_bg(masks,frame=crop_img_center(frame, 195,bias = 9))

                    cv.imshow("overripe",masked_imgs[0])
                    cv.imshow("ripe",masked_imgs[1])
                    cv.imshow("unripe",masked_imgs[2])
                    # cv.imshow("total_mask",masked_imgs[3])
                    #cv.waitKey(0)

                    # print(masked_imgs)
                    # print(img_rgb)

                    # the following code compares the lists of centroids
                    processed_imgs, centroid_results, existing_centroids = process_mask_centroids(mask_list,img_rgb)

                    print(centroid_results)
                        

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
                    #cv.imshow("annotated image",annotation_space)
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

                            quality_score = centroid[5]

                            annotate_and_show(annotation_space,centroid,similarities,is_ripe,quality=quality_score)
                        
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

                    
                    
                    #blueberry_obj.ripeness = 2 #random.choice([-1,2])
                    blueberry_obj.ripeness = random.choice([-1,2])
                    valid_send, berry_candidate = calculate_blueberry_timing(blueberry_obj,motor_throttle,time.time() - time_at_picture,control_speed)
                    
                    #print(berry_candidate)
                    #berry_candidate.actuation_time = 4.0
                    
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

    except Exception as e:
        print(f"Ending, exception:{e}")
        print("Stopping capture...")
    finally:
        current_error = 9999
        end_camera_stream(my_cam)
        #serial communication shutdown
        shutdown = True
        update_only_motor_speed(stop_speed)
        construct_and_send_LCD(goal_speed=-9.999,is_stopped=True,Error_code=current_error)
        button_thread.join()
        cleanup_gpio()

        if serial_connected:
            # comm_thread.join()
            comm_thread_berries.join()
            ser.close()
            

if __name__ == "__main__":
    serial_connected = True

    main(serial_connected)
