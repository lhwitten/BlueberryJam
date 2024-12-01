
BELT_DIAMETER = 2 #INCHES
MOTOR_TOP_SPEED = 7000 #RPM
MECHANICAL_REDUCTION = 4 #GEAR RATIO
VISIBLE_BELT_LENGHT = 24 # INCHES
TOTAL_BELT_LENGTH = 24 # INCHES. THE TOTAL LENGTH OF BELT STARTING FROM VISIBLE AREA UNTIL THE END
PIPELINE_COMPUTE_INTERVAL = .57 #seconds. the amount of time it takes for the program to run the vision pipeline
PIXELS_PER_INCH = 2592/3.5 #TODO measure values

class Blueberry:
    def __init__(self,ripeness,belt,actuation_time,location_linear):
        self.ripeness = ripeness # -1 - unripe, 0 unclassifies, 1 ripe, 2 overripe
        self.belt = belt # 1 through 3
        self.actuation_time = actuation_time #float: number of seconds til actuation time
        self.location_linear = location_linear #float: num of inches along the belt (as measured by the visible belt length)

def calculate_linear_location(pixel_location):
    return pixel_location /PIXELS_PER_INCH

def calculate_determination_window(motor_throttle:float):
    """
    return the amount of time the blueberry will be visible under the camera. in seconds
    """
    motor_rpm = MOTOR_TOP_SPEED*motor_throttle/MECHANICAL_REDUCTION
    linear_belt_speed = motor_rpm * 2 * 3.1415926 * (BELT_DIAMETER/2 ) /60 #in/s
    return linear_belt_speed, VISIBLE_BELT_LENGHT/linear_belt_speed #in/s, seconds

def calculate_blueberry_timing(blueberry:Blueberry,motor_throttle,pipeline_compute_interval):
    """
    
    return (to_actuate, actuation timing)
        to_actuate: 0 - don't actuate, reclassify later
                    1 - use actuation timing
        blueberry - a blueberry class object containing the number of seconds from this moment in which to actuate
    """

    speed, total_visible_time = calculate_determination_window(motor_throttle)

    # if total_visible_time < 2*PIPELINE_COMPUTE_INTERVAL:
    #     print("BELT SPEED IS TOO FAST") #TODO REPLACE WITH ERROR
    #     return 0,blueberry

    #how far along the belt the blueberry was at time of classification
    lag_distance = speed*pipeline_compute_interval 
    #lag_distance = speed * PIPELINE_COMPUTE_INTERVAL 
    current_location = blueberry.location_linear  + lag_distance 

    # #return nothing because we will have to reclassify it later
    # if VISIBLE_BELT_LENGHT - lag_distance > blueberry.location_linear:
    #     return 0,blueberry

    #TODO DETERMINE IF AN OFFSET IN TIMING IS NEEDED TO FIX ACTUATION TIMING
    print(f"current location,speed is {current_location},{speed}")

    blueberry.actuation_time = (TOTAL_BELT_LENGTH - current_location)/speed #the number of seconds until the end of the belt is reached

    #make this ms at the end
    return 1, blueberry

def update_persistence_tracker(tracked_blueberries:list[Blueberry],motor_throttle,time_interval):
    """
    update every blueberry in the list based on the belt speed and time
    """
    speed, total_visible_time = calculate_determination_window(motor_throttle)

    moved_distance = speed*time_interval
    num_berries = len(tracked_blueberries)

    for i in reversed(range(num_berries)):

        berry = tracked_blueberries[i]

        if berry.location_linear + moved_distance >VISIBLE_BELT_LENGHT:
            tracked_blueberries.pop(i)
        else:
            tracked_blueberries[i].location_linear += moved_distance
    

def compare_and_update_tracker(tracked_blueberries:list[Blueberry],found_berries:list[Blueberry]):
    """
    Compares the visible berries against the blueberries currently being tracked. Returns a new list which should be sent
    """

    berries_to_send = []

    for blueberry in found_berries:
        
        #check the berry against the list and update as necessary
        is_send = compare_single_berry_to_tracker(tracked_blueberries,blueberry)

        if is_send:
            berries_to_send.append(blueberry)
    return berries_to_send

def compare_single_berry_to_tracker(tracked_blueberries:list[Blueberry],to_compare:Blueberry):

    #closeness threshold
    thresh = .2 #inches
    in_list = False
    for tracked in tracked_blueberries:

        if tracked.location_linear - thresh < to_compare.location_linear < tracked.location_linear + thresh:
            in_list = True
            break
    
    if in_list:
        # do not send to arduino
        return False
    else:
        tracked_blueberries.append(to_compare)
        #Send to Arduino
        return True