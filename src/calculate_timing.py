
BELT_DIAMETER = 2 #INCHES
MOTOR_TOP_SPEED = 7000 #RPM
MECHANICAL_REDUCTION = 4 #GEAR RATIO
VISIBLE_BELT_LENGHT = 24 # INCHES
TOTAL_BELT_LENGTH = 24 # INCHES. THE TOTAL LENGTH OF BELT STARTING FROM VISIBLE AREA UNTIL THE END
PIPELINE_COMPUTE_INTERVAL = .57 #seconds. the amount of time it takes for the program to run the vision pipeline


class Blueberry:
    def init(self,ripeness,belt,actuation_time,location_linear):
        self.ripeness = ripeness # -1 - unripe, 0 unclassifies, 1 ripe, 2 overripe
        self.belt = belt # 1 through 3
        self.actuation_time = actuation_time #float: number of seconds til actuation time
        self.location_linear = location_linear #float: num of inches along the belt (as measured by the visible belt length)



def calculate_determination_window(motor_throttle:float):
    """
    return the amount of time the blueberry will be visible under the camera. in seconds
    """
    motor_rpm = MOTOR_TOP_SPEED*motor_throttle/MECHANICAL_REDUCTION
    linear_belt_speed = motor_rpm * 2 * 3.1415926 * (BELT_DIAMETER/2 ) /60 #in/s
    return linear_belt_speed, VISIBLE_BELT_LENGHT/linear_belt_speed #in/s, seconds

def calculate_blueberry_timing(blueberry:Blueberry,motor_throttle):
    """
    
    return (to_actuate, actuation timing)
        to_actuate: 0 - don't actuate, reclassify later
                    1 - use actuation timing
        blueberry - a blueberry class object containing the number of seconds from this moment in which to actuate
    """

    speed, total_visible_time = calculate_determination_window(motor_throttle)

    if total_visible_time < 2*PIPELINE_COMPUTE_INTERVAL:
        print("BELT SPEED IS TOO FAST") #TODO REPLACE WITH ERROR
        return 0,blueberry

    #how far along the belt the blueberry was at time of classification
    lag_distance = speed * PIPELINE_COMPUTE_INTERVAL 
    current_location = blueberry.location_linear  + lag_distance 

    #return nothing because we will have to reclassify it later
    if VISIBLE_BELT_LENGHT - lag_distance > blueberry.location_linear:
        return 0,blueberry

    #TODO DETERMINE IF AN OFFSET IN TIMING IS NEEDED TO FIX ACTUATION TIMING

    blueberry.actuation_time = (TOTAL_BELT_LENGTH - current_location)/speed #the number of seconds until the end of the belt is reached
    return 1, blueberry
