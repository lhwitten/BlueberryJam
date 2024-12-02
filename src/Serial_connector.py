# serial_comm.py
import serial
import threading
import time
import os
import copy
from queue import Queue
from calculate_timing import Blueberry
blueberry_process_queue = Queue()

#TODO write procedure for finding correct port

ports = ['/dev/ttyACM1','/dev/ttyACM0','/dev/ttyACM3','/dev/ttyACM4']
ser = None
for port in ports:
    if os.path.exists(port):
        ser = serial.Serial(port, 9600, timeout=1)  # Adjust port and baud rate as necessary


# Initialize variables with timestamps
motor_speed = 7.0
servo_angles = [90, 45, 135]
shutdown = False
actuation_times = [1000, 2000, 1500]
last_update_time = time.monotonic()
#blueberry_process_queue =[]

def return_ser():

    return ser

def send_data(motor_speed, shutdown, update_time):
    """
    Sends a data packet to the Arduino with timing information.
    """
    belt_num = 0
    ripeness = 0
    time1 =0.0
    elapsed_time = time.monotonic() - update_time
    motor_update = 1
    #data_packet = f"{motor_speed}|{belt_num}|{ripeness}|{int(shutdown)}|{time1}|{elapsed_time:.3f}\n"
    data_packet = f"{motor_speed:.3f}|{belt_num}|{ripeness}|{int(shutdown)}|{time1:.3f}|{elapsed_time:.3f}|{motor_update}\n"
    #ser.write(f"{data_packet}".encode('utf-8'))

    #print("sending motor update packet")
    #print(data_packet)
def async_serial_comm_read(interval =1):
    try:
        while ser.in_waiting > 0:
            response = ser.readline().decode('utf-8').strip()
            print(f"Arduino: {response}")
    except Exception as e:
        print(f"Error reading response: {e}")

def async_serial_comm_update_vars(interval=1):
    """
    Runs non-blocking serial communication to send data at specified intervals.
    """
    global last_update_time, motor_speed, servo_angles, shutdown, actuation_times
    while not shutdown:
        send_data(motor_speed, shutdown, last_update_time)
        time.sleep(interval)

def async_serial_comm_blueberry_list(interval=1):
    """
    Runs non-blocking serial communication to send data at specified intervals.
    """
    global last_update_time, motor_speed, servo_angles, shutdown, actuation_times, blueberry_process_queue
    while not shutdown:
        #print(f"running async serial comm blueberry list with list:{blueberry_process_queue}")
        send_blueberry_list_data(blueberry_process_queue,motor_speed, shutdown, last_update_time)
        time.sleep(interval)
        #blueberry_process_queue.clear() #empty the list

def update_variables(new_motor_speed, new_servo_angles, new_shutdown, new_actuation_times):
    """
    Updates motor speed, servo angles, and actuation times, and refreshes the update timestamp.
    """
    global motor_speed, servo_angles, shutdown, actuation_times, last_update_time
    motor_speed = new_motor_speed
    servo_angles = new_servo_angles
    shutdown = new_shutdown
    actuation_times = new_actuation_times
    last_update_time = time.monotonic()

def update_blueberry_queue(blueberry_list):
    #global last_update_time, blueberry_process_queue
    # blueberry_process_queue = copy.deepcopy(blueberry_list)
    global last_update_time
    for blueberry in blueberry_list:
        blueberry_process_queue.put(blueberry)  # Add each item to the thread-safe queue

    last_update_time = time.monotonic()

    # print("blueberry queue updated")
    # print(f"blueberry queue is {blueberry_process_queue}")

def update_only_motor_speed(speed_value:float):
    #updates only the motor_speed by sending a special blueberry
    blueberry = Blueberry(0,0,0.0,0.0,motor_update=1,motor_speed=speed_value)
    update_blueberry_queue([blueberry])


def start_comm_thread(option,interval=1):
    """
    Starts the asynchronous communication thread.
    """
    if option ==0:
        print("starting motor update thread")
        comm_thread = threading.Thread(target=async_serial_comm_read, args=(interval,), daemon=True)
    else:
        print("starting blueberry send thread")
        comm_thread = threading.Thread(target=async_serial_comm_blueberry_list, args=(interval,), daemon=True)

    comm_thread.start()
    return comm_thread


def send_blueberry_list_data(process_queue,motor_speed, shutdown, update_time):
    """
    Sends a data packet to the Arduino with timing information.
    """
    elapsed_time = time.monotonic() - update_time
    #print("running send blueberry list data")
    #print(process_queue)

    # if not blueberry_process_queue.empty():
    #         while not blueberry_process_queue.empty():
    #             blueberry = blueberry_process_queue.get()  # Thread-safe dequeue
    #             # Process each blueberry
    #             print(f"Processing: {blueberry}")

    # for berry_send in process_queue:
    #     time1 = berry_send.actuation_time
    #     belt_num = berry_send.belt
    #     ripeness= berry_send.ripeness
    #     motor_update =0

    #     data_packet = f"{motor_speed:.3f}|{belt_num}|{ripeness}|{int(shutdown)}|{time1:.3f}|{elapsed_time:.3f}|{motor_update}\n"
    #     print("sending blueberry packet")
    #     print(data_packet)
    #     #data_packet = f"{motor_speed}|{belt_num}|{ripeness}|{int(shutdown)}|{time1}|{elapsed_time:.3f}\n"
    #     ser.write(data_packet.encode())

    if not process_queue.empty():
            while not process_queue.empty():
                blueberry = process_queue.get()  # Thread-safe dequeue
                # Process each blueberry
                #print(f"Processing: {blueberry}")
                time1 = blueberry.actuation_time
                belt_num = blueberry.belt
                ripeness= blueberry.ripeness
                motor_update =0

                if blueberry.motor_update ==1:
                    motor_update = 1
                    motor_speed = blueberry.motor_speed
                else:
                    motor_speed = 0.0 #goal in/s

                data_packet = f"{motor_speed:.3f}|{belt_num}|{ripeness}|{int(shutdown)}|{time1:.3f}|{elapsed_time:.3f}|{motor_update}\n"
                print("sending blueberry packet")
                print(data_packet)
                #print(f"queue size is {process_queue.qsize()}")
                #data_packet = f"{motor_speed}|{belt_num}|{ripeness}|{int(shutdown)}|{time1}|{elapsed_time:.3f}\n"
                #ser.write(data_packet.encode())
                ser.write(f"{data_packet}".encode('utf-8'))



