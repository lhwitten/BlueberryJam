# serial_comm.py
import serial
import threading
import time

#TODO write procedure for finding correct port
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port and baud rate as necessary

# Initialize variables with timestamps
motor_speed = 150
servo_angles = [90, 45, 135]
shutdown = False
actuation_times = [1000, 2000, 1500]
last_update_time = time.monotonic()

def send_data(motor_speed, servo_angles, shutdown, actuation_times, update_time):
    """
    Sends a data packet to the Arduino with timing information.
    """
    elapsed_time = time.monotonic() - update_time
    data_packet = f"{motor_speed}|{servo_angles[0]}|{servo_angles[1]}|{servo_angles[2]}|{int(shutdown)}|{actuation_times[0]}|{actuation_times[1]}|{actuation_times[2]}|{elapsed_time:.3f}\n"
    ser.write(data_packet.encode())

def async_serial_comm(interval=1):
    """
    Runs non-blocking serial communication to send data at specified intervals.
    """
    global last_update_time, motor_speed, servo_angles, shutdown, actuation_times
    while not shutdown:
        send_data(motor_speed, servo_angles, shutdown, actuation_times, last_update_time)
        time.sleep(interval)

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

def start_comm_thread(interval=1):
    """
    Starts the asynchronous communication thread.
    """
    comm_thread = threading.Thread(target=async_serial_comm, args=(interval,), daemon=True)
    comm_thread.start()
    return comm_thread