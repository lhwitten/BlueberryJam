import RPi.GPIO as GPIO
import threading
import time
# from RPLCD.i2c import CharLCD

# GPIO pin definitions
START_BUTTON_PIN = 5
STOP_BUTTON_PIN = 6

# Shared global variables for button state
start_pressed = False
stop_pressed = False

# Custom GPIO Pin Definitions
RS_pin = 24      # Register Select (Example: GPIO 17)
E_pin = 23       # Enable (Example: GPIO 27)
D4_pin = 22      # Data pin 4 (Example: GPIO 22)
D5_pin = 27      # Data pin 5 (Example: GPIO 23)
D6_pin = 18      # Data pin 6 (Example: GPIO 24)
D7_pin = 17      # Data pin 7 (Example: GPIO 25)

# Define LCD commands
LCD_CLEAR = 0x01
LCD_HOME = 0x02
LCD_CMD = 0
LCD_CHR = 1
LCD_LINE_1 = 0x80  # Address for line 1
LCD_LINE_2 = 0xC0  # Address for line 2


# Initialize GPIO
# GPIO.setmode(GPIO.BCM)
#GPIO.setup([RS_pin, E_pin, D4_pin, D5_pin, D6_pin, D7_pin], GPIO.OUT)


# lcd = CharLCD(pin_rs=RS_pin, pin_e=E_pin, pins_data=[D4_pin, D5_pin, D6_pin, D7_pin],
#               numbering_mode=GPIO.BCM, cols=16, rows=2, dotsize=8)

# Function to display a message
# def lcd_display_message(line1, line2=""):
#     """
#     Display two lines of text on the LCD.
#     :param line1: First line text (max 16 chars)
#     :param line2: Second line text (max 16 chars)
#     """
#     lcd.clear()
#     lcd.cursor_pos = (0, 0)
#     lcd.write_string(line1[:16])  # Ensure no overflow beyond 16 chars

#     if line2:
#         lcd.cursor_pos = (1, 0)
#         lcd.write_string(line2[:16])

# # Function to cleanup GPIO
# def lcd_cleanup():
#     lcd.clear()
#     lcd.close()
#     GPIO.cleanup()


# GPIO Setup function
def setup_gpio():
    GPIO.setmode(GPIO.BCM)  # Use BCM GPIO numbering
    GPIO.setup(START_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)  # Pull-down for start button
    GPIO.setup(STOP_BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)   # Pull-down for stop button
    GPIO.setup([RS_pin, E_pin, D4_pin, D5_pin, D6_pin, D7_pin], GPIO.OUT)

# Thread function to monitor button presses
def button_monitor():
    global start_pressed, stop_pressed
    while True:
        # Check for button presses
        if GPIO.input(START_BUTTON_PIN) == GPIO.HIGH:
            print("Start button pressed!")
            start_pressed = True

        if GPIO.input(STOP_BUTTON_PIN) == GPIO.LOW:
            print("Stop button pressed!")
            stop_pressed = True
        
        time.sleep(0.05)  # Small delay to prevent high CPU usage

# Function to start the monitoring thread
def start_button_thread():
    setup_gpio()
    button_thread = threading.Thread(target=button_monitor, daemon=True)
    button_thread.start()
    print("Button monitoring thread started.")
    return button_thread

# Function to check button states (for main.py to call)
def were_buttons_pressed():
    global start_pressed, stop_pressed
    return start_pressed, stop_pressed

def reset_buttons():
    global start_pressed, stop_pressed
    start_pressed = False
    stop_pressed = False

# Cleanup GPIO on exit
def cleanup_gpio():
    GPIO.cleanup()



def lcd_init():
    lcd_send_byte(0x33, LCD_CMD)
    lcd_send_byte(0x32, LCD_CMD)
    lcd_send_byte(0x28, LCD_CMD)  # 4-bit mode, 2-line
    lcd_send_byte(0x0C, LCD_CMD)  # Display on, no cursor
    lcd_send_byte(0x06, LCD_CMD)  # Auto increment
    lcd_send_byte(LCD_CLEAR, LCD_CMD)
    time.sleep(0.2)

def lcd_send_byte(bits, mode):
    GPIO.output(RS_pin, mode)
    GPIO.output(D4_pin, bits & 0x10 == 0x10)
    GPIO.output(D5_pin, bits & 0x20 == 0x20)
    GPIO.output(D6_pin, bits & 0x40 == 0x40)
    GPIO.output(D7_pin, bits & 0x80 == 0x80)
    GPIO.output(E_pin, True)
    time.sleep(0.001)
    GPIO.output(E_pin, False)
    GPIO.output(D4_pin, bits & 0x01 == 0x01)
    GPIO.output(D5_pin, bits & 0x02 == 0x02)
    GPIO.output(D6_pin, bits & 0x04 == 0x04)
    GPIO.output(D7_pin, bits & 0x08 == 0x08)
    GPIO.output(E_pin, True)
    time.sleep(0.001)
    GPIO.output(E_pin, False)

def lcd_write(message,line_addr):
    lcd_send_byte(line_addr,LCD_CMD)
    for char in message:
        lcd_send_byte(ord(char), LCD_CHR)
def update_lcd(line1, line2):
    """
    Update both lines of the LCD.
    :param line1: String for line 1 (max 16 characters)
    :param line2: String for line 2 (max 16 characters)
    """
    lcd_send_byte(LCD_CLEAR, LCD_CMD)  # Clear the display
    time.sleep(0.2)  # Short delay for clearing
    lcd_write(line1[:16], LCD_LINE_1)  # Write to line 1 (truncate to 16 chars)
    lcd_write(line2[:16], LCD_LINE_2)  # Write to line 2 (truncate to 16 chars)

def construct_and_send_LCD(goal_speed,is_stopped,Error_code):
    line1 = "GOAL SPD " # 9 characters

    line1 += str(round(goal_speed,1)) #3 char
    line1 += "    "
    if is_stopped:
        line2 = "STOPPED " #  8 char
    else:
        line2 = "RUNNING "
    line2 += "ERR:" #4 char
    line2 += str(Error_code)
    update_lcd(line1,line2)
# # Main
# setup_gpio()
# lcd_init()
# # lcd_write("Hello, World!")
# # time.sleep(2)
# # GPIO.cleanup()

# goal = 1.2345
# is_stopped = True
# err_code = 15
# construct_and_send_LCD(goal,is_stopped,err_code)
# try:
#     while True:
#         err_code +=1
#         construct_and_send_LCD(goal,is_stopped,err_code)
#         time.sleep(3)
# except:
#     cleanup_gpio()