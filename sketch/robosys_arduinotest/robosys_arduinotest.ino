#include <Servo.h>

Servo servo1, servo2, servo3;

void setup() {
    Serial.begin(9600);
    servo1.attach(9);  // Example pin numbers
    servo2.attach(10);
    servo3.attach(11);
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
    if (Serial.available() > 0) {
        String data = Serial.readStringUntil('\n');
        int motor_speed, angle1, angle2, angle3, shutdown, time1, time2, time3;
        float elapsed_time;

        sscanf(data.c_str(), "%d|%d|%d|%d|%d|%d|%d|%d|%f", &motor_speed, &angle1, &angle2, &angle3, &shutdown, &time1, &time2, &time3, &elapsed_time);

        // Use parsed values for control logic
        if (shutdown) {
            // Handle shutdown
        } else {
            // Set motor speed, servo angles, etc. 
            digitalWrite(LED_BUILTIN, HIGH);
            servo1.write(angle1);
            servo2.write(angle2);
            servo3.write(angle3);
            delay(500);
            digitalWrite(LED_BUILTIN, LOW);
            delay(500);
            
        }
    }
}
