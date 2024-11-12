#include <Servo.h>
#include <CircularBuffer.hpp>

constexpr int arraySize = 30;
Servo servo1, servo2, servo3;
unsigned long first_time;

// Belt solenoids
bool sol10_waiting, sol11_waiting;
CircularBuffer<float, arraySize> sol1_u, sol1_o;

bool sol20_waiting, sol21_waiting;
CircularBuffer<float, arraySize> sol2_u, sol2_o;

bool sol30_waiting, sol31_waiting;
CircularBuffer<float, arraySize> sol3_u, sol3_o;

void setup() {
    Serial.begin(9600);
    servo1.attach(9);
    servo2.attach(10);
    servo3.attach(11);
    pinMode(LED_BUILTIN, OUTPUT);
    first_time = millis();
}

void insertSorted(CircularBuffer<float, arraySize>& sortedBuffer, float value) {
    CircularBuffer<float, arraySize> tempBuffer;

    bool valueInserted = false;

    while (!sortedBuffer.isEmpty()) {
        float current = sortedBuffer.shift();

        if (!valueInserted && current > value) {
            tempBuffer.push(value); // Insert the new value before larger elements
            valueInserted = true;
        }

        tempBuffer.push(current); // Add current element
    }

    if (!valueInserted) {
        tempBuffer.push(value); // Add value if it’s the largest
    }

    // Restore the sortedBuffer
    while (!tempBuffer.isEmpty()) {
        sortedBuffer.push(tempBuffer.shift());
    }
}



void update_solenoid_list(int belt, int ripeness, float actuation_time) {
    unsigned long current_time = millis() - first_time;
    float activation_time = current_time + actuation_time * 1000;

    switch (belt) {
        case 1:
            if (ripeness == -1) {
                insertSorted(sol1_u, activation_time);
                sol10_waiting = true;
            } else if (ripeness == 2) {
                insertSorted(sol1_o, activation_time);
                sol11_waiting = true;
            }
            break;

        case 2:
            if (ripeness == -1) {
                insertSorted(sol2_u, activation_time);
                sol20_waiting = true;
            } else if (ripeness == 2) {
                insertSorted(sol2_o, activation_time);
                sol21_waiting = true;
            }
            break;

        case 3:
            if (ripeness == -1) {
                insertSorted(sol3_u, activation_time);
                sol30_waiting = true;
            } else if (ripeness == 2) {
                insertSorted(sol3_o, activation_time);
                sol31_waiting = true;
            }
            break;

        default:
            break;
    }
}

void SerialUpdate() {
    String data = Serial.readStringUntil('\n');
    int motor_speed, belt, ripeness, actuation_time, shutdown, motor_update;
    float elapsed_time;

    sscanf(data.c_str(), "%d|%d|%d|%d|%d|%f|%d", &motor_speed, &belt, &ripeness, &actuation_time, &shutdown, &elapsed_time, &motor_update);

    if (shutdown) {
        // Handle shutdown
    } else if (motor_update == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    } else if (ripeness != 0) {
        update_solenoid_list(belt, ripeness, actuation_time);
    }
}

void ActivateSolenoid(int solenoid_num) {
    Serial.print("Activating solenoid: ");
    Serial.println(solenoid_num);
    // GPIO activation logic
}

void HandleSolenoidQueue(CircularBuffer<float, arraySize>& solenoid_array, bool& sol_waiting, int solenoid_num) {
    constexpr int threshold = 20; // in ms
    unsigned long current_time = millis() - first_time;

    if (!solenoid_array.isEmpty()) {
        if (current_time >= solenoid_array[0] - threshold && current_time <= solenoid_array[0] + threshold) {
            ActivateSolenoid(solenoid_num);
            solenoid_array.shift();
        } else if (current_time > solenoid_array[0] + threshold) {
            solenoid_array.shift(); // Remove outdated entries
        }
        sol_waiting = !solenoid_array.isEmpty();
    }
}

void HandleAllSolenoids() {
    if (sol10_waiting) HandleSolenoidQueue(sol1_u, sol10_waiting, 10);
    if (sol11_waiting) HandleSolenoidQueue(sol1_o, sol11_waiting, 11);
    if (sol20_waiting) HandleSolenoidQueue(sol2_u, sol20_waiting, 20);
    if (sol21_waiting) HandleSolenoidQueue(sol2_o, sol21_waiting, 21);
    if (sol30_waiting) HandleSolenoidQueue(sol3_u, sol30_waiting, 30);
    if (sol31_waiting) HandleSolenoidQueue(sol3_o, sol31_waiting, 31);
}

void loop() {
    if (Serial.available() > 0) {
        SerialUpdate();
    }
    HandleAllSolenoids();
}
