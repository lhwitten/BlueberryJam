//#include <Servo.h>
#include <CircularBuffer.hpp>

constexpr int arraySize = 30;
//Servo servo1, servo2, servo3;
unsigned long first_time;

// Belt solenoids
bool sol10_waiting, sol11_waiting;
CircularBuffer<float, arraySize> sol1_u, sol1_o;

bool sol20_waiting, sol21_waiting;
CircularBuffer<float, arraySize> sol2_u, sol2_o;

bool sol30_waiting, sol31_waiting;
CircularBuffer<float, arraySize> sol3_u, sol3_o;

const int SOL10_PIN = 11;
const int SOL11_PIN = 12;

bool solenoid_activated;
const int SOLENOID_ON = 20; //ms
float last_sol_time;

void setup() {
    Serial.begin(9600);
//    servo1.attach(9);
//    servo2.attach(10);
//    servo3.attach(11);
    //pins
    pinMode(LED_BUILTIN, OUTPUT);
    //solenoid pins
    pinMode(SOL10_PIN, OUTPUT);
    pinMode(SOL11_PIN, OUTPUT);
    
    //first time
    first_time = millis();
    last_sol_time = first_time;
    solenoid_activated = false;
    

    
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
    float activation_time = current_time + actuation_time;

    switch (belt) {
        case 1:
            if (ripeness == -1) {
                insertSorted(sol1_u, activation_time);
                sol10_waiting = true;
                Serial.println("Appending to Solenoid 10");
                //printCircularBuffer(sol1_u);
                
            } 
            else if (ripeness == 2) {
                insertSorted(sol1_o, activation_time);
                sol11_waiting = true;
                Serial.println("Appending to Solenoid 11");
                //printCircularBuffer(sol1_o);
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
bool parseSerialData(const String &data, float &motor_speed, int &belt, int &ripeness, int &shutdown, float &actuation_time, float &elapsed_time, int &motor_update) {
    // Create a copy of the input data
    String dataCopy = data;
    dataCopy.trim(); // Remove leading and trailing whitespace/newlines

    // Split the string by the '|' delimiter
    int fieldIndex = 0;
    String fields[7]; // There should be 7 fields

    while (dataCopy.length() > 0 && fieldIndex < 7) {
        int delimiterIndex = dataCopy.indexOf('|');
        if (delimiterIndex == -1) {
            // No more delimiters; take the rest of the string
            fields[fieldIndex++] = dataCopy;
            break;
        } else {
            // Extract the substring up to the delimiter
            fields[fieldIndex++] = dataCopy.substring(0, delimiterIndex);
            // Remove the extracted part from the data copy
            dataCopy = dataCopy.substring(delimiterIndex + 1);
        }
    }

    // If we don't have exactly 7 fields, parsing failed
    if (fieldIndex != 7) {
        return false;
    }

    // Convert each field to the appropriate type
    motor_speed = fields[0].toFloat();
    belt = fields[1].toInt();
    ripeness = fields[2].toInt();
    shutdown = fields[3].toInt();
    actuation_time = fields[4].toFloat();
    elapsed_time = fields[5].toFloat();
    motor_update = fields[6].toInt();

    // Parsing successful
    return true;
}


void SerialUpdate() {
String data = Serial.readStringUntil('\n');
    data.trim(); // Remove unnecessary whitespace
    Serial.print("Received raw data: ");
    Serial.println(data);
        int  belt, ripeness, shutdown_ard, motor_update;
    float motor_speed,elapsed_time,actuation_time;

//    int parsed = sscanf(data.c_str(), "%f|%d|%d|%d|%f|%f|%d",
//                        &motor_speed, &belt, &ripeness, &shutdown_ard, &actuation_time, &elapsed_time, &motor_update);
      bool custom_parse = parseSerialData(data, motor_speed, belt, ripeness, shutdown_ard, actuation_time, elapsed_time, motor_update);

    if (custom_parse) {
        Serial.println("Parsed values:");
        Serial.print("Motor Speed: "); Serial.println(motor_speed);
        Serial.print("Belt: "); Serial.println(belt);
        Serial.print("Ripeness: "); Serial.println(ripeness);
        Serial.print("Shutdown: "); Serial.println(shutdown_ard);
        Serial.print("Actuation Time: "); Serial.println(actuation_time);
        Serial.print("Elapsed Time: "); Serial.println(elapsed_time);
        Serial.print("Motor Update: "); Serial.println(motor_update);

        // Proceed with logic

      //    Serial.println();
          if (shutdown_ard) {
              // Handle shutdown TODO
          } else if (motor_update == 1) {
              digitalWrite(LED_BUILTIN, HIGH);
              delay(500);
              digitalWrite(LED_BUILTIN, LOW);
              delay(500);
          } else if (ripeness != 0) {
      //      Serial.println("about to append to solenoid list with Circbuffer:");
            float update_time = 1000*(actuation_time - elapsed_time); //put it in ms
              update_solenoid_list(belt, ripeness, update_time);
      
          }
    } else {
        Serial.print("Parsing failed. Fields parsed: ");
        Serial.println(custom_parse);
    }



}

void ActivateSolenoid(int solenoid_num) {
    Serial.print("Activating solenoid: ");
    Serial.println(solenoid_num);
    // GPIO activation logic

    if (solenoid_num == 10){
      digitalWrite(SOL10_PIN,HIGH);
    }
    else if (solenoid_num == 11){
      digitalWrite(SOL11_PIN,HIGH);
    }

    solenoid_activated = true;
    last_sol_time = millis();
}

void SolenoidsOff() {
    Serial.print("Deactivating solenoids: ");

      digitalWrite(SOL10_PIN,LOW);
      digitalWrite(SOL11_PIN,LOW);
      solenoid_activated = false;

}

void HandleSolenoidQueue(CircularBuffer<float, arraySize>& solenoid_array, bool& sol_waiting, int solenoid_num) {
    constexpr int threshold = 20; // in ms
    unsigned long current_time = millis() - first_time;

    if (!solenoid_array.isEmpty()) {
        if (current_time >= solenoid_array[0] - threshold && current_time <= solenoid_array[0] + threshold) {
            ActivateSolenoid(solenoid_num);
            solenoid_array.shift();
            Serial.println("Emptying Circular buff element. Newbuff:");
            printCircularBuffer(solenoid_array);
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

void printCircularBuffer(CircularBuffer<float, arraySize>& buf) {
  Serial.print("CircularBuffer Contents: ");
  for (size_t i = 0; i < buf.size(); i++) {
    Serial.print(buf[i]);  // Access each element with buf[index]
    if (i < buf.size() - 1) {
      Serial.print(", "); // Add a comma between elements
    }
  }
  Serial.println(); // Newline after printing all elements
}

void TestUpdate() {
    // Simulate data for testing
    float motor_speed = 0.5; // Example motor speed
    int belt = 1;           // Example belt (1, 2, or 3)
    int ripeness = 2;       // Example ripeness (-1 or 2)
    int shutdown = 0;       // Example shutdown flag (0 or 1)
    float actuation_time = random(1000, 5000) / 1000.0;; // Example actuation time in seconds
    float elapsed_time = 0.5;   // Example elapsed time in seconds
    int motor_update = 0;       // Example motor update flag (0 or 1)

    // Log the simulated data for clarity
//    Serial.println("TestUpdate: Simulated input values:");
//    Serial.print("Motor Speed: "); Serial.println(motor_speed);
//    Serial.print("Belt: "); Serial.println(belt);
//    Serial.print("Ripeness: "); Serial.println(ripeness);
//    Serial.print("Shutdown: "); Serial.println(shutdown);
//    Serial.print("Actuation Time: "); Serial.println(actuation_time);
//    Serial.print("Elapsed Time: "); Serial.println(elapsed_time);
//    Serial.print("Motor Update: "); Serial.println(motor_update);

    if (shutdown) {
        // Handle shutdown logic here if needed
        Serial.println("Shutdown initiated.");
    } else if (motor_update == 1) {
        digitalWrite(LED_BUILTIN, HIGH);
        delay(500);
        digitalWrite(LED_BUILTIN, LOW);
        delay(500);
    } else if (ripeness != 0) {
        //Serial.println("About to append to solenoid list with CircularBuffer:");
        float update_time =  (actuation_time - elapsed_time) * 1000; // Convert to ms
        update_solenoid_list(belt, ripeness, update_time);
        printCircularBuffer(sol1_o);
    }
}

int testCount = 0; // Counter to track the number of TestUpdate calls
const int maxTestCount = 20; // Maximum number of calls to TestUpdate
unsigned long lastPrintTime = 0; // Tracks the last time buffers were printed
const unsigned long printInterval = 3000; // 3 seconds interval

void loop() {
    while (Serial.available() > 0) {
        SerialUpdate();
    }
//      if (testCount < maxTestCount) {
//        TestUpdate(); // Call the test function
//        testCount++;  // Increment the counter
//    }

    HandleAllSolenoids();

    if (solenoid_activated && last_sol_time + SOLENOID_ON < millis()){
      SolenoidsOff();
    }
    
//    if (millis()-first_time % 1000 > 990){
      //printCircularBuffer(sol1_u);
      //printCircularBuffer(sol1_o);
//    }

    unsigned long currentMillis = millis();
    if (currentMillis - lastPrintTime >= printInterval) {
        lastPrintTime = currentMillis; // Update the last print time

        // Print the contents of the circular buffers
        Serial.println("Printing circular buffers:");
        Serial.print("Buffer sol1_u: ");
        printCircularBuffer(sol1_u);
        Serial.print("Buffer sol1_o: ");
        printCircularBuffer(sol1_o);
        Serial.print("Current Time: ");
        Serial.println(millis()-first_time);
        
        
    }

}