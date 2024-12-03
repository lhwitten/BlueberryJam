#include <CircularBuffer.hpp>


// Define the pins for the encoder
const int encoderPinA = 2; // Connect A to pin 2
const int encoderPinB = 3; // Connect B to pin 3

volatile int encoderPosition = 0; // Variable to store encoder position
volatile int lastEncoded = 0;     // To keep track of the last encoded value

volatile bool encoderUpdated = false; // Flag to indicate an update


CircularBuffer<float, 4> speedBuffer; // Circular buffer for 4 speed samples

unsigned long lastTime = 0;          // Last time the speed was calculated
int lastPosition = 0;                // Encoder position during the last check
float filteredSpeed = 0;             // Smoothed speed value

//motor
#define DR_MOTOR_ENA 6
#define DR_MOTOR_IN1 7
#define DR_MOTOR_IN2 8

//belt length is 700 mm
//belt diameter is 43 mm
const int BELT_RAD_IN = 0.8464567;
const int posPerRev = 600; // the number of encoder positions per revolution


void setup() {
  pinMode(encoderPinA, INPUT); // Set pinA as input with pull-up resistor
  pinMode(encoderPinB, INPUT); // Set pinB as input with pull-up resistor

  // Attach interrupts to the A and B channels
  attachInterrupt(digitalPinToInterrupt(encoderPinA), updateEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoderPinB), updateEncoder, CHANGE);

  Serial.begin(9600); // Start the serial monitor
}

float posPerSec2InPerSec(float posPerSec){
  //convert a Pos Per second measurement from the encoder to an inches per sec measurement on the belt
  float rev_per_sec = posPerSec / posPerRev;
  //circumference
  float inch_per_rev = 2*PI * BELT_RAD_IN;
  return inch_per_rev * rev_per_sec;
}

void update_encoder_properties(){

    unsigned long currentTime = millis(); // Get current time in milliseconds
  
  // Calculate speed every 100 ms
  if (currentTime - lastTime >= 100) {
    int positionChange = encoderPosition - lastPosition;
    float positionsPerSecond = positionChange / ((currentTime - lastTime) / 1000.0); // P/S calculation

    // Add the new speed measurement to the buffer
    speedBuffer.push(positionsPerSecond);

    // Compute the rolling average for smoothing
    filteredSpeed = 0;
    for (int i = 0; i < speedBuffer.size(); i++) {
      filteredSpeed += speedBuffer[i];
    }
    filteredSpeed /= speedBuffer.size();

    // Update time and position for the next calculation
    lastTime = currentTime;
    lastPosition = encoderPosition;

    // Print results
    Serial.print("Position: ");
    Serial.print(encoderPosition);
    Serial.print(" | Raw Speed (P/S): ");
    Serial.print(positionsPerSecond);
    Serial.print(" | Filtered Speed (P/S): ");
    Serial.println(filteredSpeed);
    //motor
      pinMode(DR_MOTOR_ENA, OUTPUT);
    pinMode(DR_MOTOR_IN1, OUTPUT);
    pinMode(DR_MOTOR_IN2, OUTPUT);
    // pinMode(VB_MOTOR_IN1, OUTPUT);
    // pinMode(VB_MOTOR_IN2, OUTPUT);
  
    // go forward full speed
    analogWrite(DR_MOTOR_ENA, 140);
    digitalWrite(DR_MOTOR_IN1, HIGH);
    digitalWrite(DR_MOTOR_IN2, LOW);
  }
}

void loop() {
  update_encoder_properties();
}

//void loop() {
//    static int lastState = LOW;
//    int currentState = digitalRead(pinA);
//    if (currentState != lastState) {
//        updateEncoder();
//        lastState = currentState;
//    }
//    if (encoderUpdated) {
//    encoderUpdated = false; // Reset the flag
//
//    // Print encoder position
//    Serial.print("Position: ");
//    Serial.println(encoderPosition);
//  }
//
//}



void updateEncoder() {
  // Read the current state of A and B
  int MSB = digitalRead(encoderPinA); // Most Significant Bit
  int LSB = digitalRead(encoderPinB); // Least Significant Bit

  int encoded = (MSB << 1) | LSB; // Combine A and B signals
  int sum = (lastEncoded << 2) | encoded; // Track transitions

  // Determine rotation direction and increment/decrement position
  if (sum == 0b1101 || sum == 0b0100 || sum == 0b0010 || sum == 0b1011)
    encoderPosition++;
  else if (sum == 0b1110 || sum == 0b0111 || sum == 0b0001 || sum == 0b1000)
    encoderPosition--;

  lastEncoded = encoded; // Update last encoded value
  encoderUpdated = true; // Set flag for processing in loop
}
